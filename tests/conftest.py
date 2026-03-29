# conftest.py — pytest fixtures for hardware integration tests.
#
# Provides session-scoped MIDI I/O and serial REPL connections to the
# Pico 2 running CircuitPython. Tests communicate with the device over:
#   - USB MIDI (via mido + python-rtmidi) for MIDI message verification
#   - USB serial (via pyserial) for REPL access and config inspection

import os
import time

import pytest

MIDI_DEVICE_NAME = os.environ.get("MIDI_DEVICE_NAME", "CircuitPython Audio")
SERIAL_PORT = os.environ.get("SERIAL_PORT", "")


@pytest.fixture(scope="session")
def midi_out():
    """Open a MIDI output port to the Pico (sends TO the device)."""
    import mido
    for name in mido.get_output_names():
        if MIDI_DEVICE_NAME in name:
            port = mido.open_output(name)
            yield port
            port.close()
            return
    pytest.skip(f"MIDI device '{MIDI_DEVICE_NAME}' not found")


@pytest.fixture(scope="session")
def midi_in():
    """Open a MIDI input port from the Pico (receives FROM the device)."""
    import mido
    for name in mido.get_input_names():
        if MIDI_DEVICE_NAME in name:
            port = mido.open_input(name)
            yield port
            port.close()
            return
    pytest.skip(f"MIDI device '{MIDI_DEVICE_NAME}' not found")


def _find_serial_port():
    """Auto-detect the CircuitPython serial port."""
    import serial.tools.list_ports
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "") + (p.manufacturer or "")
        if "CircuitPython" in desc or "Pico" in desc:
            return p.device
    return None


@pytest.fixture(scope="session")
def serial_repl():
    """Open a serial connection to the Pico's CircuitPython REPL."""
    import serial
    port = SERIAL_PORT or _find_serial_port()
    if not port:
        pytest.skip("CircuitPython serial REPL not found")
    ser = serial.Serial(port, 115200, timeout=2)
    time.sleep(0.5)
    yield ser
    ser.close()


@pytest.fixture(scope="session")
def repl(serial_repl):
    """Helper to send Python commands to the Pico REPL and read output.

    Uses CircuitPython paste mode (Ctrl-E / Ctrl-D) to reliably send
    multi-line and multi-statement commands.
    """

    class ReplHelper:
        def __init__(self, ser):
            self._ser = ser
            self._ensure_repl()

        def _ensure_repl(self):
            """Interrupt any running program and get to the REPL prompt."""
            # Ctrl-C twice to interrupt code.py
            self._ser.write(b"\x03")
            time.sleep(0.3)
            self._ser.write(b"\x03")
            time.sleep(0.5)
            self._ser.read_all()  # Discard
            # Ensure lib/ is on the import path
            self._raw_paste("import sys\nif '/lib' not in sys.path:\n sys.path.insert(0, '/lib')\n")
            time.sleep(0.3)
            self._ser.read_all()  # Discard

        def _raw_paste(self, code):
            """Send code using CircuitPython paste mode (Ctrl-E / Ctrl-D)."""
            self._ser.write(b"\x05")  # Ctrl-E = enter paste mode
            time.sleep(0.1)
            self._ser.read_all()  # Discard "paste mode" banner
            self._ser.write(code.encode())
            time.sleep(0.1)
            self._ser.write(b"\x04")  # Ctrl-D = execute paste
            time.sleep(0.8)

        def exec(self, command):
            """Send a command via paste mode and return the output.

            The command can contain semicolons or newlines — both work
            in paste mode.
            """
            self._ser.read_all()  # Clear pending data
            # Convert semicolons to newlines for paste mode
            code = command.replace("; ", "\n").replace(";", "\n")
            self._raw_paste(code + "\n")
            raw = self._ser.read_all().decode(errors="replace")
            # Parse: filter out paste mode markers and prompts
            lines = raw.replace("\r\n", "\n").replace("\r", "\n").split("\n")
            result_lines = []
            for line in lines:
                stripped = line.strip()
                if not stripped:
                    continue
                # Skip paste mode markers, prompts, echoed code
                if stripped.startswith(">>>") or stripped.startswith("..."):
                    continue
                if stripped.startswith("paste mode"):
                    continue
                if stripped == "===":
                    continue
                result_lines.append(stripped)
            return "\n".join(result_lines)

        def get_config(self, dotpath):
            """Read a config value from the device."""
            output = self.exec(
                f"from config import ConfigManager\n"
                f"c = ConfigManager('config.json')\n"
                f"c.load()\n"
                f"print(repr(c.get('{dotpath}')))"
            )
            # The print output should be the last meaningful line
            lines = [l.strip() for l in output.strip().split("\n") if l.strip()]
            return lines[-1] if lines else None

    return ReplHelper(serial_repl)
