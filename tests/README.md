# Hardware Integration Tests

Host-side tests that communicate with the Pico 2 over USB MIDI and USB serial.

## Prerequisites

- Pico 2 connected via USB, running CircuitPython with the controller firmware
- USB MIDI enabled (default — `boot.py` does not disable it)
- USB CDC serial console enabled (default)
- Python 3.9+ on the host machine

## Setup

```bash
pip install -r tests/requirements.txt
```

## Running Tests

**Automated tests** (no physical interaction needed):
```bash
pytest tests/ -v --ignore=tests/test_wheels_buttons.py
```

**Manual tests** (requires pressing buttons / moving wheels):
```bash
pytest tests/test_wheels_buttons.py -v -s -m manual
```

**All tests:**
```bash
pytest tests/ -v -s
```

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `MIDI_DEVICE_NAME` | `CircuitPython Audio` | Substring to match in MIDI device names |
| `SERIAL_PORT` | Auto-detect | Serial port for REPL (e.g., `/dev/tty.usbmodem1234`) |

## Test Categories

| File | Tests | Requires |
|------|-------|----------|
| `test_midi_device.py` | USB MIDI enumeration, port open, thru | Device connected |
| `test_config.py` | Config load/save/validation via REPL | Device in dev mode (writable FS) |
| `test_error_recovery.py` | Corrupt config, MIDI flood | Device in dev mode |
| `test_wheels_buttons.py` | Physical input verification | Human interaction |

## Notes

- Config tests require the device to be in **dev mode** (hold Button A during boot) so the filesystem is writable from both the REPL and USB.
- The REPL tests interrupt the running `code.py` with Ctrl-C. After running tests, reset the device to resume normal operation.
