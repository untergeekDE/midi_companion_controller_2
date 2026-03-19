# CircuitPython Pico 2 Port — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the Teensy 3.2 MIDI companion controller to Raspberry Pi Pico 2 running CircuitPython, with a modular architecture, JSON-based config, USB Host MIDI input, and DIN MIDI output.

**Architecture:** HAL layer (`hardware/`) wraps each physical component as a class. Feature modules (`config.py`, `midi_merge.py`, `menu.py`, `calibration.py`) contain all business logic. `code.py` wires everything together with zero logic. `boot.py` handles filesystem and USB setup.

**Tech Stack:** CircuitPython 9.x, adafruit_midi, adafruit_max3421e, adafruit_usb_host_midi, adafruit_debouncer, adafruit_displayio_ssd1306, adafruit_display_text

**Spec:** `docs/superpowers/specs/2026-03-19-circuitpython-pico2-port-design.md`

**Testing approach:** This is an embedded microcontroller project — no pytest. Each task includes an on-device verification script or REPL commands to confirm the module works on real hardware before moving on.

---

## File Map

| File | Responsibility | Created in Task |
|------|---------------|-----------------|
| `circuitpython/boot.py` | Filesystem remount, disable USB MIDI/HID, dev-mode button | 1 |
| `circuitpython/lib/hardware/pins.py` | All GPIO pin constants | 1 |
| `circuitpython/lib/config.py` | JSON config read/write/defaults/validation | 2 |
| `circuitpython/config.json` | Default configuration file | 2 |
| `circuitpython/lib/hardware/leds.py` | StatusLED class | 3 |
| `circuitpython/lib/hardware/buttons.py` | DebouncedButton with double-click | 4 |
| `circuitpython/lib/hardware/wheels.py` | AnalogWheel with calibration + deadband | 5 |
| `circuitpython/lib/hardware/display.py` | OledDisplay via displayio + SSD1306 | 6 |
| `circuitpython/lib/hardware/midi_uart.py` | UartMidi DIN MIDI In/Out | 7 |
| `circuitpython/lib/hardware/midi_usb_host.py` | UsbHostMidi via MAX3421E | 8 |
| `circuitpython/lib/midi_merge.py` | MidiMerge: N inputs → M outputs | 9 |
| `circuitpython/lib/calibration.py` | WheelCalibration interactive routine | 10 |
| `circuitpython/lib/menu.py` | OnDeviceMenu with all config options | 11 |
| `circuitpython/code.py` | Main entry point: setup + loop | 12 |

---

### Task 1: boot.py and pins.py — Foundation

**Files:**
- Create: `circuitpython/boot.py`
- Create: `circuitpython/lib/hardware/__init__.py`
- Create: `circuitpython/lib/hardware/pins.py`
- Create: `circuitpython/lib/__init__.py`

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p circuitpython/lib/hardware
```

- [ ] **Step 2: Write `boot.py`**

Create `circuitpython/boot.py`:

```python
# boot.py — Runs before code.py on every boot.
#
# Responsibilities:
# 1. Remount the CIRCUITPY filesystem as writable by CircuitPython code,
#    so that ConfigManager can save config.json. Side effect: the drive
#    becomes read-only when connected to a computer via USB.
# 2. Disable USB MIDI device and USB HID — the Pico should not appear
#    as a MIDI device or HID to a connected computer.
# 3. Dev-mode escape hatch: if Button A (the designated button) is held
#    during boot, skip the remount so the filesystem stays writable by
#    USB (for editing code/config from a PC).

import board
import digitalio
import storage
import usb_midi
import usb_hid

# Check if Button A is held during boot (dev-mode escape hatch).
# Button A is on the pin defined in pins.py — we hardcode it here because
# pins.py hasn't been imported yet (boot.py runs before code.py).
# Pin must match hardware.pins.BUTTON_A.
dev_mode_pin = digitalio.DigitalInOut(board.GP17)
dev_mode_pin.direction = digitalio.Direction.INPUT
dev_mode_pin.pull = digitalio.Pull.UP

dev_mode = not dev_mode_pin.value  # Active low: pressed = False = dev_mode True
dev_mode_pin.deinit()

if not dev_mode:
    # Normal operation: filesystem writable by CircuitPython, read-only to USB host.
    storage.remount("/", readonly=False)

# Always disable USB MIDI device and USB HID — we use DIN MIDI only for output,
# and USB Host (MAX3421E) for input. The native USB port is power + CIRCUITPY only.
usb_midi.disable()
usb_hid.disable()
```

- [ ] **Step 3: Write `pins.py`**

Create `circuitpython/lib/hardware/pins.py`:

```python
# pins.py — Central pin definitions for the MIDI Companion Controller 2.
#
# This is the ONLY file where GPIO pin numbers are defined. All other modules
# import from here. If you rewire the hardware, change ONLY this file.
#
# Pin assignments assume a Raspberry Pi Pico 2 (RP2350).
# Adjust these if using a different board or wiring.

import board

# --- Analog Wheels ---
# Wheel A: left wheel, typically pitch bend
WHEEL_A = board.A0
# Wheel B: right wheel, typically mod wheel (CC1)
WHEEL_B = board.A1

# --- Buttons (active low, INPUT_PULLUP) ---
# Button A: left button (also triggers menu on double-click)
BUTTON_A = board.GP17
# Button B: right button (confirm/set in menu)
BUTTON_B = board.GP15
# Footswitch: external footswitch jack
FOOTSWITCH = board.GP13

# --- LEDs ---
# LED1: signals MIDI input activity
LED1 = board.GP2
# LED2: signals MIDI output activity (locally generated)
LED2 = board.GP3

# --- I2C (SSD1306 OLED Display) ---
I2C_SCL = board.GP21
I2C_SDA = board.GP20

# --- UART (DIN MIDI In/Out) ---
# Standard MIDI baud rate is 31250. The UART pins connect to the
# optocoupler circuit (or Adafruit MIDI FeatherWing).
UART_TX = board.GP4
UART_RX = board.GP5

# --- SPI (MAX3421E USB Host Breakout) ---
SPI_CLK = board.GP18
SPI_MOSI = board.GP19
SPI_MISO = board.GP16
USB_HOST_CS = board.GP22
USB_HOST_INT = board.GP26  # Interrupt pin for MAX3421E (optional but recommended)
```

- [ ] **Step 4: Create `__init__.py` files**

Create empty `circuitpython/lib/__init__.py` and `circuitpython/lib/hardware/__init__.py`.

- [ ] **Step 5: Verify on device**

Copy `circuitpython/` contents to CIRCUITPY drive. Open REPL (serial console) and run:

```python
from hardware import pins
print(pins.WHEEL_A)
print(pins.BUTTON_A)
print(pins.I2C_SCL)
# Should print pin objects without errors
```

- [ ] **Step 6: Commit**

```bash
git add circuitpython/boot.py circuitpython/lib/hardware/pins.py circuitpython/lib/__init__.py circuitpython/lib/hardware/__init__.py
git commit -m "feat: add boot.py and pins.py foundation for CircuitPython port"
```

---

### Task 2: ConfigManager — JSON Configuration

**Files:**
- Create: `circuitpython/lib/config.py`
- Create: `circuitpython/config.json`

- [ ] **Step 1: Write `config.py`**

Create `circuitpython/lib/config.py`:

```python
# config.py — JSON-based configuration manager.
#
# Replaces the Teensy's EEPROM+checksum system with a human-readable JSON file
# stored on the CIRCUITPY flash filesystem.
#
# Usage:
#     config = ConfigManager("config.json")
#     config.load()                       # Read file, or use defaults if missing/corrupt
#     value = config.get("midi.tx_channel")  # Dot-notation access
#     config.set("midi.tx_channel", 2)       # Update in memory
#     config.save()                          # Persist to flash
#
# The filesystem must be writable by CircuitPython (see boot.py).

import json


# Default configuration — used when config.json is missing or corrupt.
# These values match the original Teensy EEPROM defaults.
DEFAULTS = {
    "midi": {
        "tx_channel": 1,
        "thru_enabled": False
    },
    "wheel_a": {
        "mode": "pitch_bend",
        "cc_number": None,
        "calibration": {
            "delta": 517,
            "center": 658
        }
    },
    "wheel_b": {
        "mode": "cc",
        "cc_number": 1,
        "calibration": {
            "delta": 700,
            "center": 909
        }
    },
    "button_a": {
        "mode": "cc",
        "cc_number": 64
    },
    "button_b": {
        "mode": "aftertouch",
        "aftertouch_value": 64
    },
    "footswitch": {
        "mode": "cc",
        "cc_number": 66
    },
    "deadband": 10
}

# Validation rules: (min, max) inclusive. Only numeric fields need range checks.
# Keys use dot-notation matching config.get/set paths.
VALIDATION = {
    "midi.tx_channel": (1, 16),
    "wheel_a.cc_number": (0, 127),
    "wheel_b.cc_number": (0, 127),
    "button_a.cc_number": (0, 127),
    "button_b.cc_number": (0, 127),
    "button_b.aftertouch_value": (0, 127),
    "footswitch.cc_number": (0, 127),
    "deadband": (0, 63),
}


def _deep_copy(obj):
    """Deep copy a nested dict/list structure without the copy module."""
    if isinstance(obj, dict):
        return {k: _deep_copy(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_deep_copy(item) for item in obj]
    return obj


class ConfigManager:
    """Manages reading, writing, and validating the JSON configuration file.

    Attributes:
        path: Filesystem path to the config.json file.
        data: The current configuration dictionary (in memory).
    """

    def __init__(self, path):
        self.path = path
        self.data = _deep_copy(DEFAULTS)

    def load(self):
        """Load configuration from the JSON file.

        Falls back to built-in defaults if the file is missing, empty,
        or contains invalid JSON. Merges loaded data over defaults so
        that any missing keys get default values.
        """
        try:
            with open(self.path, "r") as f:
                loaded = json.load(f)
            # Merge loaded values over a fresh copy of defaults.
            # This ensures new keys added in future firmware versions
            # get their default values even if the config file is old.
            self.data = _deep_copy(DEFAULTS)
            self._merge(self.data, loaded)
            self._validate()
        except (OSError, ValueError, KeyError):
            # File missing, empty, or corrupt — use defaults.
            self.data = _deep_copy(DEFAULTS)

    def save(self):
        """Write the current configuration to the JSON file.

        Requires the filesystem to be writable (see boot.py).
        Silently fails if the filesystem is read-only (dev mode).
        """
        try:
            with open(self.path, "w") as f:
                json.dump(self.data, f)
        except OSError:
            # Filesystem is read-only (dev mode) — silently skip.
            pass

    def get(self, dotpath):
        """Get a configuration value using dot-notation.

        Example: config.get("midi.tx_channel") returns 1.
        """
        keys = dotpath.split(".")
        obj = self.data
        for key in keys:
            obj = obj[key]
        return obj

    def set(self, dotpath, value):
        """Set a configuration value using dot-notation.

        Example: config.set("midi.tx_channel", 2).
        Updates in memory only — call save() to persist.
        """
        keys = dotpath.split(".")
        obj = self.data
        for key in keys[:-1]:
            obj = obj[key]
        obj[keys[-1]] = value

    def _merge(self, base, override):
        """Recursively merge override dict into base dict.

        Only updates keys that exist in both and have matching types,
        or adds keys from override that don't exist in base.
        """
        for key, value in override.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                self._merge(base[key], value)
            else:
                base[key] = value

    def _validate(self):
        """Clamp numeric values to their valid ranges.

        Values outside the valid range are clamped (not rejected),
        so the controller always boots with usable settings.
        """
        for dotpath, (lo, hi) in VALIDATION.items():
            try:
                val = self.get(dotpath)
                if val is not None and isinstance(val, (int, float)):
                    clamped = max(lo, min(hi, int(val)))
                    if clamped != val:
                        self.set(dotpath, clamped)
            except (KeyError, TypeError):
                pass
```

- [ ] **Step 2: Write default `config.json`**

Create `circuitpython/config.json`:

```json
{
    "midi": {
        "tx_channel": 1,
        "thru_enabled": false
    },
    "wheel_a": {
        "mode": "pitch_bend",
        "cc_number": null,
        "calibration": {
            "delta": 517,
            "center": 658
        }
    },
    "wheel_b": {
        "mode": "cc",
        "cc_number": 1,
        "calibration": {
            "delta": 700,
            "center": 909
        }
    },
    "button_a": {
        "mode": "cc",
        "cc_number": 64
    },
    "button_b": {
        "mode": "aftertouch",
        "aftertouch_value": 64
    },
    "footswitch": {
        "mode": "cc",
        "cc_number": 66
    },
    "deadband": 10
}
```

- [ ] **Step 3: Verify on device (or desktop Python)**

ConfigManager uses only `json` and file I/O — it can be tested on desktop Python:

```bash
cd circuitpython
python3 -c "
import sys; sys.path.insert(0, 'lib')
from config import ConfigManager

# Test defaults
c = ConfigManager('/tmp/test_config.json')
c.load()
assert c.get('midi.tx_channel') == 1
assert c.get('wheel_a.mode') == 'pitch_bend'
assert c.get('deadband') == 10

# Test set + save + reload
c.set('midi.tx_channel', 5)
c.save()
c2 = ConfigManager('/tmp/test_config.json')
c2.load()
assert c2.get('midi.tx_channel') == 5

# Test validation clamping
c.set('midi.tx_channel', 99)
c._validate()
assert c.get('midi.tx_channel') == 16

# Test corrupt file fallback
with open('/tmp/test_corrupt.json', 'w') as f:
    f.write('not json')
c3 = ConfigManager('/tmp/test_corrupt.json')
c3.load()
assert c3.get('midi.tx_channel') == 1  # fell back to defaults

print('All config tests passed.')
"
```

- [ ] **Step 4: Commit**

```bash
git add circuitpython/lib/config.py circuitpython/config.json
git commit -m "feat: add ConfigManager with JSON config, defaults, and validation"
```

---

### Task 3: StatusLED — Simplest Hardware Module

**Files:**
- Create: `circuitpython/lib/hardware/leds.py`

- [ ] **Step 1: Write `leds.py`**

Create `circuitpython/lib/hardware/leds.py`:

```python
# leds.py — Simple LED control for MIDI activity indicators.
#
# LED1 signals MIDI input activity (blinks when data arrives).
# LED2 signals MIDI output activity (blinks when controller sends data).
#
# Usage:
#     from hardware.leds import StatusLED
#     from hardware import pins
#     led = StatusLED(pins.LED1)
#     led.on()
#     led.off()

import digitalio


class StatusLED:
    """Controls a single status LED.

    The LED is driven as a simple digital output. Turn it on at the start
    of an event (MIDI in/out) and off at the end of the main loop cycle
    to create a brief activity blink.

    Args:
        pin: A board pin object (e.g., board.GP16).
    """

    def __init__(self, pin):
        self._led = digitalio.DigitalInOut(pin)
        self._led.direction = digitalio.Direction.OUTPUT
        self._led.value = False

    def on(self):
        """Turn the LED on."""
        self._led.value = True

    def off(self):
        """Turn the LED off."""
        self._led.value = False
```

- [ ] **Step 2: Verify on device**

Copy to CIRCUITPY. In REPL:

```python
from hardware.leds import StatusLED
from hardware import pins
led1 = StatusLED(pins.LED1)
led2 = StatusLED(pins.LED2)
led1.on()   # LED1 should light up
led2.on()   # LED2 should light up
import time; time.sleep(1)
led1.off()
led2.off()  # Both should turn off
```

- [ ] **Step 3: Commit**

```bash
git add circuitpython/lib/hardware/leds.py
git commit -m "feat: add StatusLED hardware module"
```

---

### Task 4: DebouncedButton — Button Input with Double-Click

**Files:**
- Create: `circuitpython/lib/hardware/buttons.py`

- [ ] **Step 1: Write `buttons.py`**

Create `circuitpython/lib/hardware/buttons.py`:

```python
# buttons.py — Debounced button input with double-click detection.
#
# Wraps adafruit_debouncer to provide clean button state with edge detection
# (fell/rose) and double-click detection via timestamp comparison.
#
# All buttons use INPUT_PULLUP, so pressed = LOW = False.
# "fell" means the signal fell from HIGH to LOW = button was just pressed.
# "rose" means the signal rose from LOW to HIGH = button was just released.
#
# Usage:
#     from hardware.buttons import DebouncedButton
#     from hardware import pins
#     btn = DebouncedButton(pins.BUTTON_A, double_click_ms=250)
#     while True:
#         btn.update()
#         if btn.fell:
#             print("pressed")
#         if btn.double_click:
#             print("double-click!")

import digitalio
import time
from adafruit_debouncer import Debouncer


class DebouncedButton:
    """A debounced button with edge detection and double-click support.

    Args:
        pin: A board pin object. Configured as INPUT_PULLUP internally.
        debounce_s: Debounce interval in seconds (default 0.01 = 10ms).
        double_click_ms: Time window in milliseconds for double-click
            detection (default 250). Set to 0 to disable double-click.
    """

    def __init__(self, pin, debounce_s=0.01, double_click_ms=250):
        # Set up the physical pin with pull-up resistor.
        self._pin = digitalio.DigitalInOut(pin)
        self._pin.direction = digitalio.Direction.INPUT
        self._pin.pull = digitalio.Pull.UP

        # Wrap in Debouncer for clean state transitions.
        self._debouncer = Debouncer(self._pin, interval=debounce_s)

        # Double-click detection state.
        self._double_click_ms = double_click_ms
        self._last_fell_time = 0  # monotonic_ns of last fell event
        self._double_click = False

    def update(self):
        """Sample the button and update state. Call once per main loop iteration.

        After calling update(), check .fell, .rose, and .double_click
        for the current cycle's events.
        """
        self._double_click = False
        self._debouncer.update()

        # Double-click detection: if we just got a fell event and the
        # previous fell was within the double-click window, flag it.
        if self._debouncer.fell and self._double_click_ms > 0:
            now = time.monotonic_ns()
            elapsed_ms = (now - self._last_fell_time) // 1_000_000
            if self._last_fell_time > 0 and elapsed_ms < self._double_click_ms:
                self._double_click = True
                self._last_fell_time = 0  # Reset so triple-click doesn't re-trigger.
            else:
                self._last_fell_time = now

    @property
    def fell(self):
        """True if the button was just pressed this cycle (HIGH→LOW transition)."""
        return self._debouncer.fell

    @property
    def rose(self):
        """True if the button was just released this cycle (LOW→HIGH transition)."""
        return self._debouncer.rose

    @property
    def double_click(self):
        """True if a double-click was detected this cycle."""
        return self._double_click

    @property
    def pressed(self):
        """True if the button is currently held down (steady state)."""
        return not self._debouncer.value  # Active low: pressed = False = True here.
```

- [ ] **Step 2: Verify on device**

In REPL:

```python
from hardware.buttons import DebouncedButton
from hardware import pins
btn = DebouncedButton(pins.BUTTON_A)
while True:
    btn.update()
    if btn.fell:
        print("PRESSED")
    if btn.rose:
        print("RELEASED")
    if btn.double_click:
        print("DOUBLE CLICK!")
```

Press Button A a few times — verify single press and double-click detection. Ctrl-C to exit.

- [ ] **Step 3: Commit**

```bash
git add circuitpython/lib/hardware/buttons.py
git commit -m "feat: add DebouncedButton with double-click detection"
```

---

### Task 5: AnalogWheel — Wheel Input with Calibration

**Files:**
- Create: `circuitpython/lib/hardware/wheels.py`

- [ ] **Step 1: Write `wheels.py`**

Create `circuitpython/lib/hardware/wheels.py`:

```python
# wheels.py — Analog wheel input with calibration and deadband.
#
# Reads a potentiometer/wheel via an analog input pin and converts the raw
# ADC value into a MIDI-ready value. Supports two modes:
#
# - "pitch_bend" (centered): Returns -8192..8191 (14-bit signed).
#   Uses the 'center' calibration value as zero reference.
#   Deadband is applied symmetrically around center.
#
# - "cc" (uncentered): Returns 0..127 (7-bit unsigned).
#   Uses the 'delta' calibration value as zero/low reference.
#   Deadband is applied above the zero position.
#
# The potentiometers in the wheel box have roughly a quarter of the full
# ADC range of travel, so we assume ~256 raw counts of usable range
# (out of the CircuitPython 16-bit ADC range of 0..65535).
#
# Usage:
#     from hardware.wheels import AnalogWheel
#     from hardware import pins
#     wheel = AnalogWheel(pins.WHEEL_A, mode="pitch_bend", deadband=10)
#     wheel.calibrate(center=658, delta=517)
#     value = wheel.read()

import analogio


# CircuitPython ADC returns 16-bit values (0..65535). The original Teensy
# code used 10-bit (0..1023). We scale down to 10-bit equivalent for
# compatibility with the original calibration values and algorithm.
_ADC_SHIFT = 6  # Right-shift 16-bit to 10-bit: 65535 >> 6 ≈ 1023


class AnalogWheel:
    """Reads an analog wheel/potentiometer with calibration and deadband.

    Args:
        pin: A board analog pin object (e.g., board.A0).
        mode: "pitch_bend" for centered wheel, "cc" for uncentered.
        deadband: Width of the dead zone around center/zero. Shared global
            value from config. Range 0..63.
    """

    def __init__(self, pin, mode="pitch_bend", deadband=10):
        self._adc = analogio.AnalogIn(pin)
        self.mode = mode
        self.deadband = deadband

        # Calibration values (raw 10-bit scale, matching original firmware).
        # Set via calibrate() or loaded from config.
        self._center = 512  # Mid-point for pitch bend mode.
        self._delta = 0     # Zero offset for CC mode.

        # State tracking for change detection.
        self._value = 0
        self._changed = False

    def calibrate(self, center=512, delta=0):
        """Set calibration values.

        Args:
            center: The raw ADC reading (10-bit scale) when the pitch bend
                wheel is at rest in the center position.
            delta: The raw ADC reading (10-bit scale) when the CC wheel
                is at its lowest/zero position.
        """
        self._center = center
        self._delta = delta

    def read(self):
        """Read the wheel and return a MIDI-ready value.

        Returns:
            int: -8192..8191 for pitch_bend mode, 0..127 for cc mode.
            Also updates .value and .changed properties.
        """
        # Read 16-bit ADC and scale down to 10-bit for algorithm compatibility.
        raw = self._adc.value >> _ADC_SHIFT

        if self.mode == "pitch_bend":
            new_value = self._read_centered(raw)
        else:
            new_value = self._read_uncentered(raw)

        self._changed = (new_value != self._value)
        self._value = new_value
        return new_value

    def _read_centered(self, raw):
        """Process a centered wheel (pitch bend).

        Subtracts the center calibration, applies deadband, clamps to
        -128..127 range, then scales to 14-bit (-8192..8191) for
        adafruit_midi PitchBend compatibility.

        This matches the original readWheel() with centered=true,
        plus the *64 scaling from the original main loop.
        """
        v = raw - self._center

        if v > 0:
            if v < self.deadband:
                v = 0
            else:
                v -= self.deadband
            if v > 127:
                v = 127
        elif v < 0:
            if v > -self.deadband:
                v = 0
            else:
                v += self.deadband
            if v < -128:
                v = -128

        # Scale from 8-bit (-128..127) to 14-bit (-8192..8191).
        return v * 64

    def _read_uncentered(self, raw):
        """Process an uncentered wheel (CC / mod wheel).

        Subtracts the delta calibration, applies deadband, clamps to
        0..255 range, then divides by 2 to get 0..127 for MIDI CC.

        This matches the original readWheel() with centered=false.
        """
        v = raw - self._delta

        if v < self.deadband:
            v = 0
        else:
            v -= self.deadband
        if v > 255:
            v = 255

        # Divide by 2 to get 7-bit range (0..127).
        return v >> 1

    @property
    def value(self):
        """The most recently read value."""
        return self._value

    @property
    def changed(self):
        """True if the value changed on the last read() call."""
        return self._changed
```

- [ ] **Step 2: Verify on device**

In REPL:

```python
from hardware.wheels import AnalogWheel
from hardware import pins
# Test with pitch bend wheel
wa = AnalogWheel(pins.WHEEL_A, mode="pitch_bend", deadband=10)
wa.calibrate(center=658, delta=517)
# Move wheel and read values
for _ in range(20):
    v = wa.read()
    print(f"pitch: {v:6d}  changed: {wa.changed}")
    import time; time.sleep(0.2)
```

Verify: center position reads ~0, moving wheel produces negative/positive values up to -8192/8191.

- [ ] **Step 3: Commit**

```bash
git add circuitpython/lib/hardware/wheels.py
git commit -m "feat: add AnalogWheel with calibration, deadband, and 14-bit pitch bend"
```

---

### Task 6: OledDisplay — SSD1306 Display

**Files:**
- Create: `circuitpython/lib/hardware/display.py`

- [ ] **Step 1: Write `display.py`**

Create `circuitpython/lib/hardware/display.py`:

```python
# display.py — SSD1306 128x64 OLED display abstraction.
#
# Provides a high-level interface for the three display screens:
# 1. Workscreen: shows MIDI channel and current wheel values (normal operation)
# 2. Menu screen: shows configuration options (via menu.py)
# 3. Calibration screen: shows calibration instructions (via calibration.py)
#
# Uses displayio + adafruit_displayio_ssd1306 for hardware driving,
# and adafruit_display_text for text rendering.
#
# The display is rotated 180 degrees to match the physical mounting
# of the original Teensy build.
#
# Usage:
#     import busio
#     from hardware import pins
#     from hardware.display import OledDisplay
#     i2c = busio.I2C(pins.I2C_SCL, pins.I2C_SDA)
#     disp = OledDisplay(i2c)
#     disp.show_workscreen(channel=1)
#     disp.update_values(wheel_a=0, wheel_b=64)

import displayio
import terminalio
from adafruit_displayio_ssd1306 import SSD1306
from adafruit_display_text import label

# Display dimensions.
_WIDTH = 128
_HEIGHT = 64

# Character dimensions for the built-in terminalio.FONT (6x14 at scale 1).
# We use this font at different scales for the workscreen.
_FONT = terminalio.FONT


class OledDisplay:
    """High-level interface to the SSD1306 128x64 OLED display.

    Args:
        i2c: A busio.I2C instance connected to the display.
        address: I2C address of the SSD1306 (default 0x3C).
        rotation: Display rotation in degrees (default 180 to match
            original hardware mounting).
    """

    def __init__(self, i2c, address=0x3C, rotation=180):
        # Release any previously held displays (important for soft-reboot).
        # Done here rather than at module level to avoid side effects on import.
        displayio.release_displays()
        display_bus = displayio.I2CDisplay(i2c, device_address=address)
        self._display = SSD1306(display_bus, width=_WIDTH, height=_HEIGHT,
                                rotation=rotation)

        # The root display group — everything drawn goes here.
        self._root = displayio.Group()
        self._display.root_group = self._root

        # Labels for the workscreen (created lazily in show_workscreen).
        self._ch_label = None
        self._wheel_a_label = None
        self._wheel_b_label = None

    def clear(self):
        """Clear the display by removing all elements."""
        while len(self._root) > 0:
            self._root.pop()

    def show_workscreen(self, channel=1):
        """Display the main operating screen: MIDI channel and wheel value placeholders.

        Args:
            channel: The current MIDI transmit channel (1-16).
        """
        self.clear()

        # Large channel display at the top.
        ch_text = f"Ch {channel:2d}"
        self._ch_label = label.Label(_FONT, text=ch_text, scale=2,
                                     x=0, y=10, color=0xFFFFFF)
        self._root.append(self._ch_label)

        # Wheel A label and value.
        lbl_a = label.Label(_FONT, text="A", scale=1, x=0, y=30, color=0xFFFFFF)
        self._root.append(lbl_a)
        self._wheel_a_label = label.Label(_FONT, text="  00", scale=2,
                                          x=24, y=30, color=0xFFFFFF)
        self._root.append(self._wheel_a_label)

        # Wheel B label and value.
        lbl_b = label.Label(_FONT, text="B", scale=1, x=0, y=50, color=0xFFFFFF)
        self._root.append(lbl_b)
        self._wheel_b_label = label.Label(_FONT, text="  00", scale=2,
                                          x=24, y=50, color=0xFFFFFF)
        self._root.append(self._wheel_b_label)

    def update_values(self, wheel_a=None, wheel_b=None):
        """Update the wheel value displays on the workscreen.

        Only redraws labels that have changed. Call this every loop iteration.

        Args:
            wheel_a: Current wheel A value (int), or None to skip update.
            wheel_b: Current wheel B value (int), or None to skip update.
        """
        if wheel_a is not None and self._wheel_a_label is not None:
            self._wheel_a_label.text = f"{wheel_a:4X}" if isinstance(wheel_a, int) else str(wheel_a)
        if wheel_b is not None and self._wheel_b_label is not None:
            self._wheel_b_label.text = f"{wheel_b:4X}" if isinstance(wheel_b, int) else str(wheel_b)

    def update_channel(self, channel):
        """Update the MIDI channel display on the workscreen."""
        if self._ch_label is not None:
            self._ch_label.text = f"Ch {channel:2d}"

    def show_text(self, x, y, text, scale=1, inverted=False):
        """Draw a text label at the given pixel position.

        Used by menu.py and calibration.py for arbitrary text placement.

        Args:
            x: X pixel coordinate.
            y: Y pixel coordinate.
            text: The string to display.
            scale: Font scale multiplier (1, 2, or 3).
            inverted: If True, draw black text on white background.
        """
        color = 0x000000 if inverted else 0xFFFFFF
        bg_color = 0xFFFFFF if inverted else 0x000000
        lbl = label.Label(_FONT, text=text, scale=scale,
                          x=x, y=y, color=color, background_color=bg_color)
        self._root.append(lbl)
        return lbl

    def remove_label(self, lbl):
        """Remove a previously added label from the display.

        Args:
            lbl: A label object returned by show_text().
        """
        try:
            self._root.remove(lbl)
        except ValueError:
            pass  # Already removed.
```

- [ ] **Step 2: Verify on device**

In REPL:

```python
import busio
from hardware import pins
from hardware.display import OledDisplay
i2c = busio.I2C(pins.I2C_SCL, pins.I2C_SDA)
disp = OledDisplay(i2c)
disp.show_workscreen(channel=1)
import time; time.sleep(2)
disp.update_values(wheel_a=0x40, wheel_b=0x7F)
time.sleep(2)
disp.update_channel(10)
```

Verify: display shows "Ch  1", wheel values, then updates.

- [ ] **Step 3: Commit**

```bash
git add circuitpython/lib/hardware/display.py
git commit -m "feat: add OledDisplay with workscreen, text rendering, and 180° rotation"
```

---

### Task 7: UartMidi — DIN MIDI In/Out

**Files:**
- Create: `circuitpython/lib/hardware/midi_uart.py`

- [ ] **Step 1: Write `midi_uart.py`**

Create `circuitpython/lib/hardware/midi_uart.py`:

```python
# midi_uart.py — DIN MIDI input and output over UART.
#
# Wraps adafruit_midi over a busio.UART at the standard MIDI baud rate
# of 31250. Connects to a DIN MIDI circuit (optocoupler for input,
# resistor network for output) or an Adafruit MIDI FeatherWing.
#
# Both UartMidi and UsbHostMidi use adafruit_midi message types, so
# MidiMerge can treat them identically.
#
# Usage:
#     from hardware.midi_uart import UartMidi
#     from hardware import pins
#     midi = UartMidi(pins.UART_TX, pins.UART_RX)
#     msg = midi.receive()
#     if msg is not None:
#         print(f"Got: {msg}")
#     midi.send_cc(1, 64, channel=1)

import busio
import adafruit_midi
from adafruit_midi.control_change import ControlChange
from adafruit_midi.pitch_bend import PitchBend
from adafruit_midi.channel_pressure import ChannelPressure

# Standard MIDI baud rate.
_MIDI_BAUD = 31250


class UartMidi:
    """DIN MIDI input and output over UART.

    Provides receive() for reading incoming MIDI messages and send methods
    for outgoing messages. Uses adafruit_midi message types internally.

    Args:
        tx_pin: Board pin for UART TX (MIDI Out).
        rx_pin: Board pin for UART RX (MIDI In).
        channel: Default MIDI channel for sending (0-15, where 0 = channel 1).
            Can be overridden per-send call.
    """

    def __init__(self, tx_pin, rx_pin, channel=0):
        self._uart = busio.UART(tx_pin, rx_pin, baudrate=_MIDI_BAUD, timeout=0.001)
        # adafruit_midi.MIDI wraps a UART-like stream for message parsing.
        self._midi_in = adafruit_midi.MIDI(midi_in=self._uart, in_channel=None)
        self._midi_out = adafruit_midi.MIDI(midi_out=self._uart, out_channel=channel)

    def set_channel(self, channel):
        """Update the default output channel.

        Args:
            channel: MIDI channel (1-16).
        """
        self._midi_out.out_channel = channel - 1

    def receive(self):
        """Read one MIDI message from the DIN input, if available.

        Returns:
            An adafruit_midi message object, or None if no complete message
            is available yet.
        """
        return self._midi_in.receive()

    def send(self, msg):
        """Send a MIDI message object to the DIN output.

        Args:
            msg: An adafruit_midi message (NoteOn, ControlChange, PitchBend, etc.).
        """
        self._midi_out.send(msg)

    def send_cc(self, cc_number, value, channel=None):
        """Send a MIDI Control Change message.

        Args:
            cc_number: CC number (0-127).
            value: CC value (0-127).
            channel: MIDI channel (1-16). Uses default channel if None.
        """
        ch = (channel - 1) if channel is not None else None
        msg = ControlChange(cc_number, value)
        if ch is not None:
            msg.channel = ch
        self._midi_out.send(msg)

    def send_pitch_bend(self, value, channel=None):
        """Send a MIDI Pitch Bend message.

        Args:
            value: Pitch bend value (-8192 to 8191).
            channel: MIDI channel (1-16). Uses default channel if None.
        """
        ch = (channel - 1) if channel is not None else None
        msg = PitchBend(value)
        if ch is not None:
            msg.channel = ch
        self._midi_out.send(msg)

    def send_aftertouch(self, value, channel=None):
        """Send a MIDI Channel Pressure (Aftertouch) message.

        Args:
            value: Pressure value (0-127).
            channel: MIDI channel (1-16). Uses default channel if None.
        """
        ch = (channel - 1) if channel is not None else None
        msg = ChannelPressure(value)
        if ch is not None:
            msg.channel = ch
        self._midi_out.send(msg)
```

- [ ] **Step 2: Verify on device**

Connect DIN MIDI Out to a MIDI monitor (or loop back to a computer via MIDI interface). In REPL:

```python
from hardware.midi_uart import UartMidi
from hardware import pins
midi = UartMidi(pins.UART_TX, pins.UART_RX)
midi.send_cc(1, 64, channel=1)       # Mod wheel to center
midi.send_pitch_bend(0, channel=1)   # Pitch bend center
midi.send_aftertouch(100, channel=1) # Aftertouch
print("Sent 3 messages — check MIDI monitor")
```

Verify messages arrive on the MIDI monitor.

- [ ] **Step 3: Commit**

```bash
git add circuitpython/lib/hardware/midi_uart.py
git commit -m "feat: add UartMidi for DIN MIDI In/Out over UART"
```

---

### Task 8: UsbHostMidi — USB Host Input via MAX3421E

**Files:**
- Create: `circuitpython/lib/hardware/midi_usb_host.py`

- [ ] **Step 1: Write `midi_usb_host.py`**

Create `circuitpython/lib/hardware/midi_usb_host.py`:

```python
# midi_usb_host.py — USB Host MIDI input via MAX3421E breakout.
#
# Reads MIDI messages from a USB MIDI device (e.g., Novation Launchpad Pro 3)
# connected to the MAX3421E USB Host controller over SPI.
#
# This module handles:
# - SPI communication with the MAX3421E chip
# - USB device enumeration and MIDI interface detection
# - Hot-plug: graceful handling of connect/disconnect without crashing
# - Translating USB MIDI packets into adafruit_midi message objects
#   (same types as UartMidi, so MidiMerge can treat both identically)
#
# No device filtering is applied — any USB MIDI class-compliant device
# is accepted.
#
# Usage:
#     import busio
#     from hardware import pins
#     from hardware.midi_usb_host import UsbHostMidi
#     spi = busio.SPI(pins.SPI_CLK, MOSI=pins.SPI_MOSI, MISO=pins.SPI_MISO)
#     usb_midi = UsbHostMidi(spi, pins.USB_HOST_CS)
#     msg = usb_midi.receive()  # Returns adafruit_midi message or None

import digitalio
import max3421e
import usb.core
import adafruit_usb_host_midi
import adafruit_midi


class UsbHostMidi:
    """USB Host MIDI input via MAX3421E.

    Receives MIDI messages from a USB MIDI device connected to the
    MAX3421E breakout. Handles hot-plug gracefully.

    Args:
        spi: A busio.SPI instance for the MAX3421E.
        cs_pin: A board pin for the MAX3421E chip select.
    """

    def __init__(self, spi, cs_pin):
        self._spi = spi
        self._cs_pin = cs_pin
        self._host_chip = None
        self._midi_device = None
        self._midi_in = None
        self._init_host()

    def _init_host(self):
        """Initialize the MAX3421E USB Host controller.

        Called at startup. Sets up the chip select pin and creates
        the MAX3421E host object.
        """
        try:
            cs = digitalio.DigitalInOut(self._cs_pin)
            cs.direction = digitalio.Direction.OUTPUT
            cs.value = True
            self._host_chip = max3421e.MAX3421E(self._spi, cs)
            self._find_midi_device()
        except Exception:
            # MAX3421E not responding or not present — operate without USB input.
            self._host_chip = None
            self._midi_device = None
            self._midi_in = None

    def _find_midi_device(self):
        """Scan connected USB devices for a MIDI class-compliant interface.

        If found, wraps it in adafruit_midi.MIDI for message parsing.
        If not found, sets _midi_in to None (receive() will return None).
        """
        try:
            if self._host_chip is None:
                return
            # Find any connected USB MIDI device via usb.core.
            device = usb.core.find(find_all=False)
            if device is None:
                return
            raw_midi = adafruit_usb_host_midi.MIDI(device)
            self._midi_device = raw_midi
            self._midi_in = adafruit_midi.MIDI(midi_in=raw_midi, in_channel=None)
        except Exception:
            # No MIDI device found — that's fine, we'll try again on next call.
            self._midi_device = None
            self._midi_in = None

    def receive(self):
        """Read one MIDI message from the USB device, if available.

        Returns:
            An adafruit_midi message object, or None if no device is
            connected or no message is available.
        """
        # If no MIDI interface is active, try to find one (hot-plug).
        if self._midi_in is None:
            self._find_midi_device()
            if self._midi_in is None:
                return None

        try:
            return self._midi_in.receive()
        except Exception:
            # Device disconnected or communication error.
            # Reset state so we try to re-enumerate next time.
            self._midi_device = None
            self._midi_in = None
            return None
```

- [ ] **Step 2: Verify on device**

Connect the Launchpad Pro 3 (or any USB MIDI device) to the MAX3421E breakout. In REPL:

```python
import busio
from hardware import pins
from hardware.midi_usb_host import UsbHostMidi
spi = busio.SPI(pins.SPI_CLK, MOSI=pins.SPI_MOSI, MISO=pins.SPI_MISO)
usb = UsbHostMidi(spi, pins.USB_HOST_CS)
print("Waiting for MIDI from USB device... (press keys on Launchpad)")
while True:
    msg = usb.receive()
    if msg is not None:
        print(f"USB MIDI: {msg}")
```

Verify: pressing pads on the Launchpad prints NoteOn/NoteOff messages.

- [ ] **Step 3: Commit**

```bash
git add circuitpython/lib/hardware/midi_usb_host.py
git commit -m "feat: add UsbHostMidi for MAX3421E USB Host MIDI input"
```

---

### Task 9: MidiMerge — Merge Engine

**Files:**
- Create: `circuitpython/lib/midi_merge.py`

- [ ] **Step 1: Write `midi_merge.py`**

Create `circuitpython/lib/midi_merge.py`:

```python
# midi_merge.py — MIDI merge engine.
#
# Collects messages from multiple MIDI inputs (USB Host, DIN In) and
# locally injected events (wheels, buttons), then sends everything
# to the configured outputs (DIN Out).
#
# The merge engine does not filter, remap channels, or modify messages.
# USB Host input is always forwarded. DIN input is only forwarded when
# thru_enabled is True (matching the original Teensy behavior).
#
# Usage:
#     merge = MidiMerge(
#         inputs={"usb": usb_midi, "din": uart_midi},
#         outputs=[uart_midi],
#         thru_enabled=False
#     )
#     # In the main loop:
#     merge.process()
#     merge.inject(some_midi_message)
#     # At end of loop:
#     merge.flush()

class MidiMerge:
    """Merges multiple MIDI inputs and locally injected events to outputs.

    The processing order per loop iteration is:
    1. process() — reads all inputs, queues messages for output
    2. inject() — adds locally generated messages (wheels, buttons)
    3. flush() — sends all queued messages to all outputs

    Attributes:
        had_input: True if any external input was received during the
            last process() call. Used to drive the MIDI-in LED.
        thru_enabled: Whether DIN MIDI input is forwarded to outputs.

    Args:
        inputs: Dict mapping source names to input objects. Each input
            must have a receive() method returning a message or None.
            Expected keys: "usb" for USB Host, "din" for DIN MIDI In.
        outputs: List of output objects. Each must have a send(msg) method.
        thru_enabled: If True, DIN input ("din") is forwarded to outputs.
            USB Host input ("usb") is always forwarded regardless.
    """

    def __init__(self, inputs, outputs, thru_enabled=False):
        self.inputs = inputs
        self.outputs = outputs
        self.thru_enabled = thru_enabled
        self.had_input = False
        self._queue = []  # Messages to send this cycle.

    def process(self):
        """Read all MIDI inputs and queue messages for output.

        USB Host input is always forwarded. DIN input is only forwarded
        when thru_enabled is True.

        Call once at the start of each main loop iteration.
        """
        self._queue = []
        self.had_input = False

        for name, source in self.inputs.items():
            # Read all available messages from this source.
            msg = source.receive()
            while msg is not None:
                self.had_input = True
                # DIN input is conditional on thru_enabled.
                # USB input is always forwarded.
                if name != "din" or self.thru_enabled:
                    self._queue.append(msg)
                msg = source.receive()

    def inject(self, msg):
        """Add a locally generated MIDI message to the output queue.

        Used by the main loop to inject wheel and button events.

        Args:
            msg: An adafruit_midi message object.
        """
        self._queue.append(msg)

    def flush(self):
        """Send all queued messages to all outputs.

        Call once at the end of each main loop iteration, after all
        process() and inject() calls are done.
        """
        for msg in self._queue:
            for output in self.outputs:
                output.send(msg)
        self._queue = []

    @property
    def has_output(self):
        """True if there are messages queued for output this cycle.

        Useful for driving the MIDI-out LED.
        """
        return len(self._queue) > 0
```

- [ ] **Step 2: Verify on device**

Connect Launchpad via USB Host and DIN MIDI Out to a monitor. In REPL:

```python
import busio
from hardware import pins
from hardware.midi_uart import UartMidi
from hardware.midi_usb_host import UsbHostMidi
from midi_merge import MidiMerge
from adafruit_midi.control_change import ControlChange

spi = busio.SPI(pins.SPI_CLK, MOSI=pins.SPI_MOSI, MISO=pins.SPI_MISO)
uart = UartMidi(pins.UART_TX, pins.UART_RX)
usb = UsbHostMidi(spi, pins.USB_HOST_CS)

merge = MidiMerge(
    inputs={"usb": usb, "din": uart},
    outputs=[uart],
    thru_enabled=False
)

# Inject a local CC message and flush
merge.process()
merge.inject(ControlChange(1, 100))
merge.flush()
print("Sent CC1=100 via merge — check MIDI monitor")

# Now press keys on Launchpad and verify they appear on MIDI Out
print("Press Launchpad pads...")
for _ in range(100):
    merge.process()
    if merge.had_input:
        print("Got USB input, forwarding...")
    merge.flush()
    import time; time.sleep(0.05)
```

- [ ] **Step 3: Commit**

```bash
git add circuitpython/lib/midi_merge.py
git commit -m "feat: add MidiMerge engine with USB/DIN input and inject support"
```

---

### Task 10: WheelCalibration — Interactive Calibration Routine

**Files:**
- Create: `circuitpython/lib/calibration.py`

- [ ] **Step 1: Write `calibration.py`**

Create `circuitpython/lib/calibration.py`:

```python
# calibration.py — Interactive wheel calibration routine.
#
# Guides the user through calibrating the pitch bend and mod wheel:
# 1. Display instructions on the OLED
# 2. Wait for button press
# 3. Read current analog values as center/delta reference points
# 4. Save calibration values to ConfigManager
#
# This routine blocks the main loop (MIDI processing stops), matching
# the original Teensy behavior. It is only invoked from the on-device menu.
#
# Usage:
#     from calibration import WheelCalibration
#     cal = WheelCalibration(display, wheel_a, wheel_b, button_b, config)
#     cal.run()

import analogio
from hardware.wheels import _ADC_SHIFT


class WheelCalibration:
    """Interactive calibration for the analog wheels.

    Measures the resting position of each wheel and stores the values
    in the ConfigManager. The pitch bend wheel's center position and
    the mod wheel's zero position are determined.

    Args:
        display: An OledDisplay instance for showing instructions.
        wheel_a: The AnalogWheel for wheel A (pitch bend).
        wheel_b: The AnalogWheel for wheel B (mod wheel).
        button_b: The DebouncedButton used to confirm (Button B).
        config: The ConfigManager instance to save calibration values.
    """

    def __init__(self, display, wheel_a, wheel_b, button_b, config):
        self._display = display
        self._wheel_a = wheel_a
        self._wheel_b = wheel_b
        self._button = button_b
        self._config = config

    def run(self):
        """Run the interactive calibration sequence.

        Blocks until calibration is complete (user presses Button B twice).
        """
        # Step 1: Show instructions.
        self._display.clear()
        self._display.show_text(0, 6, "Calibrate Wheels", inverted=True)
        self._display.show_text(0, 20, "Release Pitch Wh")
        self._display.show_text(0, 30, "and return MODWh")
        self._display.show_text(0, 40, "to zero position")
        self._display.show_text(0, 50, "and press button")

        # Step 2: Wait for Button B press.
        self._wait_for_press()

        # Step 3: Read raw analog values (scaled to 10-bit for compatibility).
        raw_a = self._wheel_a._adc.value >> _ADC_SHIFT
        raw_b = self._wheel_b._adc.value >> _ADC_SHIFT

        # Wheel A (pitch bend): center is the current reading,
        # delta is center - 128 (the offset from ADC zero to pot zero).
        wheel_a_center = raw_a
        wheel_a_delta = raw_a - 128

        # Wheel B (mod wheel): delta is the current reading (zero position),
        # center is delta + 128 (used if wheel B is ever set to pitch bend mode).
        wheel_b_delta = raw_b
        wheel_b_center = raw_b + 128

        # Step 4: Save to config.
        self._config.set("wheel_a.calibration.center", wheel_a_center)
        self._config.set("wheel_a.calibration.delta", wheel_a_delta)
        self._config.set("wheel_b.calibration.center", wheel_b_center)
        self._config.set("wheel_b.calibration.delta", wheel_b_delta)

        # Update the wheel objects with the new calibration values.
        self._wheel_a.calibrate(center=wheel_a_center, delta=wheel_a_delta)
        self._wheel_b.calibrate(center=wheel_b_center, delta=wheel_b_delta)

        # Step 5: Show confirmation with the measured values.
        self._display.clear()
        self._display.show_text(0, 6, "Calibration Done", inverted=True)
        self._display.show_text(0, 24, f"WA ctr: {wheel_a_center:4d}")
        self._display.show_text(0, 36, f"WA dlt: {wheel_a_delta:4d}")
        self._display.show_text(0, 48, f"WB dlt: {wheel_b_delta:4d}")
        self._display.show_text(48, 56, "OK", scale=2)

        # Step 6: Wait for confirmation press, then return.
        self._wait_for_press()

    def _wait_for_press(self):
        """Block until Button B is pressed (fell edge detected)."""
        import time
        # First, wait for button to be released (in case it's still held).
        while True:
            self._button.update()
            if not self._button.pressed:
                break
            time.sleep(0.01)  # Yield CPU to avoid 100% spin.

        # Then wait for a new press.
        while True:
            self._button.update()
            if self._button.fell:
                return
            time.sleep(0.01)
```

- [ ] **Step 2: Verify on device**

This requires the display, wheels, and a button. Best tested as part of Task 11 (menu) or by running directly:

```python
import busio
from hardware import pins
from hardware.display import OledDisplay
from hardware.wheels import AnalogWheel
from hardware.buttons import DebouncedButton
from config import ConfigManager
from calibration import WheelCalibration

i2c = busio.I2C(pins.I2C_SCL, pins.I2C_SDA)
disp = OledDisplay(i2c)
wa = AnalogWheel(pins.WHEEL_A, mode="pitch_bend")
wb = AnalogWheel(pins.WHEEL_B, mode="cc")
btn = DebouncedButton(pins.BUTTON_B)
cfg = ConfigManager("config.json")
cfg.load()

cal = WheelCalibration(disp, wa, wb, btn, cfg)
cal.run()
cfg.save()
print(f"Saved: wheel_a center={cfg.get('wheel_a.calibration.center')}")
```

- [ ] **Step 3: Commit**

```bash
git add circuitpython/lib/calibration.py
git commit -m "feat: add WheelCalibration interactive routine"
```

---

### Task 11: OnDeviceMenu — Configuration Menu

**Files:**
- Create: `circuitpython/lib/menu.py`

- [ ] **Step 1: Write `menu.py`**

Create `circuitpython/lib/menu.py`:

```python
# menu.py — On-device configuration menu.
#
# Provides an interactive menu displayed on the OLED, navigated with
# buttons and the mod wheel. Allows the user to configure:
# - CC assignments for wheels and buttons (CC 0-127, or PWH/ATO special modes)
# - MIDI transmit channel (1-16)
# - MIDI Thru on/off
# - Aftertouch default value (0-127)
# - Deadband width (0-63)
# - Wheel calibration (launches calibration.py)
#
# The menu blocks the main loop — MIDI processing stops while active.
# This matches the original Teensy behavior and is intentional (menu
# interaction is brief and infrequent).
#
# Navigation:
# - Button A: step to next option
# - Button B: confirm/toggle value
# - Wheel B: scroll value
# - Select "X" (top right) to exit
#
# Triggered by double-clicking Button A in the main loop.
#
# Usage:
#     menu = OnDeviceMenu(display, config, wheel_b, button_a, button_b, calibration)
#     menu.enter()  # Blocks until user exits menu

from calibration import WheelCalibration


# Menu item definitions. Each tuple: (display_label, config_dotpath, item_type, value_range)
# item_type: "cc_or_special" (0-127 + PWH/ATO), "channel" (1-16),
#            "bool" (ON/OFF), "value" (numeric), "action" (triggers a function)
_MENU_ITEMS = [
    ("Wh A", "wheel_a", "wheel_mode", None),
    ("Wh B", "wheel_b", "wheel_mode", None),
    ("BtnA", "button_a", "button_mode", None),
    ("BtnB", "button_b", "button_mode", None),
    ("FtSw", "footswitch", "button_mode", None),
    ("TxCh", "midi.tx_channel", "channel", (1, 16)),
    ("Thru", "midi.thru_enabled", "bool", None),
    ("ATO ", "button_b.aftertouch_value", "value", (0, 127)),
    ("DB  ", "deadband", "value", (0, 63)),
    ("CAL ", None, "action", None),
]


class OnDeviceMenu:
    """On-device configuration menu displayed on the OLED.

    Args:
        display: An OledDisplay instance.
        config: A ConfigManager instance.
        wheel_b: The AnalogWheel for wheel B (used to scroll values).
        button_a: DebouncedButton for stepping through options.
        button_b: DebouncedButton for confirming/toggling.
        wheel_a: AnalogWheel for wheel A (needed by calibration).
    """

    def __init__(self, display, config, wheel_a, wheel_b, button_a, button_b):
        self._display = display
        self._config = config
        self._wheel_a = wheel_a
        self._wheel_b = wheel_b
        self._btn_a = button_a
        self._btn_b = button_b
        self._selected = 0  # Currently highlighted menu item index.

    def enter(self):
        """Enter the menu. Blocks until the user exits.

        On exit, saves the configuration to disk.
        """
        self._draw_menu()
        wheel_moved = False
        wheel_old = self._wheel_b.read()

        import time
        while True:
            self._btn_a.update()
            self._btn_b.update()

            # Check if wheel has moved significantly.
            wheel_val = self._wheel_b.read()
            if not wheel_moved:
                diff = abs(wheel_val - wheel_old)
                wheel_moved = diff > 5

            # Handle Button A: step to next item.
            if self._btn_a.fell:
                self._selected = (self._selected + 1) % (len(_MENU_ITEMS) + 1)
                wheel_moved = False
                wheel_old = wheel_val
                self._draw_menu()

            # Handle Button B: confirm/toggle current item.
            if self._btn_b.fell:
                # Exit item (index == len means the "X" close button).
                if self._selected == len(_MENU_ITEMS):
                    break

                item = _MENU_ITEMS[self._selected]
                self._handle_confirm(item)
                self._draw_menu()

            # Handle wheel scrolling for current item.
            if wheel_moved and self._selected < len(_MENU_ITEMS):
                item = _MENU_ITEMS[self._selected]
                self._handle_wheel(item, wheel_val)
                self._draw_menu()

            time.sleep(0.01)  # Yield CPU to avoid 100% spin.

        # Save all changes on menu exit.
        self._config.save()

    def _draw_menu(self):
        """Redraw the entire menu screen."""
        self._display.clear()

        # Header.
        self._display.show_text(0, 6, "LCC2 Config", inverted=True)

        # Menu items in two columns (5 items each) to fit all 10 items.
        # Column 1: items 0-4 on the left, Column 2: items 5-9 on the right.
        # Matches the original Teensy two-column layout.
        for i, item in enumerate(_MENU_ITEMS):
            label_text, dotpath, item_type, _ = item
            inverted = (i == self._selected)

            col = i // 5           # 0 = left column, 1 = right column
            row = i % 5            # 0-4 within each column
            x = col * 64           # Left column at x=0, right at x=64
            y = 18 + row * 10      # 5 rows starting at y=18

            val_text = self._format_value(item)
            line = f"{label_text}:{val_text}"
            self._display.show_text(x, y, line, inverted=inverted)

        # "X" close button at top right.
        self._display.show_text(116, 6, "X",
                                inverted=(self._selected == len(_MENU_ITEMS)))

    def _format_value(self, item):
        """Format a menu item's current value for display."""
        _, dotpath, item_type, _ = item

        if item_type == "action":
            return "GO"

        if item_type == "wheel_mode":
            mode = self._config.get(f"{dotpath}.mode")
            if mode == "pitch_bend":
                return "PWH"
            cc = self._config.get(f"{dotpath}.cc_number")
            return f"{cc:3d}" if cc is not None else "---"

        if item_type == "button_mode":
            mode = self._config.get(f"{dotpath}.mode")
            if mode == "aftertouch":
                return "ATO"
            cc = self._config.get(f"{dotpath}.cc_number")
            return f"{cc:3d}" if cc is not None else "---"

        if item_type == "channel":
            return f"{self._config.get(dotpath):3d}"

        if item_type == "bool":
            return " ON" if self._config.get(dotpath) else "OFF"

        if item_type == "value":
            return f"{self._config.get(dotpath):3d}"

        return "???"

    def _handle_confirm(self, item):
        """Handle Button B press on the currently selected item."""
        _, dotpath, item_type, val_range = item

        if item_type == "action":
            # Launch calibration.
            cal = WheelCalibration(self._display, self._wheel_a, self._wheel_b,
                                   self._btn_b, self._config)
            cal.run()
            return

        if item_type == "wheel_mode":
            # Toggle: cc → pitch_bend → cc, cycling CC number on each press.
            mode = self._config.get(f"{dotpath}.mode")
            if mode == "pitch_bend":
                self._config.set(f"{dotpath}.mode", "cc")
                if self._config.get(f"{dotpath}.cc_number") is None:
                    self._config.set(f"{dotpath}.cc_number", 1)
            else:
                cc = self._config.get(f"{dotpath}.cc_number")
                if cc is not None and cc < 127:
                    self._config.set(f"{dotpath}.cc_number", cc + 1)
                else:
                    self._config.set(f"{dotpath}.mode", "pitch_bend")
                    self._config.set(f"{dotpath}.cc_number", None)
            return

        if item_type == "button_mode":
            # Toggle: cc → aftertouch → cc, cycling CC number on each press.
            mode = self._config.get(f"{dotpath}.mode")
            if mode == "aftertouch":
                self._config.set(f"{dotpath}.mode", "cc")
                if self._config.get(f"{dotpath}.cc_number") is None:
                    self._config.set(f"{dotpath}.cc_number", 64)
            else:
                cc = self._config.get(f"{dotpath}.cc_number")
                if cc is not None and cc < 127:
                    self._config.set(f"{dotpath}.cc_number", cc + 1)
                else:
                    self._config.set(f"{dotpath}.mode", "aftertouch")
            return

        if item_type == "channel":
            val = self._config.get(dotpath)
            val = val + 1 if val < 16 else 1
            self._config.set(dotpath, val)
            return

        if item_type == "bool":
            self._config.set(dotpath, not self._config.get(dotpath))
            return

        if item_type == "value":
            lo, hi = val_range
            val = self._config.get(dotpath)
            val = val + 1 if val < hi else lo
            self._config.set(dotpath, val)
            return

    def _handle_wheel(self, item, wheel_val):
        """Handle wheel B scrolling for the currently selected item."""
        _, dotpath, item_type, val_range = item

        if item_type == "action":
            return  # No wheel control for calibration.

        if item_type == "wheel_mode":
            # Map wheel 0-127 to CC 0-127 or pitch_bend at max.
            if wheel_val >= 127:
                self._config.set(f"{dotpath}.mode", "pitch_bend")
                self._config.set(f"{dotpath}.cc_number", None)
            else:
                self._config.set(f"{dotpath}.mode", "cc")
                self._config.set(f"{dotpath}.cc_number", wheel_val)
            return

        if item_type == "button_mode":
            if wheel_val >= 127:
                self._config.set(f"{dotpath}.mode", "aftertouch")
            else:
                self._config.set(f"{dotpath}.mode", "cc")
                self._config.set(f"{dotpath}.cc_number", wheel_val)
            return

        if item_type == "channel":
            # Map wheel 0-127 to 1-16.
            self._config.set(dotpath, (wheel_val * 15 // 127) + 1)
            return

        if item_type == "bool":
            self._config.set(dotpath, wheel_val > 63)
            return

        if item_type == "value":
            lo, hi = val_range
            # Map wheel 0-127 to lo..hi range.
            mapped = lo + (wheel_val * (hi - lo) // 127)
            self._config.set(dotpath, mapped)
            return
```

- [ ] **Step 2: Verify on device**

Full menu test with all hardware connected:

```python
import busio
from hardware import pins
from hardware.display import OledDisplay
from hardware.wheels import AnalogWheel
from hardware.buttons import DebouncedButton
from config import ConfigManager
from menu import OnDeviceMenu

i2c = busio.I2C(pins.I2C_SCL, pins.I2C_SDA)
disp = OledDisplay(i2c)
wa = AnalogWheel(pins.WHEEL_A, mode="pitch_bend")
wb = AnalogWheel(pins.WHEEL_B, mode="cc")
btn_a = DebouncedButton(pins.BUTTON_A)
btn_b = DebouncedButton(pins.BUTTON_B)
cfg = ConfigManager("config.json")
cfg.load()

menu = OnDeviceMenu(disp, cfg, wa, wb, btn_a, btn_b)
print("Entering menu... Button A=next, Button B=set, Wheel B=scroll")
menu.enter()
print(f"TX Channel after menu: {cfg.get('midi.tx_channel')}")
```

Navigate through all options, change some values, exit. Verify `config.json` was updated.

- [ ] **Step 3: Commit**

```bash
git add circuitpython/lib/menu.py
git commit -m "feat: add OnDeviceMenu with all configuration options"
```

---

### Task 12: code.py — Main Entry Point

**Files:**
- Create: `circuitpython/code.py`

- [ ] **Step 1: Write `code.py`**

Create `circuitpython/code.py`:

```python
# code.py — Main entry point for the MIDI Companion Controller 2.
#
# This file contains ONLY wiring (hardware initialization) and the main
# loop. All business logic lives in the library modules under lib/.
#
# The main loop runs synchronously:
# 1. Update button states
# 2. Check for double-click → enter configuration menu
# 3. Process MIDI inputs (USB Host + DIN In) via MidiMerge
# 4. Read wheels, inject changed values into the merge
# 5. Read buttons, inject CC/Aftertouch events into the merge
# 6. Flush merged messages to DIN MIDI Out
# 7. Update the OLED display
# 8. Blink LEDs for activity indication

import busio

from hardware import pins
from hardware.leds import StatusLED
from hardware.buttons import DebouncedButton
from hardware.wheels import AnalogWheel
from hardware.display import OledDisplay
from hardware.midi_uart import UartMidi
from hardware.midi_usb_host import UsbHostMidi
from midi_merge import MidiMerge
from config import ConfigManager
from menu import OnDeviceMenu
from adafruit_midi.control_change import ControlChange
from adafruit_midi.pitch_bend import PitchBend
from adafruit_midi.channel_pressure import ChannelPressure

# ──────────────────────────────────────────────
# 1. CONFIGURATION
# ──────────────────────────────────────────────
config = ConfigManager("config.json")
config.load()

# ──────────────────────────────────────────────
# 2. HARDWARE INITIALIZATION
# ──────────────────────────────────────────────

# Communication buses.
i2c = busio.I2C(pins.I2C_SCL, pins.I2C_SDA)
spi = busio.SPI(pins.SPI_CLK, MOSI=pins.SPI_MOSI, MISO=pins.SPI_MISO)

# Display.
display = OledDisplay(i2c)

# LEDs.
led_in = StatusLED(pins.LED1)
led_out = StatusLED(pins.LED2)

# Buttons.
button_a = DebouncedButton(pins.BUTTON_A, double_click_ms=250)
button_b = DebouncedButton(pins.BUTTON_B, double_click_ms=0)  # No double-click needed.
footswitch = DebouncedButton(pins.FOOTSWITCH, double_click_ms=0)

# Wheels — mode and calibration loaded from config.
deadband = config.get("deadband")

wheel_a = AnalogWheel(pins.WHEEL_A,
                      mode=config.get("wheel_a.mode"),
                      deadband=deadband)
wheel_a.calibrate(center=config.get("wheel_a.calibration.center"),
                  delta=config.get("wheel_a.calibration.delta"))

wheel_b = AnalogWheel(pins.WHEEL_B,
                      mode=config.get("wheel_b.mode"),
                      deadband=deadband)
wheel_b.calibrate(center=config.get("wheel_b.calibration.center"),
                  delta=config.get("wheel_b.calibration.delta"))

# MIDI interfaces.
uart_midi = UartMidi(pins.UART_TX, pins.UART_RX,
                     channel=config.get("midi.tx_channel") - 1)
usb_midi = UsbHostMidi(spi, pins.USB_HOST_CS)

# MIDI merge engine.
merge = MidiMerge(
    inputs={"usb": usb_midi, "din": uart_midi},
    outputs=[uart_midi],
    thru_enabled=config.get("midi.thru_enabled")
)

# On-device menu.
menu = OnDeviceMenu(display, config, wheel_a, wheel_b, button_a, button_b)

# ──────────────────────────────────────────────
# 3. SHOW STARTUP SCREEN
# ──────────────────────────────────────────────
display.show_workscreen(channel=config.get("midi.tx_channel"))

# ──────────────────────────────────────────────
# 4. MAIN LOOP
# ──────────────────────────────────────────────
while True:

    # --- Update button states ---
    button_a.update()
    button_b.update()
    footswitch.update()

    # --- Double-click Button A → enter menu ---
    if button_a.double_click:
        menu.enter()
        # After menu exit, reload settings that may have changed.
        tx_ch = config.get("midi.tx_channel")
        merge.thru_enabled = config.get("midi.thru_enabled")
        wheel_a.mode = config.get("wheel_a.mode")
        wheel_a.deadband = config.get("deadband")
        wheel_b.mode = config.get("wheel_b.mode")
        wheel_b.deadband = config.get("deadband")
        uart_midi.set_channel(tx_ch)
        display.show_workscreen(channel=tx_ch)
        continue

    # --- Process MIDI inputs (USB Host + DIN) ---
    merge.process()
    if merge.had_input:
        led_in.on()

    # --- Read wheels and inject changes ---
    tx_ch = config.get("midi.tx_channel")
    output_generated = False

    # Wheel A.
    wheel_a.read()
    if wheel_a.changed:
        if config.get("wheel_a.mode") == "pitch_bend":
            msg = PitchBend(wheel_a.value)
            msg.channel = tx_ch - 1
            merge.inject(msg)
        else:
            cc_num = config.get("wheel_a.cc_number")
            msg = ControlChange(cc_num, wheel_a.value)
            msg.channel = tx_ch - 1
            merge.inject(msg)
        output_generated = True

    # Wheel B.
    wheel_b.read()
    if wheel_b.changed:
        if config.get("wheel_b.mode") == "pitch_bend":
            msg = PitchBend(wheel_b.value)
            msg.channel = tx_ch - 1
            merge.inject(msg)
        else:
            cc_num = config.get("wheel_b.cc_number")
            msg = ControlChange(cc_num, wheel_b.value)
            msg.channel = tx_ch - 1
            merge.inject(msg)
        output_generated = True

    # --- Read buttons and inject events ---
    ato_value = config.get("button_b.aftertouch_value")

    # Button A.
    if button_a.fell or button_a.rose:
        if config.get("button_a.mode") == "aftertouch":
            val = ato_value if button_a.fell else 0
            msg = ChannelPressure(val)
        else:
            cc_num = config.get("button_a.cc_number")
            val = 127 if button_a.fell else 0
            msg = ControlChange(cc_num, val)
        msg.channel = tx_ch - 1
        merge.inject(msg)
        output_generated = True

    # Button B.
    if button_b.fell or button_b.rose:
        if config.get("button_b.mode") == "aftertouch":
            val = ato_value if button_b.fell else 0
            msg = ChannelPressure(val)
        else:
            cc_num = config.get("button_b.cc_number")
            val = 127 if button_b.fell else 0
            msg = ControlChange(cc_num, val)
        msg.channel = tx_ch - 1
        merge.inject(msg)
        output_generated = True

    # Footswitch.
    if footswitch.fell or footswitch.rose:
        if config.get("footswitch.mode") == "aftertouch":
            val = ato_value if footswitch.fell else 0
            msg = ChannelPressure(val)
        else:
            cc_num = config.get("footswitch.cc_number")
            val = 127 if footswitch.fell else 0
            msg = ControlChange(cc_num, val)
        msg.channel = tx_ch - 1
        merge.inject(msg)
        output_generated = True

    # --- Flush all queued messages to DIN MIDI Out ---
    if output_generated:
        led_out.on()
    merge.flush()

    # --- Update display (only on value changes) ---
    if wheel_a.changed or wheel_b.changed:
        display.update_values(
            wheel_a=wheel_a.value if wheel_a.changed else None,
            wheel_b=wheel_b.value if wheel_b.changed else None
        )

    # --- LEDs off at end of cycle ---
    led_in.off()
    led_out.off()
```

- [ ] **Step 2: Verify on device — full integration test**

Copy the entire `circuitpython/` directory to the CIRCUITPY drive (including all `lib/` modules and `config.json`). Also install required Adafruit libraries from the CircuitPython Bundle into `lib/`:
- `adafruit_midi/`
- `adafruit_usb_host_midi.mpy`
- `adafruit_max3421e.mpy`
- `adafruit_debouncer.mpy`
- `adafruit_displayio_ssd1306.mpy`
- `adafruit_display_text/`
- `adafruit_ticks.mpy` (dependency of adafruit_debouncer)

The device should auto-run `code.py` on boot. Verify:
1. OLED shows workscreen with "Ch  1"
2. Moving wheels updates hex values on display
3. DIN MIDI Out sends pitch bend / mod wheel CC
4. Pressing buttons sends CC / aftertouch via DIN MIDI Out
5. Connecting Launchpad via USB Host → notes forwarded to DIN Out
6. Double-click Button A → menu appears
7. Change TX channel in menu, exit → workscreen shows new channel
8. LED1 blinks on MIDI input, LED2 blinks on local MIDI output

- [ ] **Step 3: Commit**

```bash
git add circuitpython/code.py
git commit -m "feat: add code.py main loop — complete CircuitPython port"
```

---

### Task 13: Final cleanup and documentation

**Files:**
- Modify: `CLAUDE.md` (add CircuitPython section)

- [ ] **Step 1: Update CLAUDE.md**

Add a CircuitPython section to `CLAUDE.md` documenting:
- How to deploy to the Pico 2 (copy files to CIRCUITPY)
- Required Adafruit libraries
- Dev-mode boot (hold Button A during boot for USB filesystem access)
- Config file location and format

- [ ] **Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: add CircuitPython deployment instructions to CLAUDE.md"
```
