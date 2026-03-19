# MIDI Companion Controller 2 — CircuitPython Port for Raspberry Pi Pico 2

## Overview

Port of the Teensy 3.2-based MIDI companion controller to Raspberry Pi Pico 2 (RP2350) running CircuitPython. The controller merges MIDI input from a Novation Launchpad Pro 3 (via USB Host) and local controls (wheels, buttons, footswitch) with optional DIN MIDI Thru, outputting a merged MIDI stream over DIN MIDI to a Roland SE-02 synthesizer.

### Key Changes from Original

- Teensy 3.2 → Raspberry Pi Pico 2 (RP2350)
- C++ (Arduino/Teensyduino) → CircuitPython 9.x
- Single 790-line file → modular library with HAL + feature modules
- EEPROM with checksum → JSON config file on CIRCUITPY flash
- Custom OLED library → standard adafruit_displayio_ssd1306
- USB-MIDI device output → removed (DIN-only output). `boot.py` disables USB MIDI device and USB HID to free the USB stack. The Pico does not appear as a MIDI device to a connected computer.
- New: USB Host input via MAX3421E SPI breakout for Launchpad Pro 3
- Deliberate behavior change: channel auto-tracking from DIN input is removed. The controller always sends on the configured `tx_channel`. (The original Teensy code would adopt the channel of any incoming MIDI message.)

## Signal Flow

```
┌─────────────────────┐     ┌──────────────────┐
│  Launchpad Pro 3    │     │  DIN MIDI In      │
│  (USB Host/MAX3421E)│     │  (UART RX)        │
└────────┬────────────┘     └────────┬───────────┘
         │ all MIDI data              │ any MIDI
         │ (no filtering)            │ data
         ▼                           ▼
    ┌─────────────────────────────────────┐
    │            MidiMerge                │
    │  ┌───────────────────────────┐      │
    │  │ inject() ← local events  │      │
    │  │  · Pitch Bend (Wheel A)  │      │
    │  │  · Mod Wheel (Wheel B)   │      │
    │  │  · CC/ATO (Buttons)      │      │
    │  └───────────────────────────┘      │
    └────────────────┬────────────────────┘
                     │ merged
                     │ MIDI stream
                     ▼
            ┌─────────────────┐
            │  DIN MIDI Out   │
            │  (UART TX)      │
            │  → Roland SE-02 │
            └─────────────────┘
```

## Project Structure

```
circuitpython/
  boot.py                       # Runs before code.py: remounts filesystem writable, disables USB MIDI/HID
  code.py                       # Entry point: setup + main loop (wiring only, no logic)
  config.json                   # Persisted configuration on CIRCUITPY flash
  lib/
    hardware/
      pins.py                   # Pin definitions as central constants (only place GPIOs are defined)
      wheels.py                 # AnalogWheel: read, calibration, deadband
      buttons.py                # DebouncedButton: debounce, fell/rose, double-click detection
      leds.py                   # StatusLED: on/off
      display.py                # OledDisplay: SSD1306 via displayio
      midi_uart.py              # UartMidi: DIN MIDI In + Out over UART
      midi_usb_host.py          # UsbHostMidi: MAX3421E via adafruit_usb_host_midi
    midi_merge.py               # MidiMerge: N inputs merged onto M outputs
    config.py                   # ConfigManager: JSON read/write, defaults, validation
    menu.py                     # OnDeviceMenu: menu structure, navigation, value editing
    calibration.py              # WheelCalibration: interactive calibration, saves to config
```

The existing Teensy version remains unchanged in `src/`.

## Hardware Modules

### pins.py

Pure constants file. All GPIO assignments in one place. All other modules import from here. Also defines SPI pins for the MAX3421E.

### wheels.py — AnalogWheel

```python
wheel = AnalogWheel(pin, mode="pitch_bend", deadband=10)
wheel.calibrate(center=658, delta=517)
value = wheel.read()  # → int: -8192..8191 (pitch_bend) or 0..127 (cc)
wheel.changed         # → bool: True if value changed since last read
wheel.value           # → int: last read value
```

- Wraps `analogio.AnalogIn` with calibration offset and deadband logic.
- Mode (`pitch_bend` vs `cc`) determines centered (signed) or uncentered (unsigned) reading.
- **Pitch bend mode** returns 14-bit values (-8192..8191) as expected by `adafruit_midi.PitchBend`. The original code reads -128..127 and multiplies by 64; this module does the scaling internally.
- **CC mode** returns 7-bit values (0..127).
- Calibration uses `center` for pitch bend mode (centered wheel) and `delta` for CC mode (uncentered wheel), matching the original behavior (see original lines 725-726).
- `deadband` is a global setting shared by both wheels (loaded from `config.deadband`).
- Same algorithm as original `readWheel()`, encapsulated as a class.

### buttons.py — DebouncedButton

```python
btn = DebouncedButton(pin, debounce_ms=10, double_click_ms=250)
btn.update()
btn.fell              # → bool: just pressed
btn.rose              # → bool: just released
btn.double_click      # → bool: double-click detected
```

- Uses `adafruit_debouncer` instead of Bounce2.
- Double-click detection built in (timestamp-based, like original).
- `fell` = button pressed (active low with INPUT_PULLUP), `rose` = button released. All buttons use the same polarity convention. The main loop determines what MIDI value to send on fell vs. rose (e.g., CC 127 on fell, CC 0 on rose).

### leds.py — StatusLED

```python
led = StatusLED(pin)
led.on()
led.off()
```

- Minimal wrapper around `digitalio.DigitalInOut`.
- LED1 = MIDI input activity, LED2 = MIDI output activity.

### display.py — OledDisplay

```python
disp = OledDisplay(i2c)
disp.show_workscreen(channel=1, wheel_a=0, wheel_b=0)
disp.show_text(row, col, text, inverted=False)
disp.show_hex(row, col, value)
disp.clear()
disp.update_values(wheel_a=0, wheel_b=0)
```

- Uses `displayio` + `adafruit_displayio_ssd1306` + `adafruit_display_text`.
- Abstracts workscreen, menu, and calibration screen rendering.
- Font size handling internal (replaces global `fontsize` variable from original).
- Display is rotated 180 degrees (matching original `oled.rotateDisplay180()`) — configured in `displayio` initialization.

### midi_uart.py — UartMidi

```python
midi = UartMidi(uart_tx_pin, uart_rx_pin)
msg = midi.receive()                        # → adafruit_midi message or None
midi.send(msg)                              # Send any MIDI message
midi.send_cc(cc, value, channel)
midi.send_pitch_bend(value, channel)
midi.send_aftertouch(value, channel)
```

- Uses `adafruit_midi` over `busio.UART` at 31250 baud.
- Shared message format with `UsbHostMidi` (both use `adafruit_midi` message types).

### midi_usb_host.py — UsbHostMidi

```python
usb_midi = UsbHostMidi(spi, cs_pin)
msg = usb_midi.receive()  # → adafruit_midi message or None
```

- MAX3421E over SPI via `adafruit_max3421e` + `adafruit_usb_host_midi`.
- Same message interface as `UartMidi`.
- Receive only (Launchpad sends to controller).
- **Hot-plug handling:** `receive()` returns `None` if no device is connected. The module handles enumeration failures and disconnection gracefully — no crash, no error on display, just no input from that source. On reconnection, the device is re-enumerated automatically.
- No device filtering: any USB MIDI device is accepted, not just the Launchpad.

## MidiMerge

```python
merge = MidiMerge(
    inputs=[uart_midi_in, usb_host_midi],
    outputs=[uart_midi_out],
    thru_enabled=False
)
merge.process()           # Read all inputs, forward to outputs
merge.inject(msg)         # Inject local messages (wheels/buttons)
merge.had_input           # → bool: True if any input was received this cycle
```

### Merge Rules

- **USB Host input** (Launchpad): always forwarded to outputs unmodified. No filtering, no channel remapping.
- **DIN MIDI input:** only forwarded when `thru_enabled` is True. When `thru_enabled` is False, DIN input is ignored (not merged). This matches the original behavior where MIDI Thru is a configurable on/off toggle.
- Local events (wheels, buttons) are injected via `inject()` and always sent on `tx_channel`.
- `process()` called once per main loop iteration.
- Order per loop: read inputs → inject local events → send all.

## Configuration System

### config.json

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

### ConfigManager

```python
config = ConfigManager("config.json")
config.load()                          # Read file, fall back to built-in defaults
config.save()                          # Write current state to file
config.get("midi.tx_channel")          # → value
config.set("midi.tx_channel", 2)       # Update in memory
```

- Falls back to built-in defaults when file is missing or corrupt.
- Validates value ranges on load (channel 1-16, CC 0-127, etc.).
- Replaces EEPROM+checksum system entirely.
- **Filesystem write access:** `boot.py` remounts the CIRCUITPY filesystem as writable by CircuitPython code (`storage.remount("/", readonly=False)`). Side effect: the drive becomes read-only when connected to a computer via USB. To edit `config.json` or `code.py` from a PC, hold a designated button during boot to skip the remount (keeping the filesystem writable by USB instead).

### Interaction with On-Device Menu

1. On startup: `config.load()` reads the file.
2. Menu changes: `config.set()` updates values in memory.
3. On menu exit: `config.save()` writes everything back to JSON.

## Menu System

### OnDeviceMenu

```python
menu = OnDeviceMenu(display, config, wheels, buttons)
menu.enter()    # Blocks until menu is exited
```

**Trigger:** Double-click on Button A.

**Note:** Menu and calibration are blocking — MIDI processing stops while the menu is active. This matches the original behavior and is a deliberate choice (menu interaction is brief and infrequent).

**Navigation** (same as original):
- Button A: next option
- Button B: set/confirm value
- Wheel B: scroll value
- "X" top right: exit menu

**Menu Options:**

| Option | Range | Description |
|--------|-------|-------------|
| Wheel A | CC 0-127 / PWH | CC number or pitch bend |
| Wheel B | CC 0-127 / PWH | CC number or pitch bend |
| Button A | CC 0-127 / ATO | CC number or aftertouch |
| Button B | CC 0-127 / ATO | CC number or aftertouch |
| Footswitch | CC 0-127 / ATO | CC number or aftertouch |
| TX Channel | 1-16 | MIDI transmit channel |
| Thru | ON/OFF | DIN MIDI Thru |
| ATO Value | 0-127 | Aftertouch default value |
| Deadband | 0-63 | Deadband width |
| Calibrate | GO | Starts calibration routine |

## Calibration

### WheelCalibration

```python
cal = WheelCalibration(display, wheels, buttons, config)
cal.run()       # Blocks: shows instructions, waits for button, measures, saves
```

**Sequence** (same as original):
1. Display shows "Release Pitch Wheel / Return Mod Wheel to zero / Press button"
2. User presses Button B
3. Read current analog values as center/delta
4. Write values to ConfigManager
5. Show confirmation, wait for button press
6. Return to menu

## Main Loop (code.py)

`code.py` contains only wiring and the main loop — no business logic. All domain logic lives in the modules.

```python
# 1. SETUP
config = ConfigManager("config.json")
config.load()

# Initialize all hardware via pin constants
# Initialize MidiMerge with uart_midi + usb_host_midi inputs, uart_midi output
# Initialize OnDeviceMenu
# Show workscreen

# 2. MAIN LOOP
while True:
    # Update all buttons
    # Check for double-click → enter menu
    # merge.process() — read all MIDI inputs, forward
    # LED1 blink on MIDI input
    # Read wheels, inject changed values via merge.inject()
    # Read buttons, inject fell/rose events via merge.inject()
    # LED2 blink on MIDI output
    # Update display (only on value changes)
    # Turn off LEDs at end of loop
```

The loop is synchronous — CircuitPython on RP2350 is fast enough for real-time MIDI.

## Dependencies

### Adafruit CircuitPython Libraries (from Bundle)

| Library | Purpose |
|---------|---------|
| `adafruit_midi` | MIDI message types + UART MIDI |
| `adafruit_usb_host_midi` | USB Host MIDI via MAX3421E |
| `adafruit_max3421e` | MAX3421E SPI driver |
| `adafruit_debouncer` | Button debouncing |
| `adafruit_displayio_ssd1306` | SSD1306 OLED driver |
| `adafruit_display_text` | Text rendering on displayio |
| `adafruit_bitmap_font` | Font sizes (optional) |

### Built-in CircuitPython Modules

| Module | Purpose |
|--------|---------|
| `board`, `digitalio`, `analogio` | GPIO |
| `busio` | I2C, SPI, UART |
| `displayio` | Display framework |
| `json` | Config file parsing |
| `time` | Timing (double-click, LED blink) |
| `usb_host` | USB Host base functionality |

### Target Platform

- **Board:** Raspberry Pi Pico 2 or Pico 2 W (RP2350)
- **CircuitPython:** 9.x (stable for RP2350, USB Host support)
- **USB Host Hardware:** Adafruit MAX3421E USB Host Breakout (SPI)

### Additional Hardware

- SSD1306 128x64 I2C OLED (same as original)
- MIDI DIN In/Out circuit — either:
  - Custom optocoupler circuit (same as original), OR
  - Adafruit MIDI FeatherWing (plug-and-play alternative, provides DIN In + Out via UART)
- Potentiometers/wheels, buttons, footswitch, LEDs (same as original)
