# MIDI Companion Controller V3.0

A MIDI companion controller with two analog wheels (pitch bend + mod wheel), two buttons, a footswitch, two LEDs, and a 128x64 OLED display. Runs on a Raspberry Pi Pico 2 (RP2350) with CircuitPython 10.x.

## Features

- **Two analog wheels**: Pitch bend (centered) and modulation (CC1)
- **Two buttons + footswitch**: Configurable as CC, NoteOn/Off, Aftertouch, or Off
- **OLED display**: Shows MIDI channel, wheel values, firmware version
- **DIN MIDI In/Out**: Standard 5-pin DIN via UART + optocoupler circuit
- **USB MIDI**: Appears as a USB MIDI device to a connected computer/DAW
- **USB Host** (optional): MAX3421E breakout for connecting USB MIDI devices (e.g., Launchpad)
- **MIDI merge engine**: Routes messages between all inputs and outputs with echo prevention
- **On-device menu**: Configure CC assignments, MIDI channel, thru, deadband, calibration
- **Watchdog**: Auto-resets if the main loop hangs (enabled after first successful iteration)

## Hardware

- **Board**: Raspberry Pi Pico 2 W (RP2350, CircuitPython 10.1.4)
- **Display**: SSD1306 128x64 I2C OLED at 0x3C
- **MIDI DIN**: Optocoupler circuit or Adafruit MIDI FeatherWing
- **USB Host**: Adafruit MAX3421E breakout (optional)

### Pin Assignments

| GPIO | Function | Notes |
|------|----------|-------|
| GP0 | Button A | Menu / navigation, INPUT_PULLUP |
| GP1 | Button B | Confirm / set, INPUT_PULLUP |
| GP2 | LED 1 | MIDI input activity |
| GP3 | LED 2 | MIDI output activity |
| GP4 | UART TX | DIN MIDI Out, 31250 baud |
| GP5 | UART RX | DIN MIDI In, 31250 baud |
| GP13 | Footswitch | 1/4" jack, INPUT_PULLUP |
| GP14 | USB Host INT | MAX3421E (optional) |
| GP16 | SPI MISO | MAX3421E (optional) |
| GP18 | SPI CLK | MAX3421E (optional) |
| GP19 | SPI MOSI | MAX3421E (optional) |
| GP20 | I2C SDA | OLED |
| GP21 | I2C SCL | OLED |
| GP22 | USB Host CS | MAX3421E (optional) |
| GP26/A0 | Wheel A | Pitch bend (analog) |
| GP27/A1 | Wheel B | Modulation (analog) |

See [HARDWARE_PICO.md](HARDWARE_PICO.md) for detailed documentation.

## Installation

1. Install CircuitPython 10.x on the Pico 2
2. Copy `circuitpython/` contents to the CIRCUITPY drive
3. Install Adafruit libraries:
   ```bash
   pip install circup
   circup install adafruit_midi adafruit_debouncer adafruit_ticks adafruit_displayio_ssd1306 adafruit_display_text
   ```
4. For USB Host mode, also: `circup install adafruit_usb_host_midi max3421e`

## Developer Mode

Hold **Button A** (GP0) during power-on to enter dev mode. This keeps the CIRCUITPY drive writable from USB for editing files. In normal mode, CircuitPython owns the filesystem for writing config.json.

## Testing

Host-side integration tests communicate with the Pico over USB MIDI and serial:

```bash
pip install -r tests/requirements.txt
# Automated tests (device connected, normal mode):
MIDI_DEVICE_NAME=CircuitPython pytest tests/ -v --ignore=tests/test_wheels_buttons.py
# Interactive hardware test:
MIDI_DEVICE_NAME=CircuitPython python3 tests/test_hardware.py
```

See [tests/README.md](tests/README.md) for details.

## Known Issues and Status (2026-03-29)

### Verified Working
- Button A (GP0) and Button B (GP1) — confirmed via GPIO probe
- LED 1 (GP2) and LED 2 (GP3) — confirmed via LED test
- OLED display — working with CP10 `i2cdisplaybus.I2CDisplayBus`, rotation=0
- USB MIDI device — enumeration, send, receive all working
- DIN UART loopback — raw bytes sent on GP4/TX received on GP5/RX through DIN connectors (optocoupler circuit functional)
- Config persistence — load, save, validation, corrupt detection all working
- Wheel B (mod wheel) — sends CC1, values 0-127, display shows decimal
- Wheel A (pitch bend) — sends PitchBend, calibrated, display shows signed values

### Known Issues

| Issue | Status | Details |
|-------|--------|---------|
| **DIN MIDI not reaching external synth** | **Open** | UART loopback through DIN connectors works (raw bytes verified). USB→DIN routing via merge engine sends data, but external synthesizer does not respond. Possible causes: cable pin assignment (DIN-5 pins 4/5), signal level/polarity through optocoupler, or synth-specific issue. Needs oscilloscope verification. |
| **Wheel B ADC jitter** | **Open** | Mod wheel oscillates ±2 values at rest (e.g., 44-46), causing continuous CC output and LED B staying on. Root cause: ADC noise on the potentiometer. Mitigations: increase deadband, add 100nF capacitor across pot, or implement hysteresis filter in software. |
| **Watchdog crash-loop** | **Fixed** | Watchdog now only activates after the first fully successful loop iteration. If code.py crashes on startup, the device stays at the REPL instead of resetting endlessly. |
| **CircuitPython 10 API changes** | **Fixed** | `displayio.I2CDisplay` → `i2cdisplaybus.I2CDisplayBus`, `watchdog.WatchdogMode` → `watchdog.WatchDogMode`, `PitchBend` expects unsigned 0-16383. |

### Hardware Test Results

```
Buttons:  GP0 (A) ✓  GP1 (B) ✓
LEDs:     GP2 ✓  GP3 ✓
UART:     GP4 TX → DIN OUT → DIN IN → GP5 RX: loopback ✓
Display:  SSD1306 I2C 0x3C: ✓ (rotation=0)
USB MIDI: enumeration ✓, send ✓, receive ✓
DIN→Synth: ✗ (needs investigation)
```

## Project Structure

```
circuitpython/
  boot.py              # Filesystem + USB setup
  code.py              # Main loop (v2.0.0)
  config.json          # Persistent config (JSON)
  lib/
    config.py          # ConfigManager with validation
    menu.py            # On-device config menu
    calibration.py     # Wheel calibration (30s timeout)
    midi_merge.py      # N→M merge with echo prevention
    hardware/
      pins.py          # Pin definitions (single source of truth)
      wheels.py        # Analog wheel with deadband
      buttons.py       # Debounced buttons with double-click
      leds.py          # LEDs with 50ms min on-time
      display.py       # SSD1306 OLED via displayio
      midi_uart.py     # DIN MIDI via UART
      midi_usb_device.py  # Native USB MIDI
      midi_usb_host.py    # MAX3421E USB Host (optional)
tests/
  conftest.py          # MIDI + serial REPL fixtures
  test_hardware.py     # Interactive hardware diagnostic
  test_midi_device.py  # USB MIDI enumeration tests
  test_config.py       # Config persistence tests
  test_error_recovery.py  # Corrupt config, MIDI flood
  test_wheels_buttons.py  # Manual input tests
```

## History

- **V1.0**: Arduino + DangerShield (original prototype)
- **V2.0**: Teensy 3.2 + custom hardware (`src/main.cpp` — legacy, see [BUGLIST_TEENSY.md](BUGLIST_TEENSY.md))
- **V3.0**: Raspberry Pi Pico 2 + CircuitPython 10 (this version)

## License

CC-BY — untergeek.de
