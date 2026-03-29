# Hardware Documentation — Raspberry Pi Pico 2

Implementation: `circuitpython/`

## Board
- **MCU**: Raspberry Pi Pico 2 (RP2350, dual ARM Cortex-M33, 150 MHz)
- **USB**: Native USB MIDI device (via CircuitPython `usb_midi`)
- **ADC**: 16-bit (0-65535), right-shifted 6 bits to 10-bit scale in software
- **CircuitPython**: 9.x required

## Operating Modes

| Mode | Hardware Required | Signal Flow |
|------|-------------------|-------------|
| **Standalone** | Pico 2 + MAX3421E + DIN circuit | Launchpad + DIN In + USB Device <-> MidiMerge <-> DIN Out + USB Device + USB Host |
| **Laptop-only** | Pico 2 + DIN circuit | USB Device + DIN In <-> MidiMerge <-> DIN Out + USB Device |

The MAX3421E is detected at startup via `ImportError` catch on `from hardware.midi_usb_host import UsbHostMidi`. If the libraries aren't installed or the chip isn't present, it is skipped silently.

## Pin Assignments

Source of truth: `circuitpython/lib/hardware/pins.py` (last updated 2026-03-28)

| GPIO | Pin # | Type | Function | Notes |
|------|-------|------|----------|-------|
| GP0 | 1 | Digital | Button A | Menu / navigation, INPUT_PULLUP |
| GP1 | 2 | Digital | Button B | Confirm / set, INPUT_PULLUP |
| — | 3 | — | GND | — |
| GP2 | 4 | Digital | LED 1 | MIDI input activity |
| GP3 | 5 | Digital | LED 2 | MIDI output activity |
| GP4 | 6 | UART TX | MIDI Out (DIN-5) | 31,250 baud |
| GP5 | 7 | UART RX | MIDI In (DIN-5) | 31,250 baud |
| GP13 | 17 | Digital | Footswitch | 1/4" jack, INPUT_PULLUP |
| — | 18 | — | GND | — |
| GP14 | 19 | Digital | USB Host INT | MAX3421E interrupt (optional) |
| GP16 | 21 | SPI MISO | USB Host Data In | MAX3421E (optional) |
| GP18 | 24 | SPI CLK | USB Host Clock | MAX3421E (optional) |
| GP19 | 25 | SPI MOSI | USB Host Data Out | MAX3421E (optional) |
| GP20 | 26 | I2C SDA | OLED Data | SSD1306 |
| GP21 | 27 | I2C SCL | OLED Clock | SSD1306 |
| — | 28 | — | GND | — |
| GP22 | 29 | Digital | USB Host CS | MAX3421E chip select (optional) |
| GP26/A0 | 31 | Analog | Wheel A (Pitch Bend) | Left wheel |
| GP27/A1 | 32 | Analog | Wheel B (Modulation) | Right wheel |
| — | 33 | — | AGND | Analog ground |

## Components
- **Display**: SSD1306 128x64 I2C OLED at address 0x3C (via `displayio` + `adafruit_displayio_ssd1306`)
- **MIDI DIN**: In/Out via UART (GP4/GP5) at 31,250 baud, optocoupler circuit or Adafruit MIDI FeatherWing
- **USB Host**: Adafruit MAX3421E breakout (SPI) — optional, for standalone mode
- **Buttons**: Momentary pushbuttons, active low (INPUT_PULLUP), debounced via `adafruit_debouncer` (10ms interval)
- **Footswitch**: 1/4" TRS jack, active low (INPUT_PULLUP), debounced
- **LEDs**: Standard LEDs with current-limiting resistors, HIGH = on, LOW = off
- **Wheels**: Linear potentiometers, ~256 usable ADC count range (same physical pots as Teensy version)

## Bus Details

### SPI Bus (MAX3421E USB Host — optional)
- Clock: GP18
- MOSI: GP19
- MISO: GP16
- CS: GP22
- INT: GP14

### I2C Bus (OLED Display)
- SDA: GP20
- SCL: GP21
- Address: 0x3C
- Driver: `displayio.I2CDisplay`

### UART (DIN MIDI)
- TX: GP4
- RX: GP5
- Baud: 31,250
- Timeout: 0.001s (in `busio.UART`)

## Configuration Storage

JSON file (`config.json`) on the CircuitPython filesystem. No checksum or integrity protection.

### Default Configuration

```json
{
  "midi": { "tx_channel": 1, "thru_enabled": false },
  "wheel_a": { "mode": "pitch_bend", "cc_number": null, "calibration": { "delta": 517, "center": 658 } },
  "wheel_b": { "mode": "cc", "cc_number": 1, "calibration": { "delta": 700, "center": 909 } },
  "button_a": { "mode": "cc", "cc_number": 64 },
  "button_b": { "mode": "aftertouch", "aftertouch_value": 64 },
  "footswitch": { "mode": "cc", "cc_number": 66 },
  "deadband": 10
}
```

### Validation Ranges

| Config Key | Min | Max |
|-----------|-----|-----|
| midi.tx_channel | 1 | 16 |
| wheel_a/b.cc_number | 0 | 127 |
| button_a/b.cc_number | 0 | 127 |
| button_b.aftertouch_value | 0 | 127 |
| footswitch.cc_number | 0 | 127 |
| deadband | 0 | 63 |

Note: validation runs on `load()` only, not on `set()`. See BUGLIST_PICO.md P4.

## Dev Mode

Hold Button A (GP0) during boot to keep the filesystem writable by USB.

Dev mode pin was corrected from GP17 to GP0 on 2026-03-29 (P1 fix).

## Required Libraries

**Both modes:**
- `adafruit_midi/`
- `adafruit_debouncer.mpy`
- `adafruit_ticks.mpy`
- `adafruit_displayio_ssd1306.mpy`
- `adafruit_display_text/`

**Standalone mode (with MAX3421E) only:**
- `adafruit_usb_host_midi.mpy`
- `max3421e.mpy`

## Module Architecture

```
circuitpython/
  boot.py                # Filesystem + USB setup (runs before code.py)
  code.py                # Main loop (wiring only, no logic)
  config.json            # Persistent configuration (JSON)
  lib/
    hardware/
      pins.py            # ALL pin definitions (single source of truth)
      wheels.py          # AnalogWheel: read, calibrate, deadband
      buttons.py         # DebouncedButton: debounce, double-click
      leds.py            # StatusLED: on/off
      display.py         # OledDisplay: SSD1306 via displayio
      midi_uart.py       # UartMidi: DIN MIDI In/Out
      midi_usb_device.py # UsbDeviceMidi: native USB MIDI (laptop/DAW)
      midi_usb_host.py   # UsbHostMidi: MAX3421E USB Host (optional)
    config.py            # ConfigManager: JSON config with validation
    midi_merge.py        # MidiMerge: N inputs → M outputs, echo prevention
    menu.py              # OnDeviceMenu: on-device config
    calibration.py       # WheelCalibration: interactive routine
```
