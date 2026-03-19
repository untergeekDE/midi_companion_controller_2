# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Teensy 3.2-based MIDI controller firmware with two analog wheels (pitch bend + mod wheel), two buttons, a footswitch, two LEDs, and a 128x64 OLED display. Sends MIDI over both serial (DIN) and USB simultaneously.

## Build & Upload

No `platformio.ini` or Arduino project file is currently checked in. The code can be built with either:

- **Arduino/Teensyduino IDE**: Open `src/main.cpp`, select board "Teensy 3.2", set Tools → USB Type → MIDI. Compile and upload.
- **PlatformIO**: Create a `platformio.ini` targeting `teensy31` with `build_flags = -UUSB_SERIAL -DUSB_MIDI`. Run `pio run` to build, `pio run -t upload` to flash.

## Dependencies

- [MIDI Library](https://github.com/FortySevenEffects/arduino_midi_library) (v4.3+)
- [Bounce2](https://github.com/thomasfredericks/Bounce2) — button debouncing
- [OLED_I2C_128x64_Monochrome](https://github.com/untergeekDE/OLED_I2C_128x64_Monochrome_Library) — custom OLED library (must be installed manually into Arduino/Teensy libraries folder)
- Standard Arduino libraries: SPI, Wire, EEPROM

## Architecture

Everything lives in a single file: `src/main.cpp`. Key sections in order:

1. **MIDI setup** (~line 77): Dual MIDI instances — `midiA` (serial/DIN) and `midiB` (USB, conditional on `HAS_USBMIDI`)
2. **PROGMEM strings** (~line 128): All display text stored in flash as `const byte[]` arrays
3. **Custom print()** (~line 167): Overloaded `print()` functions for the OLED — handles PROGMEM, byte*, and char* strings
4. **Wheel reading** (~line 234): `readWheel()` reads analog input with calibration offset and deadband, returns centered (signed, pitch bend) or uncentered (unsigned, CC) values
5. **EEPROM/NVR** (~line 269): 32-byte EEPROM layout storing CC assignments, wheel calibration, MIDI channel, thru setting, deadband. Protected by additive checksum at byte 0
6. **Calibration** (~line 345): Interactive wheel calibration routine storing offsets to EEPROM
7. **Menu** (~line 395): On-device configuration menu (double-click button A to enter). Navigate with button A, set values with wheel B or button B
8. **setup()** (~line 542): Hardware init, EEPROM validation, MIDI begin
9. **loop()** (~line 605): Main loop — reads MIDI input (with optional thru), scans all buttons and wheels, sends MIDI output, updates display

## EEPROM Memory Map

| Byte(s) | Purpose |
|---------|---------|
| 0 | Checksum |
| 1 | Wheel A CC (0x80 = pitch bend) |
| 2 | Wheel B CC (0x80 = pitch bend) |
| 3-5 | Button A/B/C CC (0x80 = aftertouch) |
| 6-7 | Wheel A delta (calibration offset) |
| 8-9 | Wheel A center |
| 12-13 | Wheel B delta |
| 14-15 | Wheel B center |
| 18 | TX MIDI channel |
| 19 | Aftertouch default value |
| 20 | MIDI Thru on/off |
| 21 | Deadband width |

## Hardware Pin Mapping

- Wheels: A22 (pitch), A23 (mod)
- Buttons: D17 (A), D15 (B), D13 (C/footswitch) — INPUT_PULLUP
- LEDs: D16 (MIDI in), D14 (MIDI out)
- OLED: I2C (Wire), reset on D4
