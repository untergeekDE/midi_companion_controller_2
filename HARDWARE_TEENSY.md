# Hardware Documentation — Teensy 3.2

Implementation: `src/main.cpp`

## Board
- **MCU**: Teensy 3.2 (MK20DX256, ARM Cortex-M4, 72 MHz)
- **USB**: Native USB MIDI (via Teensyduino)
- **ADC**: 10-bit (0-1023)

## Pin Assignments

| Pin | Type | Function | Notes |
|-----|------|----------|-------|
| A22 | Analog | Wheel A (Pitch Bend) | Left wheel |
| A23 | Analog | Wheel B (Modulation) | Right wheel |
| D17 | Digital | Button A | Menu / navigation, INPUT_PULLUP |
| D15 | Digital | Button B | Confirm / set, INPUT_PULLUP |
| D13 | Digital | Button C (Footswitch) | 1/4" jack, INPUT_PULLUP |
| D16 | Digital | LED 1 | MIDI input activity |
| D14 | Digital | LED 2 | MIDI output activity |
| D4 | Digital | OLED Reset | SSD1306 reset pin |
| Serial1 TX/RX | UART | DIN MIDI In/Out | 31,250 baud |
| I2C (Wire) | I2C | OLED Display | SSD1306 128x64 |

## Components
- **Display**: SSD1306 128x64 I2C OLED (custom `OLED_I2C_128x64_Monochrome` library)
- **MIDI**: DIN-5 In/Out via Serial1 UART at 31,250 baud
- **Buttons**: Momentary pushbuttons, active low (INPUT_PULLUP)
- **Footswitch**: 1/4" TRS jack, active low (INPUT_PULLUP)
- **LEDs**: Standard LEDs, HIGH = on
- **Wheels**: Linear potentiometers, ~256 ADC count range

## Wheel Calibration Defaults
| Parameter | Wheel A (Pitch) | Wheel B (Mod) |
|-----------|-----------------|---------------|
| Center | 658 | 909 |
| Delta | 517 | 700 |

## EEPROM Memory Map (32 bytes)

| Byte(s) | Purpose | Default |
|---------|---------|---------|
| 0 | Checksum (additive sum) | — |
| 1 | Wheel A CC (0x80 = pitch bend) | 0x80 |
| 2 | Wheel B CC (0x80 = pitch bend) | 1 (CC1 Mod) |
| 3 | Button A CC (0x80 = aftertouch) | 64 (Sustain) |
| 4 | Button B CC (0x80 = aftertouch) | 0x80 |
| 5 | Button C (Footswitch) CC | 64 (Sustain) — code default; `#define CCBUTTON_C 66` is unused |
| 6-7 | Wheel A delta (calibration) | 517 |
| 8-9 | Wheel A center | 658 |
| 10-11 | Wheel A Hi (unused) | 256 |
| 12-13 | Wheel B delta | 700 |
| 14-15 | Wheel B center | 909 |
| 16-17 | Wheel B Hi (unused) | 256 |
| 18 | TX MIDI channel (1-16) | 1 |
| 19 | Aftertouch default value | 64 |
| 20 | MIDI Thru on/off | 0 |
| 21 | Deadband width (0-63) | 10 |
| 22-31 | Unused | 0 |

## Schematic

Fritzing file: `midicompanion2-0.fzz`
