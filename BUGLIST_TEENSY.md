# Bug List — Teensy 3.2 (`src/main.cpp`)

Last reviewed: 2026-03-29, iteration 5 (Teensy review complete — further iterations focused on Pico only)

## Bugs

| # | Severity | Location | Description |
|---|----------|----------|-------------|
| T1 | **Critical** | Lines 743, 771 | **CC modulo is wrong**: `EEPROM[1] % 0x7f` uses 127, not 128. CC value 127 wraps to 0. Should be `% 128` or bounds-check to 0-127. |
| T2 | **Critical** | Lines 656, 688, 708 | **Aftertouch never triggers**: `EEPROM[3] > 0x80` should be `>= 0x80`. The aftertouch sentinel is 0x80 (128), but `128 > 128` is false. The aftertouch branch is unreachable for all three buttons. Instead, CC 128 is sent (out of MIDI spec). |
| T3 | **High** | Lines 695, 715 | **Button B/C CC polarity inverted**: Button A uses `!button1.read() * 127` (pressed=127, released=0), but buttons B and C use `button2.read() ? 127 : 0` (pressed=0, released=127). CC messages are backwards — sustain activates on release, not press. |
| T4 | **Medium** | Line 737 | **Pitch bend scaling short of full range**: `value * 64` gives max 127x64=8128, but MIDI pitch bend max is 8191. Positive range is ~0.8% short of full scale. |

## Weaknesses

| # | Severity | Location | Description |
|---|----------|----------|-------------|
| T5 | Medium | Lines 636-647 | **Auto-adopts incoming MIDI channel**: `MidiTxCh = MidiRxCh` silently changes the TX channel to match any incoming message. |
| T6 | Medium | Lines 612-761 | **LED blinks invisible**: LEDs turn on/off within a single loop iteration (<2ms). No minimum on-time. |
| T7 | Medium | Line 417 | **Menu has no timeout**: `for(;;)` blocks indefinitely. No watchdog or auto-exit. |
| T8 | Medium | Lines 378-379 | **Calibration assumes ~256 ADC range**: `center = delta + 128` hardcoded. |
| T9 | Medium | Lines 653, 685, 705 | **Buttons send CC on every update**: RefreshFlag causes continuous CC sends, not just on edges. |
| T10 | Low | Line 500 | **Menu calls itself recursively**: `calibrate()` -> `menu()` recursive. Stack grows per calibration. |
| T11 | Low | Line 280 | **Footswitch default CC mismatch**: `nvrInits[5] = 64` but `#define CCBUTTON_C 66` unused. |
| T12 | Low | Lines 252-264 | **Deadband asymmetric**: Pitch wheel symmetric, CC wheel low-end only. Intentional but undocumented. |
| T13 | Low | EEPROM byte 0 | **Weak checksum**: Additive sum mod 256. No CRC. |
| T14 | Low | Lines 619-625 | **MIDI Thru UART only**: USB thru block empty. USB input never forwarded. |
| T15 | Low | EEPROM 10-11, 16-17 | **Dead EEPROM entries**: WheelA/B Hi values written but never read. |
| T16 | Low | Line 591 | **No input channel filtering**: MIDI_CHANNEL_OMNI only. |
| T17 | Low | Main loop | **USB MIDI input never read**: `midiB.read()` never called. Output-only. |
| T18 | Low | Line 79 | **USB uses serial transport**: `SerialUSB` via 47 Effects library, not Teensyduino's native `usbMIDI`. |
| T19 | Info | Lines 214-215 | **EEPROM display access**: Low risk — protected by nvrInit on first boot. |

## Design Concerns

| # | Area | Description |
|---|------|-------------|
| D1 | Menu blocks main loop | No MIDI processing during menu interaction. |
| D2 | No watchdog | Software hangs require power cycle. |
| D3 | Synchronous MIDI output | Slow DIN output can delay USB timing. |
| D4 | Single-file architecture | 788-line monolithic file. |
| D5 | `#include <SPI.h>` unused | SPI library included but only I2C (Wire) used. |
