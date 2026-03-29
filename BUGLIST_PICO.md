# Bug List — CircuitPython / Pico 2 (`circuitpython/`)

Last reviewed: 2026-03-29, iteration 7 (final)
All bugs fixed: 2026-03-29

## Bugs — ALL FIXED

| # | Severity | Fix | Description |
|---|----------|-----|-------------|
| P1 | **Critical** | boot.py: `board.GP17` → `board.GP0` | **Dev mode reads wrong pin** — now reads correct Button A pin. |
| P2 | **High** | code.py: added "off" and "note" mode handling for all wheels/buttons | **"off" and "note" modes** now skip output or send NoteOn/NoteOff respectively. |
| P3 | **High** | midi_usb_host.py: `except Exception` → `except (OSError, RuntimeError, ValueError)` | **Bare Exception catches** narrowed to specific types. |
| P4 | **High** | config.py: added `_validate_key()` called from `set()` | **No validation on set()** — values now clamped on set. |
| P5 | **Medium** | display.py: `update_values()` now takes mode params, uses decimal for pitch_bend | **Display garbled for pitch bend** — shows signed decimal for pitch, hex for CC. |
| P6 | **Medium** | code.py: checks `merge.has_output` before flush for LED | **Output LED for thru** — LED now lights for all output including forwarded messages. |
| P7 | **Medium** | menu.py: wheel_b forced to CC mode during menu | **Menu broken with pitch_bend wheel_b** — wheel always reads 0..127 in menu. |
| P8 | **Medium** | midi_usb_host.py: 500ms scan cooldown via `_SCAN_COOLDOWN_NS` | **USB bus scan every loop** — now scans at most every 500ms. |
| P9 | **Medium** | code.py: `cc_num is not None` guard on all ControlChange/NoteOn/NoteOff calls | **CC number None crash** — silently skips if cc_number is None. |

## Weaknesses — ALL FIXED

| # | Severity | Fix | Description |
|---|----------|-----|-------------|
| P10 | Medium | midi_merge.py: `_MAX_QUEUE = 64`, breaks/returns when exceeded | **Unbounded MIDI queue** — capped at 64 messages. |
| P11 | Medium | calibration.py: `_TIMEOUT_S = 30`, `_wait_for_press()` returns bool | **Calibration no timeout** — exits after 30s with "Timeout!" message. |
| P12 | Medium | config.py: `save()` returns bool; menu.py shows "SAVE FAILED" | **No save failure feedback** — user sees error on screen. |
| P13 | Medium | config.py: `load()` returns status; code.py shows "Config corrupt!" warning | **Silent config corruption** — corrupt JSON detected and reported on display. |
| P14 | Medium | config.py + code.py: clarifying comments added | **Aftertouch key naming** — documented as global value. |
| P15 | Medium | menu.py: `self._selected = 0` at start of `enter()` | **Menu remembers position** — cursor resets to first item. |
| P16 | Low | wheels.py: expanded ADC_SHIFT comment with board-specific guidance | **ADC resolution documented**. |
| P17 | Low | midi_merge.py: `__init__()` prints warning for mismatched input/output names | **Echo prevention naming** — development-time warning added. |
| P18 | Low | wheels.py: `calibrate()` clamps `self.deadband = min(self.deadband, 63)` | **Deadband validated** against max range. |
| P19 | Low | wheels.py: `v * 8191 // 127` (positive) / `v * 8192 // 128` (negative) | **Pitch bend full range** — now reaches -8192..8191. |
| P20 | Low | config.py: `_merge()` skips keys not in DEFAULTS | **Unknown keys rejected** on load. |
| P21 | Low | tests/ directory with pytest harness over USB MIDI + serial | **Test harness added** — host-side integration tests. |
| P22 | Low | code.py: `__version__ = "2.0.0"`, displayed on startup | **Firmware version string** shown on OLED. |

## Design Concerns — ALL ADDRESSED

| # | Area | Resolution |
|---|------|------------|
| D1 | Menu blocks main loop | Intentional — documented, no change. |
| D2 | No watchdog | **Fixed**: 8s watchdog enabled, fed in main loop + menu + calibration. |
| D3 | Synchronous MIDI output | Acceptable for current volumes — documented, no change. |
| D4 | LED blinks too brief | **Fixed**: `StatusLED` has 50ms minimum on-time. |
| D5 | Config.get() per iteration | Deferred — acceptable on 150MHz processor. |
| D6 | No first-boot config save | **Fixed**: defaults saved to disk on first boot. |

## Files Modified

| File | Changes |
|------|---------|
| `boot.py` | P1: GP17→GP0 |
| `code.py` | P2, P6, P9, P13, P14, P22, D2, D6: modes, LED, guards, version, watchdog, first-boot save |
| `lib/config.py` | P4, P12, P13, P14, P20: validate set(), save returns bool, load returns status, reject unknown keys |
| `lib/menu.py` | P7, P12, P15: wheel_b CC mode, save feedback, cursor reset, watchdog feed |
| `lib/calibration.py` | P11: 30s timeout, watchdog feed |
| `lib/midi_merge.py` | P10, P17: queue cap, naming warning |
| `lib/hardware/midi_usb_host.py` | P3, P8: narrow exceptions, scan cooldown |
| `lib/hardware/display.py` | P5: mode-aware formatting |
| `lib/hardware/wheels.py` | P16, P18, P19: ADC docs, deadband clamp, full pitch range |
| `lib/hardware/leds.py` | D4: 50ms minimum on-time |

## Test Harness (new)

| File | Tests |
|------|-------|
| `tests/conftest.py` | MIDI I/O + serial REPL fixtures |
| `tests/test_midi_device.py` | USB MIDI enumeration, port open, thru |
| `tests/test_config.py` | Config load/save/validation/unknown keys |
| `tests/test_error_recovery.py` | Corrupt config, MIDI flood |
| `tests/test_wheels_buttons.py` | Physical input verification (manual) |
