# Pico 2 Bug Fix and Host-Side Test Harness — Implementation Plan

**Goal:** Fix all 22 bugs (P1-P22) and address 6 design concerns (D1-D6) in the CircuitPython/Pico 2 MIDI companion controller, then add a host-side pytest test harness (`tests/`) for integration testing over USB MIDI and serial.

**Bug list:** `BUGLIST_PICO.md`
**Hardware reference:** `HARDWARE_PICO.md`

---

## File Map

| File | Changes | Bugs Addressed |
|------|---------|----------------|
| `circuitpython/boot.py` | Fix dev mode pin | P1 |
| `circuitpython/code.py` | Add "off"/"note" modes, guard None CC, fix LED, version, cache config, watchdog, first-boot save | P2, P6, P9, P14, P22, D2, D4, D5, D6 |
| `circuitpython/lib/hardware/midi_usb_host.py` | Narrow exception types, add scan cooldown | P3, P8 |
| `circuitpython/lib/config.py` | Validate on set(), reject unknown keys, return save status, distinguish load errors | P4, P13, P20, D6 |
| `circuitpython/lib/menu.py` | Clamp wheel_b, reset cursor, save feedback | P7, P12, P15 |
| `circuitpython/lib/hardware/display.py` | Fix pitch bend display format | P5 |
| `circuitpython/lib/midi_merge.py` | Bound queue size | P10 |
| `circuitpython/lib/calibration.py` | Add timeout | P11 |
| `circuitpython/lib/hardware/wheels.py` | Fix pitch bend range, document ADC, validate deadband | P16, P18, P19 |
| `circuitpython/lib/hardware/leds.py` | Minimum on-duration | D4 |
| `tests/` | Host-side pytest harness | P21 |

---

## Phase 1: Critical and High-Severity Bugs (P1, P3, P4, P2)

### Task 1.1: Fix dev mode pin in boot.py [P1]
- [ ] Line 37: `board.GP17` → `board.GP0`
- [ ] Line 20: update comment `GP17` → `GP0`

### Task 1.2: Narrow exception handling in midi_usb_host.py [P3]
- [ ] Lines 61, 82, 107, 125: `except Exception:` → `except (OSError, RuntimeError, ValueError):`

### Task 1.3: Add validation to config.set() [P4]
- [ ] Add `_validate_key(dotpath)` method that clamps a single key
- [ ] Call it at the end of `set()`

### Task 1.4: Add "off" and "note" modes in code.py [P2, P9]
- [ ] Wheels: guard with `if mode != "off"`, guard `cc_num is not None`
- [ ] Buttons: add "off" skip, add "note" mode (NoteOn/NoteOff), guard `cc_num is not None`
- [ ] Add `NoteOn`/`NoteOff` imports

**Review checkpoint:** Device boots, dev mode works, all modes send correct MIDI.

---

## Phase 2: Medium-Severity Bugs (P5, P6, P7, P8)

### Task 2.1: Fix pitch bend display [P5]
- [ ] Add mode params to `update_values()` — decimal for pitch_bend, hex for CC
- [ ] Update call site in code.py
- [ ] Widen label initial text for pitch bend values

### Task 2.2: Fix output LED for thru messages [P6]
- [ ] Check `merge.has_output` before flush, light LED for any queued output

### Task 2.3: Fix menu with pitch_bend wheel_b [P7]
- [ ] Temporarily force wheel_b to CC mode during menu, restore on exit

### Task 2.4: Add USB scan cooldown [P8]
- [ ] Add `_SCAN_COOLDOWN_NS = 500_000_000` and `_last_scan_time` in midi_usb_host.py
- [ ] Guard `_find_midi_device()` call with cooldown check

**Review checkpoint:** Display correct, LED for thru, menu safe, no excessive USB scanning.

---

## Phase 3: Medium Weaknesses (P10-P15)

### Task 3.1: Bound MIDI queue [P10]
- [ ] `_MAX_QUEUE = 64`, break/return when exceeded

### Task 3.2: Calibration timeout [P11]
- [ ] 30s timeout in `_wait_for_press()`, return gracefully

### Task 3.3: Save failure feedback [P12]
- [ ] `save()` returns bool, menu shows "SAVE FAILED" on error

### Task 3.4: Distinguish missing vs. corrupt config [P13]
- [ ] `load()` returns status string, code.py shows warning for "corrupt"

### Task 3.5: Document aftertouch key [P14]
- [ ] Add clarifying comments in config.py and code.py

### Task 3.6: Reset menu cursor [P15]
- [ ] `self._selected = 0` at start of `enter()`

**Review checkpoint:** Queue bounded, calibration has timeout, save feedback works.

---

## Phase 4: Low-Severity Issues and Design Concerns

### Task 4.1: Document ADC shift [P16]
### Task 4.2: Strengthen echo prevention naming [P17]
### Task 4.3: Validate deadband [P18]
### Task 4.4: Fix pitch bend full range [P19]
- [ ] Replace `v * 64` with `v * 8191 // 127` (positive) / `v * 8192 // 128` (negative)

### Task 4.5: Reject unknown config keys [P20]
### Task 4.6: Add firmware version [P22]
### Task 4.7: Design concerns [D1-D6]
- [ ] D2: Add watchdog (5s timeout, feed in main loop + menu + calibration)
- [ ] D4: LED minimum on-time (50ms)
- [ ] D5: Cache config values as locals, reload after menu
- [ ] D6: Save defaults on first boot

**Review checkpoint:** All P-items and D-items addressed.

---

## Phase 5: Host-Side Test Harness

### Task 5.1: Project setup
- [ ] `tests/requirements.txt` (pytest, mido, python-rtmidi, pyserial)
- [ ] `tests/README.md`
- [ ] `.vscode/settings.json` pytest config
- [ ] `pyproject.toml` with pytest markers

### Task 5.2: Test fixtures (`tests/conftest.py`)
- [ ] `midi_out` / `midi_in` fixtures via mido
- [ ] `serial_repl` / `repl` fixtures via pyserial

### Task 5.3: USB MIDI device tests (`tests/test_midi_device.py`)
- [ ] Device enumeration, port open, MIDI thru

### Task 5.4: Wheel/button tests (`tests/test_wheels_buttons.py`)
- [ ] Manual-marked tests for physical input verification

### Task 5.5: Config tests (`tests/test_config.py`)
- [ ] Load, set, persist, validation clamping

### Task 5.6: Error recovery tests (`tests/test_error_recovery.py`)
- [ ] Corrupt config recovery, MIDI flood survival

**Review checkpoint:** All tests pass with device connected.

---

## Phase 6: Verification

- [ ] Run automated tests: `pytest tests/ -v`
- [ ] Run manual tests: `pytest tests/ -v -m manual`
- [ ] Update BUGLIST_PICO.md marking bugs as fixed
- [ ] Smoke test: boot, dev mode, all modes, menu, calibration timeout, thru LED, watchdog