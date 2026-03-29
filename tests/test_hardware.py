#!/usr/bin/env python3
"""Hardware diagnostic test — run with the Pico connected via USB.

Tests wheels, buttons, LEDs, and display by reading MIDI output and
prompting for physical interaction. Run directly (not via pytest):

    MIDI_DEVICE_NAME=CircuitPython python3 tests/test_hardware.py

Or via pytest for individual tests:

    MIDI_DEVICE_NAME=CircuitPython pytest tests/test_hardware.py -v -s
"""

import os
import sys
import time

import mido
import pytest

DEVICE_NAME = os.environ.get("MIDI_DEVICE_NAME", "CircuitPython")


def _open_midi_in():
    for name in mido.get_input_names():
        if DEVICE_NAME in name:
            return mido.open_input(name)
    return None


def _flush(port):
    while port.poll():
        pass


def _collect(port, seconds=3):
    """Collect MIDI messages for a given duration."""
    msgs = []
    start = time.time()
    while time.time() - start < seconds:
        msg = port.poll()
        if msg:
            msgs.append(msg)
        time.sleep(0.005)
    return msgs


# ---------------------------------------------------------------------------
# Direct-run mode: interactive hardware walkthrough
# ---------------------------------------------------------------------------

def run_interactive():
    port = _open_midi_in()
    if not port:
        print(f"ERROR: No MIDI device matching '{DEVICE_NAME}' found.")
        print(f"Available: {mido.get_input_names()}")
        sys.exit(1)

    print(f"Connected to: {port.name}")
    print("=" * 60)

    # --- Test 1: Wheel B (Mod Wheel) ---
    print("\n[TEST 1] Wheel B — Mod Wheel (CC 1)")
    print("  Move Wheel B slowly from min to max...")
    _flush(port)
    msgs = _collect(port, 5)
    cc_msgs = [m for m in msgs if m.type == "control_change" and m.control == 1]
    if cc_msgs:
        values = [m.value for m in cc_msgs]
        print(f"  OK: {len(cc_msgs)} CC1 messages, range {min(values)}-{max(values)}")
        if max(values) - min(values) < 20:
            print(f"  WARNING: Small range — wheel may not be moving enough")
    else:
        print("  FAIL: No CC1 messages received")

    # --- Test 2: Wheel A (Pitch Bend) ---
    print("\n[TEST 2] Wheel A — Pitch Bend")
    print("  Move Wheel A from center to extremes...")
    _flush(port)
    msgs = _collect(port, 5)
    pb_msgs = [m for m in msgs if m.type == "pitchwheel"]
    if pb_msgs:
        values = [m.pitch for m in pb_msgs]
        print(f"  OK: {len(pb_msgs)} PitchBend messages, range {min(values)} to {max(values)}")
        has_neg = any(v < -1000 for v in values)
        has_pos = any(v > 1000 for v in values)
        if not has_neg:
            print("  WARNING: No significant negative values — check calibration")
        if not has_pos:
            print("  WARNING: No significant positive values — check calibration")
    else:
        print("  FAIL: No PitchBend messages received")

    # --- Test 3: Button A ---
    print("\n[TEST 3] Button A — CC 64 (Sustain)")
    print("  Press and release Button A...")
    _flush(port)
    msgs = _collect(port, 5)
    cc64 = [m for m in msgs if m.type == "control_change" and m.control == 64]
    if len(cc64) >= 2:
        print(f"  OK: {len(cc64)} CC64 messages — press={cc64[0].value}, release={cc64[-1].value}")
    elif cc64:
        print(f"  PARTIAL: Only {len(cc64)} CC64 message(s) — press+release expected")
    else:
        print("  FAIL: No CC64 messages received")

    # --- Test 4: Button B ---
    print("\n[TEST 4] Button B — Aftertouch")
    print("  Press and release Button B...")
    _flush(port)
    msgs = _collect(port, 5)
    ato = [m for m in msgs if m.type == "aftertouch"]
    if len(ato) >= 2:
        print(f"  OK: {len(ato)} Aftertouch messages — press={ato[0].value}, release={ato[-1].value}")
    elif ato:
        print(f"  PARTIAL: Only {len(ato)} Aftertouch message(s)")
    else:
        print("  FAIL: No Aftertouch messages received")

    # --- Test 5: Footswitch ---
    print("\n[TEST 5] Footswitch — CC 66 (Sostenuto)")
    print("  Press and release the footswitch...")
    _flush(port)
    msgs = _collect(port, 5)
    cc66 = [m for m in msgs if m.type == "control_change" and m.control == 66]
    if len(cc66) >= 2:
        print(f"  OK: {len(cc66)} CC66 messages — press={cc66[0].value}, release={cc66[-1].value}")
    elif cc66:
        print(f"  PARTIAL: Only {len(cc66)} CC66 message(s)")
    else:
        print("  FAIL: No CC66 messages. Footswitch connected?")

    # --- Test 6: LED Activity ---
    print("\n[TEST 6] LED Check")
    print("  LED A (input) should blink when MIDI arrives on DIN In")
    print("  LED B (output) should blink when you move wheels/press buttons")
    if sys.stdin.isatty():
        print("  Move a wheel now — is LED B blinking? (y/n) ", end="", flush=True)
        answer = input().strip().lower()
        if answer == "y":
            print("  OK: LED B confirmed working")
        else:
            print("  NOTE: LED B may be stuck on due to wheel jitter (known issue)")
    else:
        print("  (Skipped — requires interactive terminal)")

    # --- Test 7: Jitter Check ---
    print("\n[TEST 7] Jitter Analysis — leave all controls untouched for 3 seconds...")
    _flush(port)
    time.sleep(0.5)
    _flush(port)
    msgs = _collect(port, 3)
    if msgs:
        types = {}
        for m in msgs:
            key = m.type
            types[key] = types.get(key, 0) + 1
        print(f"  Jitter: {len(msgs)} messages while idle: {types}")
        if len(msgs) > 10:
            print("  WARNING: Significant jitter — consider increasing deadband or adding capacitors")
    else:
        print("  OK: No messages while idle — clean signal")

    # --- Summary ---
    print("\n" + "=" * 60)
    print("Hardware test complete.")
    port.close()


# ---------------------------------------------------------------------------
# Pytest-compatible tests (for CI / automated runs)
# ---------------------------------------------------------------------------

@pytest.mark.manual
def test_jitter_at_rest(midi_in):
    """With controls untouched, minimal MIDI output expected."""
    _flush(midi_in)
    time.sleep(0.3)
    _flush(midi_in)
    msgs = _collect(midi_in, 2)
    # Allow some jitter but flag excessive
    print(f"\n  Idle messages in 2s: {len(msgs)}")
    if len(msgs) > 20:
        pytest.fail(f"Excessive jitter: {len(msgs)} messages in 2 seconds at rest")


if __name__ == "__main__":
    run_interactive()
