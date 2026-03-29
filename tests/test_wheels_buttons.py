# test_wheels_buttons.py — Manual tests requiring physical interaction.
#
# These tests prompt the user to move a wheel or press a button, then
# verify the expected MIDI output. Run with: pytest -m manual -v -s

import time

import pytest


@pytest.mark.manual
def test_wheel_a_sends_pitch_bend(midi_in):
    """Move Wheel A — expect pitch bend messages."""
    # Flush.
    while midi_in.poll():
        pass

    print("\n>>> Move Wheel A (pitch bend) now... (5 seconds)")
    start = time.time()
    messages = []
    while time.time() - start < 5:
        msg = midi_in.poll()
        if msg and msg.type == "pitchwheel":
            messages.append(msg)
        time.sleep(0.01)

    assert len(messages) > 0, "No pitch bend messages received — did you move wheel A?"
    print(f"    Received {len(messages)} pitch bend messages")


@pytest.mark.manual
def test_wheel_b_sends_cc(midi_in):
    """Move Wheel B — expect CC 1 (mod wheel) messages."""
    while midi_in.poll():
        pass

    print("\n>>> Move Wheel B (mod wheel) now... (5 seconds)")
    start = time.time()
    messages = []
    while time.time() - start < 5:
        msg = midi_in.poll()
        if msg and msg.type == "control_change":
            messages.append(msg)
        time.sleep(0.01)

    assert len(messages) > 0, "No CC messages received — did you move wheel B?"
    print(f"    Received {len(messages)} CC messages, last value: {messages[-1].value}")


@pytest.mark.manual
def test_button_a_sends_cc(midi_in):
    """Press and release Button A — expect CC 64 on/off."""
    while midi_in.poll():
        pass

    print("\n>>> Press and release Button A now... (5 seconds)")
    start = time.time()
    messages = []
    while time.time() - start < 5:
        msg = midi_in.poll()
        if msg and msg.type == "control_change":
            messages.append(msg)
        time.sleep(0.01)

    assert len(messages) >= 2, (
        f"Expected at least 2 CC messages (press + release), got {len(messages)}"
    )
    assert messages[0].value == 127, f"First CC should be 127 (press), got {messages[0].value}"
    assert messages[-1].value == 0, f"Last CC should be 0 (release), got {messages[-1].value}"
    print(f"    Button A: press={messages[0].value}, release={messages[-1].value}")


@pytest.mark.manual
def test_footswitch_sends_cc(midi_in):
    """Press and release footswitch — expect CC 66 on/off."""
    while midi_in.poll():
        pass

    print("\n>>> Press and release footswitch now... (5 seconds)")
    start = time.time()
    messages = []
    while time.time() - start < 5:
        msg = midi_in.poll()
        if msg and msg.type == "control_change":
            messages.append(msg)
        time.sleep(0.01)

    assert len(messages) >= 2, (
        f"Expected at least 2 CC messages (press + release), got {len(messages)}"
    )
    print(f"    Footswitch: press={messages[0].value}, release={messages[-1].value}")
