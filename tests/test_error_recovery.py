# test_error_recovery.py — Error recovery and edge case tests.
#
# Tests that the device recovers gracefully from corrupt config,
# MIDI floods, and other error conditions.

import time

import pytest


def test_corrupt_config_recovery(repl):
    """P13: Device should recover from corrupt config.json."""
    # Write garbage to config.json.
    repl.exec("f = open('config.json', 'w'); f.write('{bad json'); f.close()")
    # Reload config.
    output = repl.exec(
        "from config import ConfigManager; c = ConfigManager('config.json');"
        " status = c.load(); print(status)"
    )
    assert "corrupt" in output, f"Expected 'corrupt' status, got: {output}"
    # Verify defaults are active.
    ch = repl.get_config("midi.tx_channel")
    assert ch.strip() == "1", f"Expected default channel 1, got: {ch}"
    # Restore valid config.
    repl.exec(
        "from config import ConfigManager; c = ConfigManager('config.json');"
        " c.load(); c.save()"
    )


def test_missing_config_returns_defaults(repl):
    """Config should return 'defaults' status when file is missing."""
    # Remove config.json.
    repl.exec("import os; os.remove('config.json')")
    output = repl.exec(
        "from config import ConfigManager; c = ConfigManager('config.json');"
        " status = c.load(); print(status)"
    )
    assert "defaults" in output, f"Expected 'defaults' status, got: {output}"
    # Restore.
    repl.exec(
        "from config import ConfigManager; c = ConfigManager('config.json');"
        " c.load(); c.save()"
    )


def test_device_survives_midi_flood(midi_out, midi_in):
    """P10: Device should handle rapid MIDI input without crashing."""
    import mido

    # Flush input.
    while midi_in.poll():
        pass

    # Send 500 CC messages rapidly.
    for i in range(500):
        midi_out.send(mido.Message("control_change", control=1, value=i % 128))

    time.sleep(1)

    # Device should still be alive — the MIDI port should still work.
    # Send one more message and see if we get any response.
    midi_out.send(mido.Message("control_change", control=1, value=0))
    time.sleep(0.2)

    # If we can still poll without error, the device survived.
    # We don't necessarily expect a response (thru may be off).
    midi_in.poll()  # Should not raise
