# test_config.py — Config persistence and validation tests via REPL.
#
# Communicates with the Pico's CircuitPython REPL over USB serial to
# verify config load/save/validation behavior.

import pytest


def test_config_loads_with_valid_defaults(repl):
    """Config should load with a valid default channel."""
    output = repl.get_config("midi.tx_channel")
    assert output is not None
    ch = int(output)
    assert 1 <= ch <= 16, f"Channel {ch} out of range"


def test_config_set_and_persist(repl):
    """Setting a value should persist across load cycles."""
    repl.exec(
        "from config import ConfigManager; c = ConfigManager('config.json');"
        " c.load(); c.set('midi.tx_channel', 5); c.save()"
    )
    output = repl.get_config("midi.tx_channel")
    assert output.strip() == "5"
    # Restore default.
    repl.exec(
        "from config import ConfigManager; c = ConfigManager('config.json');"
        " c.load(); c.set('midi.tx_channel', 1); c.save()"
    )


def test_config_validation_clamps_high(repl):
    """P4: set() should clamp out-of-range values (high)."""
    repl.exec(
        "from config import ConfigManager; c = ConfigManager('config.json');"
        " c.load(); c.set('midi.tx_channel', 200); c.save()"
    )
    output = repl.get_config("midi.tx_channel")
    assert output.strip() == "16", f"Expected 16 (clamped), got {output}"
    # Restore.
    repl.exec(
        "from config import ConfigManager; c = ConfigManager('config.json');"
        " c.load(); c.set('midi.tx_channel', 1); c.save()"
    )


def test_config_validation_clamps_low(repl):
    """P4: set() should clamp out-of-range values (low)."""
    repl.exec(
        "from config import ConfigManager; c = ConfigManager('config.json');"
        " c.load(); c.set('deadband', -5); c.save()"
    )
    output = repl.get_config("deadband")
    assert output.strip() == "0", f"Expected 0 (clamped), got {output}"
    # Restore.
    repl.exec(
        "from config import ConfigManager; c = ConfigManager('config.json');"
        " c.load(); c.set('deadband', 10); c.save()"
    )


def test_config_rejects_unknown_keys(repl):
    """P20: Unknown keys from config.json should be ignored on load."""
    # Write config with an unknown key.
    repl.exec(
        "import json; "
        "f = open('config.json', 'r'); data = json.load(f); f.close(); "
        "data['unknown_key'] = 42; "
        "f = open('config.json', 'w'); json.dump(data, f); f.close()"
    )
    # Reload and check that unknown_key is not in config.
    output = repl.exec(
        "from config import ConfigManager; c = ConfigManager('config.json');"
        " c.load(); print('unknown_key' in c._data)"
    )
    assert "False" in output, f"Unknown key should have been rejected: {output}"
    # Clean up by saving valid config.
    repl.exec(
        "from config import ConfigManager; c = ConfigManager('config.json');"
        " c.load(); c.save()"
    )
