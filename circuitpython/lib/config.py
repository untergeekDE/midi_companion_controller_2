# config.py — ConfigManager for MIDI Companion Controller 2
#
# This module provides persistent configuration management backed by a JSON
# file on the CircuitPython flash filesystem. It supports:
#   - Loading settings from JSON with fallback to built-in defaults
#   - Saving settings back to JSON
#   - Dot-notation access and mutation of nested values
#   - Validation / clamping of numeric values to legal ranges
#
# IMPORTANT: Only the standard `json` module and built-in file I/O are used.
# The `copy` module is not available in CircuitPython, so deep copies are
# performed by a hand-written helper (_deep_copy). This means config.py can
# be tested without modification on desktop Python 3.

import json

# ---------------------------------------------------------------------------
# Built-in defaults
# ---------------------------------------------------------------------------
# These values are used when the config file is missing, unreadable, or
# contains invalid JSON. They represent the factory defaults for all settings.
#
# Structure:
#   midi        — global MIDI transport settings
#   wheel_a     — expression wheel A (left wheel / pitch bend wheel)
#   wheel_b     — expression wheel B (right wheel / modulation wheel)
#   button_a    — Button A (e.g. sustain)
#   button_b    — Button B (e.g. aftertouch trigger)
#   footswitch  — external footswitch jack
#   deadband    — ADC deadband in raw counts (filters jitter near center)

DEFAULTS = {
    "midi": {
        "tx_channel": 1,        # MIDI channel 1–16 for outgoing messages
        "thru_enabled": False,  # Whether to forward incoming DIN MIDI to out
    },
    "wheel_a": {
        "mode": "pitch_bend",   # "pitch_bend" | "cc" | "off"
        "cc_number": None,      # CC number used when mode == "cc" (0–127)
        "calibration": {
            "delta": 517,       # Half-range of the ADC reading (raw counts)
            "center": 658,      # ADC reading at physical center position
        },
    },
    "wheel_b": {
        "mode": "cc",           # "pitch_bend" | "cc" | "off"
        "cc_number": 1,         # CC 1 = Modulation Wheel (GM standard)
        "calibration": {
            "delta": 700,
            "center": 909,
        },
    },
    "button_a": {
        "mode": "cc",           # "cc" | "note" | "off"
        "cc_number": 64,        # CC 64 = Sustain Pedal (GM standard)
    },
    "button_b": {
        "mode": "aftertouch",   # "aftertouch" | "cc" | "note" | "off"
        "aftertouch_value": 64, # Global aftertouch value for ALL buttons/footswitch (0–127)
    },
    "footswitch": {
        "mode": "cc",           # "cc" | "note" | "off"
        "cc_number": 66,        # CC 66 = Sostenuto Pedal (GM standard)
    },
    "deadband": 10,             # ADC counts either side of center to ignore
}

# ---------------------------------------------------------------------------
# Validation ranges
# ---------------------------------------------------------------------------
# Maps dot-notation config keys to (min, max) inclusive numeric ranges.
# Values outside the range are clamped during _validate().
# Keys not listed here are not range-checked.

VALIDATION = {
    "midi.tx_channel":          (1, 16),
    "wheel_a.cc_number":        (0, 127),
    "wheel_b.cc_number":        (0, 127),
    "button_a.cc_number":       (0, 127),
    "button_b.cc_number":       (0, 127),
    "button_b.aftertouch_value":(0, 127),
    "footswitch.cc_number":     (0, 127),
    "deadband":                 (0, 63),
}


# ---------------------------------------------------------------------------
# Helper: deep copy without the `copy` module
# ---------------------------------------------------------------------------

def _deep_copy(obj):
    """Recursively copy a dict/list structure without using the `copy` module.

    CircuitPython does not include `copy.deepcopy`, so we roll our own.
    Only dicts, lists, and scalar values (int, float, str, bool, None) are
    expected in a JSON-derived config tree — that covers everything we need.

    Args:
        obj: The object to copy. May be a dict, list, or scalar.

    Returns:
        An independent deep copy of obj.
    """
    if isinstance(obj, dict):
        # Recursively copy each value in the dictionary.
        return {k: _deep_copy(v) for k, v in obj.items()}
    if isinstance(obj, list):
        # Recursively copy each element in the list.
        return [_deep_copy(item) for item in obj]
    # Scalars (int, float, str, bool, None) are immutable — return as-is.
    return obj


# ---------------------------------------------------------------------------
# ConfigManager
# ---------------------------------------------------------------------------

class ConfigManager:
    """Manages a JSON configuration file with dot-notation access.

    Usage example::

        cfg = ConfigManager("/config.json")
        cfg.load()                          # Load from disk (or use defaults)
        ch = cfg.get("midi.tx_channel")     # Read a nested value
        cfg.set("midi.tx_channel", 3)       # Write a nested value
        cfg.save()                          # Persist to disk

    All public methods are safe to call on a read-only filesystem — errors
    are caught and silently ignored so the device keeps running.
    """

    def __init__(self, path):
        """Initialise the manager without loading anything from disk yet.

        Args:
            path (str): Absolute path to the JSON config file, e.g.
                        "/config.json" on the CircuitPython flash drive.
        """
        # Store the file path for later load()/save() calls.
        self._path = path

        # Start with a fresh deep copy of the built-in defaults so each
        # ConfigManager instance has its own independent dict tree.
        self._data = _deep_copy(DEFAULTS)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def load(self):
        """Load configuration from the JSON file, merging over defaults.

        Behaviour:
        - If the file is missing or unreadable (OSError), fall back to
          defaults silently. This is normal on first boot.
        - If the file contains invalid JSON (ValueError), fall back to
          defaults silently and log a warning.
        - If a key lookup fails unexpectedly (KeyError), fall back to
          defaults silently.
        - After a successful merge, _validate() is called to clamp any
          out-of-range numeric values.

        The merge strategy ensures that keys present in DEFAULTS but
        missing from the file still receive their default values — so
        adding new config keys to a future firmware version works
        automatically without resetting the user's existing settings.

        Returns:
            str: "loaded" if file loaded successfully, "defaults" if file
                 was missing (first boot), "corrupt" if JSON was malformed.
        """
        # Always reset to a clean copy of defaults first, so that any key
        # missing from the file on disk falls back to its default value.
        self._data = _deep_copy(DEFAULTS)

        status = "loaded"
        try:
            with open(self._path, "r") as f:
                loaded = json.load(f)
            # Recursively merge the loaded dict over our defaults.
            self._merge(self._data, loaded)
        except OSError:
            # File not found, not readable, or filesystem error.
            # Silently continue with defaults — this is expected on first boot.
            status = "defaults"
        except (ValueError, KeyError):
            # json.load() raised ValueError for malformed JSON, or
            # unexpected key error during merge — keep defaults.
            status = "corrupt"

        # Clamp any out-of-range values regardless of where data came from.
        self._validate()

        return status

    def save(self):
        """Write the current configuration to the JSON file.

        Errors (e.g. read-only filesystem in developer mode) are caught and
        silently ignored — the device continues running with its in-memory
        config.

        Returns:
            bool: True if save succeeded, False on OSError (read-only
                  filesystem, storage full, etc.).
        """
        try:
            with open(self._path, "w") as f:
                json.dump(self._data, f)
            return True
        except OSError:
            # Filesystem is read-only (developer mode) or storage is full.
            # Nothing we can do — silently skip the save.
            return False

    def get(self, dotpath):
        """Read a config value using dot notation.

        Args:
            dotpath (str): Dot-separated key path, e.g. "midi.tx_channel"
                           or "wheel_a.calibration.delta".

        Returns:
            The value at that path, which may be any JSON-compatible type
            (int, float, str, bool, None, dict, list).

        Raises:
            KeyError: If any segment of the path does not exist.
        """
        node = self._data
        for key in dotpath.split("."):
            # Each iteration descends one level into the dict tree.
            node = node[key]
        return node

    def set(self, dotpath, value):
        """Write a config value using dot notation.

        Intermediate dicts are traversed but not created — the full path
        must already exist (it will, as long as the key is in DEFAULTS).

        Args:
            dotpath (str): Dot-separated key path, e.g. "midi.tx_channel".
            value:         The new value to store.

        Raises:
            KeyError: If any intermediate segment does not exist.
        """
        keys = dotpath.split(".")
        node = self._data

        # Walk down to the parent of the final key.
        for key in keys[:-1]:
            node = node[key]

        # Set the leaf value.
        node[keys[-1]] = value

        # Clamp to valid range if a validation rule exists for this key.
        self._validate_key(dotpath)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _merge(self, base, override):
        """Recursively merge override dict into base dict (in place).

        For each key in override:
        - If both base[key] and override[key] are dicts, recurse.
        - Otherwise, overwrite base[key] with override[key].

        Keys present in base but absent from override are left untouched,
        which is exactly what we want for forward-compatible config files.

        Args:
            base (dict):     The dict to merge into (mutated in place).
            override (dict): The dict whose values take precedence.
        """
        for key, value in override.items():
            if key not in base:
                # Reject unknown keys not present in defaults — silently skip.
                continue
            if isinstance(base[key], dict) and isinstance(value, dict):
                # Both sides are dicts — recurse to merge nested structure.
                self._merge(base[key], value)
            else:
                # Scalar, list, or type mismatch — override wins outright.
                base[key] = value

    def _validate_key(self, dotpath):
        """Clamp a single config value to its declared valid range.

        Looks up the dotpath in the VALIDATION dict. If a range exists and
        the current value is numeric, clamps it in place using direct dict
        traversal (avoids calling self.set() which would recurse).

        Non-numeric values (e.g. None) are left untouched.

        Args:
            dotpath (str): Dot-separated key path, e.g. "midi.tx_channel".
        """
        if dotpath not in VALIDATION:
            return

        lo, hi = VALIDATION[dotpath]

        # Read the current value via direct traversal.
        keys = dotpath.split(".")
        node = self._data
        for key in keys[:-1]:
            node = node[key]

        value = node[keys[-1]]

        if not isinstance(value, (int, float)):
            return

        clamped = max(lo, min(hi, value))
        if clamped != value:
            node[keys[-1]] = clamped

    def _validate(self):
        """Clamp numeric config values to their declared valid ranges.

        Iterates over the VALIDATION dict, reads the current value at each
        dotpath, and replaces it with the clamped value if it is out of range.

        Non-numeric values (e.g. None for an unused cc_number) are skipped
        silently, so optional fields that are legitimately None don't cause
        errors.
        """
        for dotpath, (lo, hi) in VALIDATION.items():
            try:
                value = self.get(dotpath)
            except KeyError:
                # Path doesn't exist in current data — skip.
                continue

            # Only clamp values that are actually numeric.
            # None is a valid sentinel for "not configured".
            if not isinstance(value, (int, float)):
                continue

            # Clamp: if below min, set to min; if above max, set to max.
            clamped = max(lo, min(hi, value))
            if clamped != value:
                self.set(dotpath, clamped)
