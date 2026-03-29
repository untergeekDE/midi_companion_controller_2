# boot.py — runs before code.py on every power-on or reset
#
# CircuitPython executes boot.py first, before the main program (code.py).
# This is the only place where certain privileged operations are allowed:
#   - Remounting the filesystem with write access for CircuitPython
#   - Enabling or disabling USB interfaces (MIDI, HID, CDC, etc.)
#
# IMPORTANT: Changes made here take effect on the NEXT boot, not immediately.
# If you modify this file over USB, you must reset the board before the
# new settings apply.

import board
import digitalio
import storage
import usb_hid

# ---------------------------------------------------------------------------
# Developer Mode Detection
# ---------------------------------------------------------------------------
# Button A (GP0) is used as the "developer mode" trigger during boot.
#
# Behavior:
#   - If Button A is held DOWN when the board powers on or resets,
#     we skip the filesystem remount. This leaves the filesystem writable
#     by the host computer (normal CircuitPython behavior), which is useful
#     when you need to edit files over USB without fighting write conflicts.
#
#   - If Button A is NOT held, we remount the filesystem so that
#     CircuitPython (code.py) can write to it at runtime. This is required
#     for saving settings, logs, or any persistent state.
#
# The button uses INPUT_PULLUP, meaning:
#   - Pin reads HIGH (True)  when the button is NOT pressed (pull-up keeps it high)
#   - Pin reads LOW  (False) when the button IS pressed (button connects pin to GND)
# This is called "active low" logic.

button_a = digitalio.DigitalInOut(board.GP0)
button_a.direction = digitalio.Direction.INPUT
button_a.pull = digitalio.Pull.UP

# Read the button state once at boot.
# If it's LOW (False), the button is being held — enter developer mode.
developer_mode = not button_a.value  # True when button is pressed (active low)

# Release the pin immediately. We only needed it for the boot check.
# Leaving it configured here would prevent code.py from using it later.
button_a.deinit()

# ---------------------------------------------------------------------------
# Filesystem Remount
# ---------------------------------------------------------------------------
# By default, CircuitPython mounts the internal flash filesystem as read-only
# for the running Python code, while the USB mass storage interface can write
# to it. This allows editing files over USB but prevents code.py from writing
# to the filesystem at runtime.
#
# For this project, code.py needs to write configuration and state to flash
# (e.g., saved presets, learned MIDI mappings). So we flip the ownership:
# CircuitPython gets read/write access, and USB mass storage becomes read-only
# (or is effectively disabled as a writable drive).
#
# We skip this in developer mode so you can still edit files over USB.

if not developer_mode:
    # Remount "/" as writable by CircuitPython.
    # readonly=False means CircuitPython can write; USB mass storage becomes
    # read-only from the host's perspective.
    storage.remount("/", readonly=False)

# ---------------------------------------------------------------------------
# USB Interface Configuration
# ---------------------------------------------------------------------------
# USB MIDI is LEFT ENABLED so the controller can operate in two modes:
#   1. Standalone: merge Launchpad (USB Host) + local controls → DIN MIDI Out
#   2. Connected to laptop: also appear as a USB MIDI device for DAW use
#
# USB HID is disabled because this device has no keyboard/mouse/gamepad
# functionality.
#
# USB CDC (serial console) remains enabled for debugging via the REPL.

# USB MIDI stays enabled — do NOT call usb_midi.disable().

# Disable the USB HID interface. This device has no keyboard/mouse/gamepad
# functionality exposed over USB.
usb_hid.disable()
