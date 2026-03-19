# calibration.py — Interactive wheel calibration routine.
#
# Guides the user through calibrating the pitch bend and mod wheel:
# 1. Display instructions on the OLED
# 2. Wait for button press
# 3. Read current analog values as center/delta reference points
# 4. Save calibration values to ConfigManager
#
# This routine blocks the main loop (MIDI processing stops), matching
# the original Teensy behavior. It is only invoked from the on-device menu.
#
# Usage:
#     from calibration import WheelCalibration
#     cal = WheelCalibration(display, wheel_a, wheel_b, button_b, config)
#     cal.run()

import time
from hardware.wheels import _ADC_SHIFT


class WheelCalibration:
    """Interactive calibration for the analog wheels.

    Measures the resting position of each wheel and stores the values
    in the ConfigManager. The pitch bend wheel's center position and
    the mod wheel's zero position are determined.

    Args:
        display: An OledDisplay instance for showing instructions.
        wheel_a: The AnalogWheel for wheel A (pitch bend).
        wheel_b: The AnalogWheel for wheel B (mod wheel).
        button_b: The DebouncedButton used to confirm (Button B).
        config: The ConfigManager instance to save calibration values.
    """

    def __init__(self, display, wheel_a, wheel_b, button_b, config):
        self._display = display
        self._wheel_a = wheel_a
        self._wheel_b = wheel_b
        self._button = button_b
        self._config = config

    def run(self):
        """Run the interactive calibration sequence.

        Blocks until calibration is complete (user presses Button B twice).
        """
        # Step 1: Show instructions.
        self._display.clear()
        self._display.show_text(0, 6, "Calibrate Wheels", inverted=True)
        self._display.show_text(0, 20, "Release Pitch Wh")
        self._display.show_text(0, 30, "and return MODWh")
        self._display.show_text(0, 40, "to zero position")
        self._display.show_text(0, 50, "and press button")

        # Step 2: Wait for Button B press.
        self._wait_for_press()

        # Step 3: Read raw analog values (scaled to 10-bit for compatibility).
        raw_a = self._wheel_a._adc.value >> _ADC_SHIFT
        raw_b = self._wheel_b._adc.value >> _ADC_SHIFT

        # Wheel A (pitch bend): center is the current reading,
        # delta is center - 128 (the offset from ADC zero to pot zero).
        wheel_a_center = raw_a
        wheel_a_delta = raw_a - 128

        # Wheel B (mod wheel): delta is the current reading (zero position),
        # center is delta + 128 (used if wheel B is ever set to pitch bend mode).
        wheel_b_delta = raw_b
        wheel_b_center = raw_b + 128

        # Step 4: Save to config.
        self._config.set("wheel_a.calibration.center", wheel_a_center)
        self._config.set("wheel_a.calibration.delta", wheel_a_delta)
        self._config.set("wheel_b.calibration.center", wheel_b_center)
        self._config.set("wheel_b.calibration.delta", wheel_b_delta)

        # Update the wheel objects with the new calibration values.
        self._wheel_a.calibrate(center=wheel_a_center, delta=wheel_a_delta)
        self._wheel_b.calibrate(center=wheel_b_center, delta=wheel_b_delta)

        # Step 5: Show confirmation with the measured values.
        self._display.clear()
        self._display.show_text(0, 6, "Calibration Done", inverted=True)
        self._display.show_text(0, 24, f"WA ctr: {wheel_a_center:4d}")
        self._display.show_text(0, 36, f"WA dlt: {wheel_a_delta:4d}")
        self._display.show_text(0, 48, f"WB dlt: {wheel_b_delta:4d}")
        self._display.show_text(48, 56, "OK", scale=2)

        # Step 6: Wait for confirmation press, then return.
        self._wait_for_press()

    def _wait_for_press(self):
        """Block until Button B is pressed (fell edge detected)."""
        # First, wait for button to be released (in case it's still held).
        while True:
            self._button.update()
            if not self._button.pressed:
                break
            time.sleep(0.01)  # Yield CPU to avoid 100% spin.

        # Then wait for a new press.
        while True:
            self._button.update()
            if self._button.fell:
                return
            time.sleep(0.01)
