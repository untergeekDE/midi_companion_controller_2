# wheels.py — Analog wheel input with calibration and deadband.
#
# Reads a potentiometer/wheel via an analog input pin and converts the raw
# ADC value into a MIDI-ready value. Supports two modes:
#
# - "pitch_bend" (centered): Returns -8192..8191 (14-bit signed).
#   Uses the 'center' calibration value as zero reference.
#   Deadband is applied symmetrically around center.
#
# - "cc" (uncentered): Returns 0..127 (7-bit unsigned).
#   Uses the 'delta' calibration value as zero/low reference.
#   Deadband is applied above the zero position.
#
# The potentiometers in the wheel box have roughly a quarter of the full
# ADC range of travel, so we assume ~256 raw counts of usable range
# (out of the CircuitPython 16-bit ADC range of 0..65535).
#
# Usage:
#     from hardware.wheels import AnalogWheel
#     from hardware import pins
#     wheel = AnalogWheel(pins.WHEEL_A, mode="pitch_bend", deadband=10)
#     wheel.calibrate(center=658, delta=517)
#     value = wheel.read()

import analogio


# CircuitPython ADC returns 16-bit values (0..65535). The original Teensy
# code used 10-bit (0..1023). We scale down to 10-bit equivalent for
# compatibility with the original calibration values and algorithm.
_ADC_SHIFT = 6  # Right-shift 16-bit to 10-bit: 65535 >> 6 ~ 1023


class AnalogWheel:
    """Reads an analog wheel/potentiometer with calibration and deadband.

    Args:
        pin: A board analog pin object (e.g., board.A0).
        mode: "pitch_bend" for centered wheel, "cc" for uncentered.
        deadband: Width of the dead zone around center/zero. Shared global
            value from config. Range 0..63.
    """

    def __init__(self, pin, mode="pitch_bend", deadband=10):
        self._adc = analogio.AnalogIn(pin)
        self.mode = mode
        self.deadband = deadband

        # Calibration values (raw 10-bit scale, matching original firmware).
        # Set via calibrate() or loaded from config.
        self._center = 512  # Mid-point for pitch bend mode.
        self._delta = 0     # Zero offset for CC mode.

        # State tracking for change detection.
        self._value = 0
        self._changed = False

    def calibrate(self, center=512, delta=0):
        """Set calibration values.

        Args:
            center: The raw ADC reading (10-bit scale) when the pitch bend
                wheel is at rest in the center position.
            delta: The raw ADC reading (10-bit scale) when the CC wheel
                is at its lowest/zero position.
        """
        self._center = center
        self._delta = delta

    def read(self):
        """Read the wheel and return a MIDI-ready value.

        Returns:
            int: -8192..8191 for pitch_bend mode, 0..127 for cc mode.
            Also updates .value and .changed properties.
        """
        # Read 16-bit ADC and scale down to 10-bit for algorithm compatibility.
        raw = self._adc.value >> _ADC_SHIFT

        if self.mode == "pitch_bend":
            new_value = self._read_centered(raw)
        else:
            new_value = self._read_uncentered(raw)

        self._changed = (new_value != self._value)
        self._value = new_value
        return new_value

    def _read_centered(self, raw):
        """Process a centered wheel (pitch bend).

        Subtracts the center calibration, applies deadband, clamps to
        -128..127 range, then scales to 14-bit (-8192..8191) for
        adafruit_midi PitchBend compatibility.

        This matches the original readWheel() with centered=true,
        plus the *64 scaling from the original main loop.
        """
        v = raw - self._center

        if v > 0:
            if v < self.deadband:
                v = 0
            else:
                v -= self.deadband
            if v > 127:
                v = 127
        elif v < 0:
            if v > -self.deadband:
                v = 0
            else:
                v += self.deadband
            if v < -128:
                v = -128

        # Scale from 8-bit (-128..127) to 14-bit (-8192..8191).
        return v * 64

    def _read_uncentered(self, raw):
        """Process an uncentered wheel (CC / mod wheel).

        Subtracts the delta calibration, applies deadband, clamps to
        0..255 range, then divides by 2 to get 0..127 for MIDI CC.

        This matches the original readWheel() with centered=false.
        """
        v = raw - self._delta

        if v < self.deadband:
            v = 0
        else:
            v -= self.deadband
        if v > 255:
            v = 255

        # Divide by 2 to get 7-bit range (0..127).
        return v >> 1

    @property
    def value(self):
        """The most recently read value."""
        return self._value

    @property
    def changed(self):
        """True if the value changed on the last read() call."""
        return self._changed
