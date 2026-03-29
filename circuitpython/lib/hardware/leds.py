# leds.py — Simple LED control for MIDI activity indicators.
#
# LED1 signals MIDI input activity (blinks when data arrives).
# LED2 signals MIDI output activity (blinks when controller sends data).
#
# Usage:
#     from hardware.leds import StatusLED
#     from hardware import pins
#     led = StatusLED(pins.LED1)
#     led.on()
#     led.off()

import digitalio
import time


class StatusLED:
    """Controls a single status LED.

    The LED is driven as a simple digital output. Turn it on at the start
    of an event (MIDI in/out) and off at the end of the main loop cycle
    to create a brief activity blink.

    A minimum on-time ensures the blink is visible even when the main loop
    runs faster than the human eye can perceive.

    Args:
        pin: A board pin object (e.g., board.GP2).
    """

    _MIN_ON_NS = 50_000_000  # 50 ms — minimum visible blink duration

    def __init__(self, pin):
        self._led = digitalio.DigitalInOut(pin)
        self._led.direction = digitalio.Direction.OUTPUT
        self._led.value = False
        self._on_time = 0

    def on(self):
        """Turn the LED on."""
        self._led.value = True
        self._on_time = time.monotonic_ns()

    def off(self):
        """Turn the LED off (respects minimum on-time for visibility)."""
        if self._led.value and (time.monotonic_ns() - self._on_time) < self._MIN_ON_NS:
            return  # Keep on — minimum blink time not reached yet
        self._led.value = False
