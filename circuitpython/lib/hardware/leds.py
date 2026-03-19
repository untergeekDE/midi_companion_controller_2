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


class StatusLED:
    """Controls a single status LED.

    The LED is driven as a simple digital output. Turn it on at the start
    of an event (MIDI in/out) and off at the end of the main loop cycle
    to create a brief activity blink.

    Args:
        pin: A board pin object (e.g., board.GP2).
    """

    def __init__(self, pin):
        self._led = digitalio.DigitalInOut(pin)
        self._led.direction = digitalio.Direction.OUTPUT
        self._led.value = False

    def on(self):
        """Turn the LED on."""
        self._led.value = True

    def off(self):
        """Turn the LED off."""
        self._led.value = False
