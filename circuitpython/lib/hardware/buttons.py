# buttons.py — Debounced button input with double-click detection.
#
# Wraps adafruit_debouncer to provide clean button state with edge detection
# (fell/rose) and double-click detection via timestamp comparison.
#
# All buttons use INPUT_PULLUP, so pressed = LOW = False.
# "fell" means the signal fell from HIGH to LOW = button was just pressed.
# "rose" means the signal rose from LOW to HIGH = button was just released.
#
# Usage:
#     from hardware.buttons import DebouncedButton
#     from hardware import pins
#     btn = DebouncedButton(pins.BUTTON_A, double_click_ms=250)
#     while True:
#         btn.update()
#         if btn.fell:
#             print("pressed")
#         if btn.double_click:
#             print("double-click!")

import digitalio
import time
from adafruit_debouncer import Debouncer


class DebouncedButton:
    """A debounced button with edge detection and double-click support.

    Args:
        pin: A board pin object. Configured as INPUT_PULLUP internally.
        debounce_s: Debounce interval in seconds (default 0.01 = 10ms).
        double_click_ms: Time window in milliseconds for double-click
            detection (default 250). Set to 0 to disable double-click.
    """

    def __init__(self, pin, debounce_s=0.01, double_click_ms=250):
        # Set up the physical pin with pull-up resistor.
        self._pin = digitalio.DigitalInOut(pin)
        self._pin.direction = digitalio.Direction.INPUT
        self._pin.pull = digitalio.Pull.UP

        # Wrap in Debouncer for clean state transitions.
        self._debouncer = Debouncer(self._pin, interval=debounce_s)

        # Double-click detection state.
        self._double_click_ms = double_click_ms
        self._last_fell_time = 0  # monotonic_ns of last fell event
        self._double_click = False

    def update(self):
        """Sample the button and update state. Call once per main loop iteration.

        After calling update(), check .fell, .rose, and .double_click
        for the current cycle's events.
        """
        self._double_click = False
        self._debouncer.update()

        # Double-click detection: if we just got a fell event and the
        # previous fell was within the double-click window, flag it.
        if self._debouncer.fell and self._double_click_ms > 0:
            now = time.monotonic_ns()
            elapsed_ms = (now - self._last_fell_time) // 1_000_000
            if self._last_fell_time > 0 and elapsed_ms < self._double_click_ms:
                self._double_click = True
                self._last_fell_time = 0  # Reset so triple-click doesn't re-trigger.
            else:
                self._last_fell_time = now

    @property
    def fell(self):
        """True if the button was just pressed this cycle (HIGH->LOW transition)."""
        return self._debouncer.fell

    @property
    def rose(self):
        """True if the button was just released this cycle (LOW->HIGH transition)."""
        return self._debouncer.rose

    @property
    def double_click(self):
        """True if a double-click was detected this cycle."""
        return self._double_click

    @property
    def pressed(self):
        """True if the button is currently held down (steady state)."""
        return not self._debouncer.value  # Active low: pressed = False = True here.
