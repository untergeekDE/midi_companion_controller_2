# display.py — SSD1306 128x64 OLED display abstraction.
#
# Provides a high-level interface for the three display screens:
# 1. Workscreen: shows MIDI channel and current wheel values (normal operation)
# 2. Menu screen: shows configuration options (via menu.py)
# 3. Calibration screen: shows calibration instructions (via calibration.py)
#
# Uses displayio + adafruit_displayio_ssd1306 for hardware driving,
# and adafruit_display_text for text rendering.
#
# The display is rotated 180 degrees to match the physical mounting
# of the original Teensy build.
#
# Usage:
#     import busio
#     from hardware import pins
#     from hardware.display import OledDisplay
#     i2c = busio.I2C(pins.I2C_SCL, pins.I2C_SDA)
#     disp = OledDisplay(i2c)
#     disp.show_workscreen(channel=1)
#     disp.update_values(wheel_a=0, wheel_b=64)

import displayio
import i2cdisplaybus
import terminalio
from adafruit_displayio_ssd1306 import SSD1306
from adafruit_display_text import label

# Display dimensions.
_WIDTH = 128
_HEIGHT = 64

# Built-in font — monospaced, good enough for a MIDI controller display.
_FONT = terminalio.FONT


class OledDisplay:
    """High-level interface to the SSD1306 128x64 OLED display.

    Args:
        i2c: A busio.I2C instance connected to the display.
        address: I2C address of the SSD1306 (default 0x3C).
        rotation: Display rotation in degrees (default 180 to match
            original hardware mounting).
    """

    def __init__(self, i2c, address=0x3C, rotation=0):
        # Note: displayio.release_displays() is called in code.py before I2C init
        # to avoid "pin in use" errors on soft reboot.
        display_bus = i2cdisplaybus.I2CDisplayBus(i2c, device_address=address)
        self._display = SSD1306(display_bus, width=_WIDTH, height=_HEIGHT,
                                rotation=rotation)

        # The root display group — everything drawn goes here.
        self._root = displayio.Group()
        self._display.root_group = self._root

        # Labels for the workscreen (created lazily in show_workscreen).
        self._ch_label = None
        self._wheel_a_label = None
        self._wheel_b_label = None

    def clear(self):
        """Clear the display by removing all elements."""
        while len(self._root) > 0:
            self._root.pop()

    def show_workscreen(self, channel=1):
        """Display the main operating screen: MIDI channel and wheel value placeholders.

        Args:
            channel: The current MIDI transmit channel (1-16).
        """
        self.clear()

        # Large channel display at the top.
        ch_text = f"Ch {channel:2d}"
        self._ch_label = label.Label(_FONT, text=ch_text, scale=2,
                                     x=0, y=10, color=0xFFFFFF)
        self._root.append(self._ch_label)

        # Wheel A label and value.
        lbl_a = label.Label(_FONT, text="A", scale=1, x=0, y=30, color=0xFFFFFF)
        self._root.append(lbl_a)
        self._wheel_a_label = label.Label(_FONT, text="    0", scale=2,
                                          x=24, y=30, color=0xFFFFFF)
        self._root.append(self._wheel_a_label)

        # Wheel B label and value.
        lbl_b = label.Label(_FONT, text="B", scale=1, x=0, y=50, color=0xFFFFFF)
        self._root.append(lbl_b)
        self._wheel_b_label = label.Label(_FONT, text="    0", scale=2,
                                          x=24, y=50, color=0xFFFFFF)
        self._root.append(self._wheel_b_label)

    @staticmethod
    def _format_wheel(value, mode):
        """Format a wheel value for display."""
        if not isinstance(value, int):
            return str(value)
        if mode == "pitch_bend":
            # Convert unsigned 0-16383 to signed -8192..8191 for display.
            signed = value - 8192
            return f"{signed:5d}"
        # CC mode: show decimal 0-127.
        return f"{value:5d}"

    def update_values(self, wheel_a=None, wheel_b=None, mode_a="cc", mode_b="cc"):
        """Update the wheel value displays on the workscreen.

        Only redraws labels that have changed. Call this every loop iteration.

        Args:
            wheel_a: Current wheel A value (int), or None to skip update.
            wheel_b: Current wheel B value (int), or None to skip update.
            mode_a: Wheel A mode ("cc" or "pitch_bend").
            mode_b: Wheel B mode ("cc" or "pitch_bend").
        """
        if wheel_a is not None and self._wheel_a_label is not None:
            self._wheel_a_label.text = self._format_wheel(wheel_a, mode_a)
        if wheel_b is not None and self._wheel_b_label is not None:
            self._wheel_b_label.text = self._format_wheel(wheel_b, mode_b)

    def update_channel(self, channel):
        """Update the MIDI channel display on the workscreen."""
        if self._ch_label is not None:
            self._ch_label.text = f"Ch {channel:2d}"

    def show_text(self, x, y, text, scale=1, inverted=False):
        """Draw a text label at the given pixel position.

        Used by menu.py and calibration.py for arbitrary text placement.

        Args:
            x: X pixel coordinate.
            y: Y pixel coordinate.
            text: The string to display.
            scale: Font scale multiplier (1, 2, or 3).
            inverted: If True, draw black text on white background.

        Returns:
            The label object, in case the caller needs to remove it later.
        """
        color = 0x000000 if inverted else 0xFFFFFF
        bg_color = 0xFFFFFF if inverted else 0x000000
        lbl = label.Label(_FONT, text=text, scale=scale,
                          x=x, y=y, color=color, background_color=bg_color)
        self._root.append(lbl)
        return lbl

    def remove_label(self, lbl):
        """Remove a previously added label from the display.

        Args:
            lbl: A label object returned by show_text().
        """
        try:
            self._root.remove(lbl)
        except ValueError:
            pass  # Already removed.
