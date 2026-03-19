# menu.py — On-device configuration menu.
#
# Provides an interactive menu displayed on the OLED, navigated with
# buttons and the mod wheel. Allows the user to configure:
# - CC assignments for wheels and buttons (CC 0-127, or PWH/ATO special modes)
# - MIDI transmit channel (1-16)
# - MIDI Thru on/off
# - Aftertouch default value (0-127)
# - Deadband width (0-63)
# - Wheel calibration (launches calibration.py)
#
# The menu blocks the main loop — MIDI processing stops while active.
# This matches the original Teensy behavior and is intentional (menu
# interaction is brief and infrequent).
#
# Navigation:
# - Button A: step to next option
# - Button B: confirm/toggle value
# - Wheel B: scroll value
# - Select "X" (top right) to exit
#
# Triggered by double-clicking Button A in the main loop.
#
# Usage:
#     menu = OnDeviceMenu(display, config, wheel_a, wheel_b, button_a, button_b)
#     menu.enter()  # Blocks until user exits menu

import time
from calibration import WheelCalibration


# Menu item definitions.
# Each tuple: (display_label, config_dotpath, item_type, value_range)
# item_type: "wheel_mode" (0-127 + PWH), "button_mode" (0-127 + ATO),
#            "channel" (1-16), "bool" (ON/OFF), "value" (numeric),
#            "action" (triggers a function)
_MENU_ITEMS = [
    ("Wh A", "wheel_a", "wheel_mode", None),
    ("Wh B", "wheel_b", "wheel_mode", None),
    ("BtnA", "button_a", "button_mode", None),
    ("BtnB", "button_b", "button_mode", None),
    ("FtSw", "footswitch", "button_mode", None),
    ("TxCh", "midi.tx_channel", "channel", (1, 16)),
    ("Thru", "midi.thru_enabled", "bool", None),
    ("ATO ", "button_b.aftertouch_value", "value", (0, 127)),
    ("DB  ", "deadband", "value", (0, 63)),
    ("CAL ", None, "action", None),
]


class OnDeviceMenu:
    """On-device configuration menu displayed on the OLED.

    Args:
        display: An OledDisplay instance.
        config: A ConfigManager instance.
        wheel_a: AnalogWheel for wheel A (needed by calibration).
        wheel_b: The AnalogWheel for wheel B (used to scroll values).
        button_a: DebouncedButton for stepping through options.
        button_b: DebouncedButton for confirming/toggling.
    """

    def __init__(self, display, config, wheel_a, wheel_b, button_a, button_b):
        self._display = display
        self._config = config
        self._wheel_a = wheel_a
        self._wheel_b = wheel_b
        self._btn_a = button_a
        self._btn_b = button_b
        self._selected = 0  # Currently highlighted menu item index.

    def enter(self):
        """Enter the menu. Blocks until the user exits.

        On exit, saves the configuration to disk.
        """
        self._draw_menu()
        wheel_moved = False
        wheel_old = self._wheel_b.read()

        while True:
            self._btn_a.update()
            self._btn_b.update()

            # Check if wheel has moved significantly.
            wheel_val = self._wheel_b.read()
            if not wheel_moved:
                diff = abs(wheel_val - wheel_old)
                wheel_moved = diff > 5

            # Handle Button A: step to next item.
            if self._btn_a.fell:
                self._selected = (self._selected + 1) % (len(_MENU_ITEMS) + 1)
                wheel_moved = False
                wheel_old = wheel_val
                self._draw_menu()

            # Handle Button B: confirm/toggle current item.
            if self._btn_b.fell:
                # Exit item (index == len means the "X" close button).
                if self._selected == len(_MENU_ITEMS):
                    break

                item = _MENU_ITEMS[self._selected]
                self._handle_confirm(item)
                self._draw_menu()

            # Handle wheel scrolling for current item.
            if wheel_moved and self._selected < len(_MENU_ITEMS):
                item = _MENU_ITEMS[self._selected]
                self._handle_wheel(item, wheel_val)
                self._draw_menu()

            time.sleep(0.01)  # Yield CPU to avoid 100% spin.

        # Save all changes on menu exit.
        self._config.save()

    def _draw_menu(self):
        """Redraw the entire menu screen."""
        self._display.clear()

        # Header.
        self._display.show_text(0, 6, "LCC2 Config", inverted=True)

        # Menu items in two columns (5 items each) to fit all 10 items.
        # Column 1: items 0-4 on the left, Column 2: items 5-9 on the right.
        # Matches the original Teensy two-column layout.
        for i, item in enumerate(_MENU_ITEMS):
            label_text, dotpath, item_type, _ = item
            inverted = (i == self._selected)

            col = i // 5           # 0 = left column, 1 = right column
            row = i % 5            # 0-4 within each column
            x = col * 64           # Left column at x=0, right at x=64
            y = 18 + row * 10      # 5 rows starting at y=18

            val_text = self._format_value(item)
            line = f"{label_text}:{val_text}"
            self._display.show_text(x, y, line, inverted=inverted)

        # "X" close button at top right.
        self._display.show_text(116, 6, "X",
                                inverted=(self._selected == len(_MENU_ITEMS)))

    def _format_value(self, item):
        """Format a menu item's current value for display."""
        _, dotpath, item_type, _ = item

        if item_type == "action":
            return "GO"

        if item_type == "wheel_mode":
            mode = self._config.get(f"{dotpath}.mode")
            if mode == "pitch_bend":
                return "PWH"
            cc = self._config.get(f"{dotpath}.cc_number")
            return f"{cc:3d}" if cc is not None else "---"

        if item_type == "button_mode":
            mode = self._config.get(f"{dotpath}.mode")
            if mode == "aftertouch":
                return "ATO"
            cc = self._config.get(f"{dotpath}.cc_number")
            return f"{cc:3d}" if cc is not None else "---"

        if item_type == "channel":
            return f"{self._config.get(dotpath):3d}"

        if item_type == "bool":
            return " ON" if self._config.get(dotpath) else "OFF"

        if item_type == "value":
            return f"{self._config.get(dotpath):3d}"

        return "???"

    def _handle_confirm(self, item):
        """Handle Button B press on the currently selected item."""
        _, dotpath, item_type, val_range = item

        if item_type == "action":
            # Launch calibration.
            cal = WheelCalibration(self._display, self._wheel_a, self._wheel_b,
                                   self._btn_b, self._config)
            cal.run()
            return

        if item_type == "wheel_mode":
            # Toggle: cc -> pitch_bend -> cc, cycling CC number on each press.
            mode = self._config.get(f"{dotpath}.mode")
            if mode == "pitch_bend":
                self._config.set(f"{dotpath}.mode", "cc")
                if self._config.get(f"{dotpath}.cc_number") is None:
                    self._config.set(f"{dotpath}.cc_number", 1)
            else:
                cc = self._config.get(f"{dotpath}.cc_number")
                if cc is not None and cc < 127:
                    self._config.set(f"{dotpath}.cc_number", cc + 1)
                else:
                    self._config.set(f"{dotpath}.mode", "pitch_bend")
                    self._config.set(f"{dotpath}.cc_number", None)
            return

        if item_type == "button_mode":
            # Toggle: cc -> aftertouch -> cc, cycling CC number on each press.
            mode = self._config.get(f"{dotpath}.mode")
            if mode == "aftertouch":
                self._config.set(f"{dotpath}.mode", "cc")
                if self._config.get(f"{dotpath}.cc_number") is None:
                    self._config.set(f"{dotpath}.cc_number", 64)
            else:
                cc = self._config.get(f"{dotpath}.cc_number")
                if cc is not None and cc < 127:
                    self._config.set(f"{dotpath}.cc_number", cc + 1)
                else:
                    self._config.set(f"{dotpath}.mode", "aftertouch")
            return

        if item_type == "channel":
            val = self._config.get(dotpath)
            val = val + 1 if val < 16 else 1
            self._config.set(dotpath, val)
            return

        if item_type == "bool":
            self._config.set(dotpath, not self._config.get(dotpath))
            return

        if item_type == "value":
            lo, hi = val_range
            val = self._config.get(dotpath)
            val = val + 1 if val < hi else lo
            self._config.set(dotpath, val)
            return

    def _handle_wheel(self, item, wheel_val):
        """Handle wheel B scrolling for the currently selected item."""
        _, dotpath, item_type, val_range = item

        if item_type == "action":
            return  # No wheel control for calibration.

        if item_type == "wheel_mode":
            # Map wheel 0-127 to CC 0-127 or pitch_bend at max.
            if wheel_val >= 127:
                self._config.set(f"{dotpath}.mode", "pitch_bend")
                self._config.set(f"{dotpath}.cc_number", None)
            else:
                self._config.set(f"{dotpath}.mode", "cc")
                self._config.set(f"{dotpath}.cc_number", wheel_val)
            return

        if item_type == "button_mode":
            if wheel_val >= 127:
                self._config.set(f"{dotpath}.mode", "aftertouch")
            else:
                self._config.set(f"{dotpath}.mode", "cc")
                self._config.set(f"{dotpath}.cc_number", wheel_val)
            return

        if item_type == "channel":
            # Map wheel 0-127 to 1-16.
            self._config.set(dotpath, (wheel_val * 15 // 127) + 1)
            return

        if item_type == "bool":
            self._config.set(dotpath, wheel_val > 63)
            return

        if item_type == "value":
            lo, hi = val_range
            # Map wheel 0-127 to lo..hi range.
            mapped = lo + (wheel_val * (hi - lo) // 127)
            self._config.set(dotpath, mapped)
            return
