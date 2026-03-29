# code.py — Main entry point for the MIDI Companion Controller 2.
#
# This file contains ONLY wiring (hardware initialization) and the main
# loop. All business logic lives in the library modules under lib/.
#
# The main loop runs synchronously:
# 1. Update button states
# 2. Check for double-click → enter configuration menu
# 3. Process MIDI inputs (USB Host + DIN In) via MidiMerge
# 4. Read wheels, inject changed values into the merge
# 5. Read buttons, inject CC/Aftertouch events into the merge
# 6. Flush merged messages to all outputs (DIN + USB Device + USB Host)
# 7. Update the OLED display
# 8. Blink LEDs for activity indication

import busio

from hardware import pins
from hardware.leds import StatusLED
from hardware.buttons import DebouncedButton
from hardware.wheels import AnalogWheel
from hardware.display import OledDisplay
from hardware.midi_uart import UartMidi
from hardware.midi_usb_device import UsbDeviceMidi
from midi_merge import MidiMerge

# USB Host (MAX3421E) is optional — controller works without it as a
# pure USB MIDI device connected to a laptop. If the MAX3421E libraries
# aren't installed or the chip isn't present, we skip it silently.
try:
    from hardware.midi_usb_host import UsbHostMidi
    _HAS_USB_HOST = True
except ImportError:
    _HAS_USB_HOST = False
from config import ConfigManager
from menu import OnDeviceMenu
from adafruit_midi.control_change import ControlChange
from adafruit_midi.pitch_bend import PitchBend
from adafruit_midi.channel_pressure import ChannelPressure
from adafruit_midi.note_on import NoteOn
from adafruit_midi.note_off import NoteOff

__version__ = "2.0.0"

# ──────────────────────────────────────────────
# 1. CONFIGURATION
# ──────────────────────────────────────────────
import time

config = ConfigManager("config.json")
_load_status = config.load()

# ──────────────────────────────────────────────
# 2. HARDWARE INITIALIZATION
# ──────────────────────────────────────────────

# Release any displays held from a previous soft-reboot before claiming I2C pins.
import displayio
displayio.release_displays()

# Communication buses.
i2c = busio.I2C(pins.I2C_SCL, pins.I2C_SDA)

# Display.
display = OledDisplay(i2c)

# Show warning if config was corrupt, persist defaults on first boot.
if _load_status == "corrupt":
    display.show_text(0, 20, "Config corrupt!", inverted=True)
    display.show_text(0, 36, "Using defaults")
    time.sleep(2)
    display.clear()
elif _load_status == "defaults":
    config.save()

# LEDs.
led_in = StatusLED(pins.LED1)
led_out = StatusLED(pins.LED2)

# Buttons.
button_a = DebouncedButton(pins.BUTTON_A, double_click_ms=400)
button_b = DebouncedButton(pins.BUTTON_B, double_click_ms=0)  # No double-click needed.
footswitch = DebouncedButton(pins.FOOTSWITCH, double_click_ms=0)

# Wheels — mode and calibration loaded from config.
deadband = config.get("deadband")

wheel_a = AnalogWheel(pins.WHEEL_A,
                      mode=config.get("wheel_a.mode"),
                      deadband=deadband)
wheel_a.calibrate(center=config.get("wheel_a.calibration.center"),
                  delta=config.get("wheel_a.calibration.delta"))

wheel_b = AnalogWheel(pins.WHEEL_B,
                      mode=config.get("wheel_b.mode"),
                      deadband=deadband)
wheel_b.calibrate(center=config.get("wheel_b.calibration.center"),
                  delta=config.get("wheel_b.calibration.delta"))

# MIDI interfaces.
uart_midi = UartMidi(pins.UART_TX, pins.UART_RX,
                     channel=config.get("midi.tx_channel") - 1)
usb_device_midi = UsbDeviceMidi(channel=config.get("midi.tx_channel") - 1)

# USB Host (MAX3421E) — only if hardware + libraries are present.
merge_inputs = {"usb_device": usb_device_midi, "din": uart_midi}
merge_outputs = {"din": uart_midi, "usb_device": usb_device_midi}

if _HAS_USB_HOST:
    spi = busio.SPI(pins.SPI_CLK, MOSI=pins.SPI_MOSI, MISO=pins.SPI_MISO)
    usb_host_midi = UsbHostMidi(spi, pins.USB_HOST_CS)
    merge_inputs["usb_host"] = usb_host_midi
    merge_outputs["usb_host"] = usb_host_midi

# MIDI merge engine with echo prevention.
merge = MidiMerge(
    inputs=merge_inputs,
    outputs=merge_outputs,
    thru_enabled=config.get("midi.thru_enabled")
)

# On-device menu.
menu = OnDeviceMenu(display, config, wheel_a, wheel_b, button_a, button_b)

# ──────────────────────────────────────────────
# 3. SHOW STARTUP SCREEN
# ──────────────────────────────────────────────
display.show_workscreen(channel=config.get("midi.tx_channel"))
display.show_text(80, 10, f"v{__version__}", scale=1)

# Watchdog — auto-resets if main loop hangs for 8 seconds.
import microcontroller
import watchdog
microcontroller.watchdog.timeout = 8.0
microcontroller.watchdog.mode = watchdog.WatchDogMode.RESET

# ──────────────────────────────────────────────
# 4. MAIN LOOP
# ──────────────────────────────────────────────
while True:
    microcontroller.watchdog.feed()

    # --- Update button states ---
    button_a.update()
    button_b.update()
    footswitch.update()

    # --- Double-click Button A → enter menu ---
    if button_a.double_click:
        menu.enter()
        # After menu exit, reload settings that may have changed.
        tx_ch = config.get("midi.tx_channel")
        merge.thru_enabled = config.get("midi.thru_enabled")
        wheel_a.mode = config.get("wheel_a.mode")
        wheel_a.deadband = config.get("deadband")
        wheel_b.mode = config.get("wheel_b.mode")
        wheel_b.deadband = config.get("deadband")
        uart_midi.set_channel(tx_ch)
        usb_device_midi.set_channel(tx_ch)
        display.show_workscreen(channel=tx_ch)
        microcontroller.watchdog.feed()
        continue

    # --- Process MIDI inputs (USB Host + DIN) ---
    merge.process()
    if merge.had_input:
        led_in.on()

    # --- Read wheels and inject changes ---
    tx_ch = config.get("midi.tx_channel")
    output_generated = False

    # Wheel A.
    wheel_a.read()
    if wheel_a.changed:
        wa_mode = config.get("wheel_a.mode")
        if wa_mode != "off":
            if wa_mode == "pitch_bend":
                msg = PitchBend(wheel_a.value)
            else:  # "cc"
                cc_num = config.get("wheel_a.cc_number")
                msg = ControlChange(cc_num, wheel_a.value) if cc_num is not None else None
            if msg is not None:
                msg.channel = tx_ch - 1
                merge.inject(msg)
                output_generated = True

    # Wheel B.
    wheel_b.read()
    if wheel_b.changed:
        wb_mode = config.get("wheel_b.mode")
        if wb_mode != "off":
            if wb_mode == "pitch_bend":
                msg = PitchBend(wheel_b.value)
            else:  # "cc"
                cc_num = config.get("wheel_b.cc_number")
                msg = ControlChange(cc_num, wheel_b.value) if cc_num is not None else None
            if msg is not None:
                msg.channel = tx_ch - 1
                merge.inject(msg)
                output_generated = True

    # --- Read buttons and inject events ---
    # Global aftertouch value (stored under button_b for historical reasons).
    ato_value = config.get("button_b.aftertouch_value")

    # Button A.
    if button_a.fell or button_a.rose:
        ba_mode = config.get("button_a.mode")
        msg = None
        if ba_mode == "off":
            pass
        elif ba_mode == "aftertouch":
            msg = ChannelPressure(ato_value if button_a.fell else 0)
        elif ba_mode == "note":
            cc_num = config.get("button_a.cc_number")
            if cc_num is not None:
                msg = NoteOn(cc_num, 127) if button_a.fell else NoteOff(cc_num, 0)
        else:  # "cc"
            cc_num = config.get("button_a.cc_number")
            if cc_num is not None:
                msg = ControlChange(cc_num, 127 if button_a.fell else 0)
        if msg is not None:
            msg.channel = tx_ch - 1
            merge.inject(msg)
            output_generated = True

    # Button B.
    if button_b.fell or button_b.rose:
        bb_mode = config.get("button_b.mode")
        msg = None
        if bb_mode == "off":
            pass
        elif bb_mode == "aftertouch":
            msg = ChannelPressure(ato_value if button_b.fell else 0)
        elif bb_mode == "note":
            cc_num = config.get("button_b.cc_number")
            if cc_num is not None:
                msg = NoteOn(cc_num, 127) if button_b.fell else NoteOff(cc_num, 0)
        else:  # "cc"
            cc_num = config.get("button_b.cc_number")
            if cc_num is not None:
                msg = ControlChange(cc_num, 127 if button_b.fell else 0)
        if msg is not None:
            msg.channel = tx_ch - 1
            merge.inject(msg)
            output_generated = True

    # Footswitch.
    if footswitch.fell or footswitch.rose:
        fs_mode = config.get("footswitch.mode")
        msg = None
        if fs_mode == "off":
            pass
        elif fs_mode == "aftertouch":
            msg = ChannelPressure(ato_value if footswitch.fell else 0)
        elif fs_mode == "note":
            cc_num = config.get("footswitch.cc_number")
            if cc_num is not None:
                msg = NoteOn(cc_num, 127) if footswitch.fell else NoteOff(cc_num, 0)
        else:  # "cc"
            cc_num = config.get("footswitch.cc_number")
            if cc_num is not None:
                msg = ControlChange(cc_num, 127 if footswitch.fell else 0)
        if msg is not None:
            msg.channel = tx_ch - 1
            merge.inject(msg)
            output_generated = True

    # --- Flush all queued messages to all outputs ---
    has_any_output = output_generated or merge.has_output
    merge.flush()
    if has_any_output:
        led_out.on()

    # --- Update display (only on value changes) ---
    if wheel_a.changed or wheel_b.changed:
        display.update_values(
            wheel_a=wheel_a.value if wheel_a.changed else None,
            wheel_b=wheel_b.value if wheel_b.changed else None,
            mode_a=config.get("wheel_a.mode"),
            mode_b=config.get("wheel_b.mode")
        )

    # --- LEDs off at end of cycle ---
    led_in.off()
    led_out.off()
