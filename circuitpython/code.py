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
from hardware.midi_usb_host import UsbHostMidi
from hardware.midi_usb_device import UsbDeviceMidi
from midi_merge import MidiMerge
from config import ConfigManager
from menu import OnDeviceMenu
from adafruit_midi.control_change import ControlChange
from adafruit_midi.pitch_bend import PitchBend
from adafruit_midi.channel_pressure import ChannelPressure

# ──────────────────────────────────────────────
# 1. CONFIGURATION
# ──────────────────────────────────────────────
config = ConfigManager("config.json")
config.load()

# ──────────────────────────────────────────────
# 2. HARDWARE INITIALIZATION
# ──────────────────────────────────────────────

# Communication buses.
i2c = busio.I2C(pins.I2C_SCL, pins.I2C_SDA)
spi = busio.SPI(pins.SPI_CLK, MOSI=pins.SPI_MOSI, MISO=pins.SPI_MISO)

# Display.
display = OledDisplay(i2c)

# LEDs.
led_in = StatusLED(pins.LED1)
led_out = StatusLED(pins.LED2)

# Buttons.
button_a = DebouncedButton(pins.BUTTON_A, double_click_ms=250)
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
usb_host_midi = UsbHostMidi(spi, pins.USB_HOST_CS)
usb_device_midi = UsbDeviceMidi(channel=config.get("midi.tx_channel") - 1)

# MIDI merge engine.
# Inputs: USB Host (Launchpad), USB Device (laptop/DAW), DIN In
# Outputs: DIN Out (Roland SE-02), USB Device (laptop/DAW), USB Host (Launchpad)
# Echo prevention: matching names (e.g., "usb_host") prevent a source's
# messages from being sent back to itself.
merge = MidiMerge(
    inputs={"usb_host": usb_host_midi, "usb_device": usb_device_midi, "din": uart_midi},
    outputs={"din": uart_midi, "usb_device": usb_device_midi, "usb_host": usb_host_midi},
    thru_enabled=config.get("midi.thru_enabled")
)

# On-device menu.
menu = OnDeviceMenu(display, config, wheel_a, wheel_b, button_a, button_b)

# ──────────────────────────────────────────────
# 3. SHOW STARTUP SCREEN
# ──────────────────────────────────────────────
display.show_workscreen(channel=config.get("midi.tx_channel"))

# ──────────────────────────────────────────────
# 4. MAIN LOOP
# ──────────────────────────────────────────────
while True:

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
        if config.get("wheel_a.mode") == "pitch_bend":
            msg = PitchBend(wheel_a.value)
            msg.channel = tx_ch - 1
            merge.inject(msg)
        else:
            cc_num = config.get("wheel_a.cc_number")
            msg = ControlChange(cc_num, wheel_a.value)
            msg.channel = tx_ch - 1
            merge.inject(msg)
        output_generated = True

    # Wheel B.
    wheel_b.read()
    if wheel_b.changed:
        if config.get("wheel_b.mode") == "pitch_bend":
            msg = PitchBend(wheel_b.value)
            msg.channel = tx_ch - 1
            merge.inject(msg)
        else:
            cc_num = config.get("wheel_b.cc_number")
            msg = ControlChange(cc_num, wheel_b.value)
            msg.channel = tx_ch - 1
            merge.inject(msg)
        output_generated = True

    # --- Read buttons and inject events ---
    ato_value = config.get("button_b.aftertouch_value")

    # Button A.
    if button_a.fell or button_a.rose:
        if config.get("button_a.mode") == "aftertouch":
            val = ato_value if button_a.fell else 0
            msg = ChannelPressure(val)
        else:
            cc_num = config.get("button_a.cc_number")
            val = 127 if button_a.fell else 0
            msg = ControlChange(cc_num, val)
        msg.channel = tx_ch - 1
        merge.inject(msg)
        output_generated = True

    # Button B.
    if button_b.fell or button_b.rose:
        if config.get("button_b.mode") == "aftertouch":
            val = ato_value if button_b.fell else 0
            msg = ChannelPressure(val)
        else:
            cc_num = config.get("button_b.cc_number")
            val = 127 if button_b.fell else 0
            msg = ControlChange(cc_num, val)
        msg.channel = tx_ch - 1
        merge.inject(msg)
        output_generated = True

    # Footswitch.
    if footswitch.fell or footswitch.rose:
        if config.get("footswitch.mode") == "aftertouch":
            val = ato_value if footswitch.fell else 0
            msg = ChannelPressure(val)
        else:
            cc_num = config.get("footswitch.cc_number")
            val = 127 if footswitch.fell else 0
            msg = ControlChange(cc_num, val)
        msg.channel = tx_ch - 1
        merge.inject(msg)
        output_generated = True

    # --- Flush all queued messages to all outputs ---
    if output_generated:
        led_out.on()
    merge.flush()

    # --- Update display (only on value changes) ---
    if wheel_a.changed or wheel_b.changed:
        display.update_values(
            wheel_a=wheel_a.value if wheel_a.changed else None,
            wheel_b=wheel_b.value if wheel_b.changed else None
        )

    # --- LEDs off at end of cycle ---
    led_in.off()
    led_out.off()
