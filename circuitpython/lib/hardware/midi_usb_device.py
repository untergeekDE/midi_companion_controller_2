# midi_usb_device.py — Native USB MIDI device interface.
#
# Wraps CircuitPython's built-in usb_midi module so the controller
# appears as a USB MIDI device when connected to a laptop or DAW.
#
# This is the Pico's NATIVE USB port — distinct from the MAX3421E USB
# Host port (midi_usb_host.py). Both can operate simultaneously:
#   - USB Host (MAX3421E): receives from Launchpad Pro 3
#   - USB Device (native):  sends/receives to/from laptop
#
# The controller merges all inputs and sends to ALL outputs (DIN + USB
# Device + optionally USB Host), so a laptop DAW sees the same merged
# MIDI stream as the Roland SE-02 on the DIN port.
#
# Usage:
#     from hardware.midi_usb_device import UsbDeviceMidi
#     usb_dev = UsbDeviceMidi(channel=0)
#     msg = usb_dev.receive()
#     usb_dev.send(some_midi_message)

import usb_midi
import adafruit_midi


class UsbDeviceMidi:
    """Native USB MIDI device interface.

    Makes the Pico appear as a USB MIDI device to a connected computer.
    Provides both input (receive from DAW) and output (send to DAW).

    Args:
        channel: Default MIDI output channel (0-15, where 0 = channel 1).
    """

    def __init__(self, channel=0):
        # usb_midi.ports is a tuple: (PortIn, PortOut)
        # These are available because boot.py does NOT call usb_midi.disable().
        self._midi_in = adafruit_midi.MIDI(
            midi_in=usb_midi.ports[0], in_channel=None
        )
        self._midi_out = adafruit_midi.MIDI(
            midi_out=usb_midi.ports[1], out_channel=channel
        )

    def set_channel(self, channel):
        """Update the default output channel.

        Args:
            channel: MIDI channel (1-16).
        """
        self._midi_out.out_channel = channel - 1

    def receive(self):
        """Read one MIDI message from the USB host (laptop/DAW).

        Returns:
            An adafruit_midi message object, or None if nothing available.
        """
        return self._midi_in.receive()

    def send(self, msg):
        """Send a MIDI message to the USB host (laptop/DAW).

        Args:
            msg: An adafruit_midi message object.
        """
        self._midi_out.send(msg)
