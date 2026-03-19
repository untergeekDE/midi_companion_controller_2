# midi_usb_host.py — USB Host MIDI input via MAX3421E breakout.
#
# Reads MIDI messages from a USB MIDI device (e.g., Novation Launchpad Pro 3)
# connected to the MAX3421E USB Host controller over SPI.
#
# This module handles:
# - SPI communication with the MAX3421E chip
# - USB device enumeration and MIDI interface detection
# - Hot-plug: graceful handling of connect/disconnect without crashing
# - Translating USB MIDI packets into adafruit_midi message objects
#   (same types as UartMidi, so MidiMerge can treat both identically)
#
# No device filtering is applied — any USB MIDI class-compliant device
# is accepted.
#
# Usage:
#     import busio
#     from hardware import pins
#     from hardware.midi_usb_host import UsbHostMidi
#     spi = busio.SPI(pins.SPI_CLK, MOSI=pins.SPI_MOSI, MISO=pins.SPI_MISO)
#     usb_midi = UsbHostMidi(spi, pins.USB_HOST_CS)
#     msg = usb_midi.receive()  # Returns adafruit_midi message or None

import digitalio
import max3421e
import usb.core
import adafruit_usb_host_midi
import adafruit_midi


class UsbHostMidi:
    """USB Host MIDI input via MAX3421E.

    Receives MIDI messages from a USB MIDI device connected to the
    MAX3421E breakout. Handles hot-plug gracefully.

    Args:
        spi: A busio.SPI instance for the MAX3421E.
        cs_pin: A board pin for the MAX3421E chip select.
    """

    def __init__(self, spi, cs_pin):
        self._spi = spi
        self._cs_pin = cs_pin
        self._host_chip = None
        self._midi_device = None
        self._midi_in = None
        self._init_host()

    def _init_host(self):
        """Initialize the MAX3421E USB Host controller.

        Called at startup. Sets up the chip select pin and creates
        the MAX3421E host object.
        """
        try:
            cs = digitalio.DigitalInOut(self._cs_pin)
            cs.direction = digitalio.Direction.OUTPUT
            cs.value = True
            self._host_chip = max3421e.MAX3421E(self._spi, cs)
            self._find_midi_device()
        except Exception:
            # MAX3421E not responding or not present — operate without USB input.
            self._host_chip = None
            self._midi_device = None
            self._midi_in = None

    def _find_midi_device(self):
        """Scan connected USB devices for a MIDI class-compliant interface.

        If found, wraps it in adafruit_midi.MIDI for message parsing.
        If not found, sets _midi_in to None (receive() will return None).
        """
        try:
            if self._host_chip is None:
                return
            # Find any connected USB MIDI device via usb.core.
            device = usb.core.find(find_all=False)
            if device is None:
                return
            raw_midi = adafruit_usb_host_midi.MIDI(device)
            self._midi_device = raw_midi
            self._midi_in = adafruit_midi.MIDI(midi_in=raw_midi, in_channel=None)
        except Exception:
            # No MIDI device found — that's fine, we'll try again on next call.
            self._midi_device = None
            self._midi_in = None

    def receive(self):
        """Read one MIDI message from the USB device, if available.

        Returns:
            An adafruit_midi message object, or None if no device is
            connected or no message is available.
        """
        # If no MIDI interface is active, try to find one (hot-plug).
        if self._midi_in is None:
            self._find_midi_device()
            if self._midi_in is None:
                return None

        try:
            return self._midi_in.receive()
        except Exception:
            # Device disconnected or communication error.
            # Reset state so we try to re-enumerate next time.
            self._midi_device = None
            self._midi_in = None
            return None
