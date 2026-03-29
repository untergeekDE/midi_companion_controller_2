# midi_usb_host.py — USB Host MIDI via MAX3421E breakout.
#
# Reads and writes MIDI messages to/from a USB MIDI device (e.g., Novation
# Launchpad Pro 3) connected to the MAX3421E USB Host controller over SPI.
#
# This module handles:
# - SPI communication with the MAX3421E chip
# - USB device enumeration and MIDI interface detection
# - Hot-plug: graceful handling of connect/disconnect without crashing
# - Translating USB MIDI packets into adafruit_midi message objects
#   (same types as UartMidi, so MidiMerge can treat both identically)
# - Bidirectional: receive from device AND send merged output back to device
#
# No device filtering is applied — any USB MIDI class-compliant device
# is accepted.
#
# Usage:
#     import busio
#     from hardware import pins
#     from hardware.midi_usb_host import UsbHostMidi
#     spi = busio.SPI(pins.SPI_CLK, MOSI=pins.SPI_MOSI, MISO=pins.SPI_MISO)
#     usb_host = UsbHostMidi(spi, pins.USB_HOST_CS)
#     msg = usb_host.receive()  # Returns adafruit_midi message or None
#     usb_host.send(some_msg)   # Send merged output back to the device

import time

import digitalio
import max3421e
import usb.core
import adafruit_usb_host_midi
import adafruit_midi


class UsbHostMidi:
    """USB Host MIDI via MAX3421E — bidirectional.

    Receives and sends MIDI messages to/from a USB MIDI device connected
    to the MAX3421E breakout. Handles hot-plug gracefully.

    Args:
        spi: A busio.SPI instance for the MAX3421E.
        cs_pin: A board pin for the MAX3421E chip select.
    """

    _SCAN_COOLDOWN_NS = 500_000_000  # 500ms between USB bus scans

    def __init__(self, spi, cs_pin):
        self._spi = spi
        self._cs_pin = cs_pin
        self._host_chip = None
        self._midi_device = None
        self._midi_in = None
        self._midi_out = None
        self._last_scan_time = 0
        self._init_host()

    def _init_host(self):
        """Initialize the MAX3421E USB Host controller."""
        try:
            cs = digitalio.DigitalInOut(self._cs_pin)
            cs.direction = digitalio.Direction.OUTPUT
            cs.value = True
            self._host_chip = max3421e.MAX3421E(self._spi, cs)
            self._find_midi_device()
        except (OSError, RuntimeError, ValueError):
            self._host_chip = None
            self._midi_device = None
            self._midi_in = None
            self._midi_out = None

    def _find_midi_device(self):
        """Scan connected USB devices for a MIDI class-compliant interface.

        Sets up both input and output MIDI streams if a device is found.
        """
        try:
            if self._host_chip is None:
                return
            device = usb.core.find(find_all=False)
            if device is None:
                return
            raw_midi = adafruit_usb_host_midi.MIDI(device)
            self._midi_device = raw_midi
            self._midi_in = adafruit_midi.MIDI(midi_in=raw_midi, in_channel=None)
            self._midi_out = adafruit_midi.MIDI(midi_out=raw_midi)
        except (OSError, RuntimeError, ValueError):
            self._midi_device = None
            self._midi_in = None
            self._midi_out = None

    def _reset(self):
        """Reset MIDI state after a device error or disconnection."""
        self._midi_device = None
        self._midi_in = None
        self._midi_out = None

    def receive(self):
        """Read one MIDI message from the USB device, if available.

        Returns:
            An adafruit_midi message object, or None if no device is
            connected or no message is available.
        """
        if self._midi_in is None:
            now = time.monotonic_ns()
            if (now - self._last_scan_time) >= self._SCAN_COOLDOWN_NS:
                self._last_scan_time = now
                self._find_midi_device()
            if self._midi_in is None:
                return None

        try:
            return self._midi_in.receive()
        except (OSError, RuntimeError, ValueError):
            self._reset()
            return None

    def send(self, msg):
        """Send a MIDI message to the USB device (e.g., back to Launchpad).

        Silently does nothing if no device is connected.

        Args:
            msg: An adafruit_midi message object.
        """
        if self._midi_out is None:
            return

        try:
            self._midi_out.send(msg)
        except (OSError, RuntimeError, ValueError):
            self._reset()
