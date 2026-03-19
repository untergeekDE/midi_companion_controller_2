# midi_uart.py — DIN MIDI input and output over UART.
#
# Wraps adafruit_midi over a busio.UART at the standard MIDI baud rate
# of 31250. Connects to a DIN MIDI circuit (optocoupler for input,
# resistor network for output) or an Adafruit MIDI FeatherWing.
#
# Both UartMidi and UsbHostMidi use adafruit_midi message types, so
# MidiMerge can treat them identically.
#
# Usage:
#     from hardware.midi_uart import UartMidi
#     from hardware import pins
#     midi = UartMidi(pins.UART_TX, pins.UART_RX)
#     msg = midi.receive()
#     if msg is not None:
#         print(f"Got: {msg}")
#     midi.send_cc(1, 64, channel=1)

import busio
import adafruit_midi
from adafruit_midi.control_change import ControlChange
from adafruit_midi.pitch_bend import PitchBend
from adafruit_midi.channel_pressure import ChannelPressure

# Standard MIDI baud rate.
_MIDI_BAUD = 31250


class UartMidi:
    """DIN MIDI input and output over UART.

    Provides receive() for reading incoming MIDI messages and send methods
    for outgoing messages. Uses adafruit_midi message types internally.

    Args:
        tx_pin: Board pin for UART TX (MIDI Out).
        rx_pin: Board pin for UART RX (MIDI In).
        channel: Default MIDI channel for sending (0-15, where 0 = channel 1).
            Can be overridden per-send call.
    """

    def __init__(self, tx_pin, rx_pin, channel=0):
        self._uart = busio.UART(tx_pin, rx_pin, baudrate=_MIDI_BAUD, timeout=0.001)
        # adafruit_midi.MIDI wraps a UART-like stream for message parsing.
        self._midi_in = adafruit_midi.MIDI(midi_in=self._uart, in_channel=None)
        self._midi_out = adafruit_midi.MIDI(midi_out=self._uart, out_channel=channel)

    def set_channel(self, channel):
        """Update the default output channel.

        Args:
            channel: MIDI channel (1-16).
        """
        self._midi_out.out_channel = channel - 1

    def receive(self):
        """Read one MIDI message from the DIN input, if available.

        Returns:
            An adafruit_midi message object, or None if no complete message
            is available yet.
        """
        return self._midi_in.receive()

    def send(self, msg):
        """Send a MIDI message object to the DIN output.

        Args:
            msg: An adafruit_midi message (NoteOn, ControlChange, PitchBend, etc.).
        """
        self._midi_out.send(msg)

    def send_cc(self, cc_number, value, channel=None):
        """Send a MIDI Control Change message.

        Args:
            cc_number: CC number (0-127).
            value: CC value (0-127).
            channel: MIDI channel (1-16). Uses default channel if None.
        """
        ch = (channel - 1) if channel is not None else None
        msg = ControlChange(cc_number, value)
        if ch is not None:
            msg.channel = ch
        self._midi_out.send(msg)

    def send_pitch_bend(self, value, channel=None):
        """Send a MIDI Pitch Bend message.

        Args:
            value: Pitch bend value (-8192 to 8191).
            channel: MIDI channel (1-16). Uses default channel if None.
        """
        ch = (channel - 1) if channel is not None else None
        msg = PitchBend(value)
        if ch is not None:
            msg.channel = ch
        self._midi_out.send(msg)

    def send_aftertouch(self, value, channel=None):
        """Send a MIDI Channel Pressure (Aftertouch) message.

        Args:
            value: Pressure value (0-127).
            channel: MIDI channel (1-16). Uses default channel if None.
        """
        ch = (channel - 1) if channel is not None else None
        msg = ChannelPressure(value)
        if ch is not None:
            msg.channel = ch
        self._midi_out.send(msg)
