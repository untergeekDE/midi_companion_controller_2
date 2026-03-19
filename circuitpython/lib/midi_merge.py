# midi_merge.py — MIDI merge engine.
#
# Collects messages from multiple MIDI inputs (USB Host, DIN In) and
# locally injected events (wheels, buttons), then sends everything
# to the configured outputs (DIN Out).
#
# The merge engine does not filter, remap channels, or modify messages.
# USB Host input is always forwarded. DIN input is only forwarded when
# thru_enabled is True (matching the original Teensy behavior).
#
# Usage:
#     merge = MidiMerge(
#         inputs={"usb": usb_midi, "din": uart_midi},
#         outputs=[uart_midi],
#         thru_enabled=False
#     )
#     # In the main loop:
#     merge.process()
#     merge.inject(some_midi_message)
#     # At end of loop:
#     merge.flush()


class MidiMerge:
    """Merges multiple MIDI inputs and locally injected events to outputs.

    The processing order per loop iteration is:
    1. process() — reads all inputs, queues messages for output
    2. inject() — adds locally generated messages (wheels, buttons)
    3. flush() — sends all queued messages to all outputs

    Attributes:
        had_input: True if any external input was received during the
            last process() call. Used to drive the MIDI-in LED.
        thru_enabled: Whether DIN MIDI input is forwarded to outputs.

    Args:
        inputs: Dict mapping source names to input objects. Each input
            must have a receive() method returning a message or None.
            Expected keys: "usb" for USB Host, "din" for DIN MIDI In.
        outputs: List of output objects. Each must have a send(msg) method.
        thru_enabled: If True, DIN input ("din") is forwarded to outputs.
            USB Host input ("usb") is always forwarded regardless.
    """

    def __init__(self, inputs, outputs, thru_enabled=False):
        self.inputs = inputs
        self.outputs = outputs
        self.thru_enabled = thru_enabled
        self.had_input = False
        self._queue = []  # Messages to send this cycle.

    def process(self):
        """Read all MIDI inputs and queue messages for output.

        USB Host input is always forwarded. DIN input is only forwarded
        when thru_enabled is True.

        Call once at the start of each main loop iteration.
        """
        self._queue = []
        self.had_input = False

        for name, source in self.inputs.items():
            # Read all available messages from this source.
            msg = source.receive()
            while msg is not None:
                self.had_input = True
                # DIN input is conditional on thru_enabled.
                # USB input is always forwarded.
                if name != "din" or self.thru_enabled:
                    self._queue.append(msg)
                msg = source.receive()

    def inject(self, msg):
        """Add a locally generated MIDI message to the output queue.

        Used by the main loop to inject wheel and button events.

        Args:
            msg: An adafruit_midi message object.
        """
        self._queue.append(msg)

    def flush(self):
        """Send all queued messages to all outputs.

        Call once at the end of each main loop iteration, after all
        process() and inject() calls are done.
        """
        for msg in self._queue:
            for output in self.outputs:
                output.send(msg)
        self._queue = []

    @property
    def has_output(self):
        """True if there are messages queued for output this cycle.

        Useful for driving the MIDI-out LED.
        """
        return len(self._queue) > 0
