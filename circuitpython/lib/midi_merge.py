# midi_merge.py — MIDI merge engine.
#
# Collects messages from multiple MIDI inputs (USB Host, USB Device, DIN In)
# and locally injected events (wheels, buttons), then sends everything
# to the configured outputs (DIN Out, USB Device, USB Host).
#
# Echo prevention: messages from a source are NOT sent back to that same
# source. For example, data received from the Launchpad (USB Host) is sent
# to DIN Out and USB Device, but NOT back to the Launchpad.
#
# The merge engine does not filter, remap channels, or modify messages.
# DIN input is only forwarded when thru_enabled is True.
# All other inputs are always forwarded.
#
# Usage:
#     merge = MidiMerge(
#         inputs={"usb_host": usb_host, "usb_device": usb_dev, "din": uart},
#         outputs={"din": uart, "usb_device": usb_dev, "usb_host": usb_host},
#         thru_enabled=False
#     )
#     merge.process()
#     merge.inject(some_midi_message)
#     merge.flush()


class MidiMerge:
    """Merges multiple MIDI inputs and locally injected events to outputs.

    The processing order per loop iteration is:
    1. process() — reads all inputs, queues messages for output
    2. inject() — adds locally generated messages (wheels, buttons)
    3. flush() — sends all queued messages to outputs (avoiding echo)

    Attributes:
        had_input: True if any external input was received during the
            last process() call. Used to drive the MIDI-in LED.
        thru_enabled: Whether DIN MIDI input is forwarded to outputs.

    Args:
        inputs: Dict mapping source names to input objects. Each input
            must have a receive() method returning a message or None.
        outputs: Dict mapping output names to output objects. Each must
            have a send(msg) method. Names matching input names enable
            echo prevention (e.g., "usb_host" in both dicts means USB
            Host input won't be echoed back to USB Host output).
        thru_enabled: If True, DIN input ("din") is forwarded to outputs.
    """

    _MAX_QUEUE = 64

    def __init__(self, inputs, outputs, thru_enabled=False):
        self.inputs = inputs
        self.outputs = outputs
        self.thru_enabled = thru_enabled
        self.had_input = False
        # Queue entries are (msg, source_name) tuples.
        # source_name is None for locally injected messages.
        self._queue = []
        # Warn about mismatched names (development aid for echo prevention).
        for name in inputs:
            if name not in outputs:
                print(f"MidiMerge: input '{name}' has no matching output")

    def process(self):
        """Read all MIDI inputs and queue messages for output.

        DIN input is only forwarded when thru_enabled is True.
        All other inputs are always forwarded.

        Call once at the start of each main loop iteration.
        """
        self._queue = []
        self.had_input = False

        for name, source in self.inputs.items():
            msg = source.receive()
            while msg is not None:
                self.had_input = True
                # DIN input is conditional on thru_enabled.
                if name != "din" or self.thru_enabled:
                    if len(self._queue) >= self._MAX_QUEUE:
                        break
                    self._queue.append((msg, name))
                msg = source.receive()

    def inject(self, msg):
        """Add a locally generated MIDI message to the output queue.

        Locally injected messages are sent to ALL outputs (no echo issue).

        Args:
            msg: An adafruit_midi message object.
        """
        if len(self._queue) >= self._MAX_QUEUE:
            return
        self._queue.append((msg, None))

    def flush(self):
        """Send all queued messages to outputs, avoiding echo loops.

        Messages from a named source are sent to all outputs EXCEPT the
        output with the same name. Locally injected messages (source=None)
        are sent to all outputs.

        Call once at the end of each main loop iteration.
        """
        for msg, source_name in self._queue:
            for out_name, output in self.outputs.items():
                # Don't echo a message back to its source.
                if source_name is not None and out_name == source_name:
                    continue
                output.send(msg)
        self._queue = []

    @property
    def has_output(self):
        """True if there are messages queued for output this cycle."""
        return len(self._queue) > 0
