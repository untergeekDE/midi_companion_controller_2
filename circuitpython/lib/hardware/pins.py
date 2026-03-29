# pins.py — Central pin assignment registry for the MIDI Companion Controller
#
# This is the SINGLE SOURCE OF TRUTH for all GPIO pin assignments on the
# Raspberry Pi Pico 2 (RP2350). Every other module that needs to talk to
# hardware MUST import its pin constants from here.
#
# WHY THIS MATTERS:
# Scattering GPIO numbers across multiple files makes it easy to accidentally
# assign two functions to the same pin, or to forget which pin does what.
# Centralising everything here means:
#   - Pin conflicts are immediately visible (duplicates stand out)
#   - Changing a pin assignment only requires editing this one file
#   - New developers get a complete hardware map in one place
#
# NAMING CONVENTION:
# Constants are named by function, not by GPIO number. For example, UART_TX
# rather than GP4. This makes the intent clear and lets you remap a pin
# without hunting through calling code.
#
# 2026-03-28: Adapting for easier soldering (JE)
#
#   GP0   → BUTTON_A (Pin 1)
#   GP1   → BUTTON_B (pin 2)
#           GND (Pin 3)
#   GP2   → LED1 (pin 4)
#   GP3   → LED2 (pin 5)
#   GP4   → UART_TX (pin 6)
#   GP5   → UART_RX (Pin 7)
#   GP13  → FOOTSWITCH (Pin 17)
#           GND (Pin 18)
#   GP14  → USB_HOST_INT (Pin 19)
#   GP16  → SPI_MISO (pin 21)
#   GP18  → SPI_CLK (pin 24)
#   GP19  → SPI_MOSI (Pin 25)
#   GP20  → I2C_SDA (pin 26)
#   GP21  → I2C_SCL (pin 27)
#           GND (pin 28)
#   GP22  → USB_HOST_CS (pin 29)
#   GP26/A0    → WHEEL_A (pin 31)
#   GP27/A1    → WHEEL_B (pin 32)
#           AGND (pin 33)

import board

# ---------------------------------------------------------------------------
# Analog Inputs — Wheels / Expression Controls
# ---------------------------------------------------------------------------
# These pins connect to potentiometers (or expression pedals wired as voltage
# dividers). CircuitPython's analogio.AnalogIn reads a 0–65535 value that the
# main code maps to MIDI pitch bend or mod wheel messages.

# Pitch bend wheel — connected to an analog potentiometer.
# board.A0 is the CircuitPython alias for the first analog-capable pin (GP26).
WHEEL_A = board.A0

# Modulation wheel — connected to a second analog potentiometer.
# board.A1 maps to GP27.
WHEEL_B = board.A1

# ---------------------------------------------------------------------------
# Digital Inputs — Buttons
# ---------------------------------------------------------------------------
# All buttons are wired with one leg to the GPIO pin and the other leg to GND.
# We use INPUT_PULLUP so the pin rests HIGH and goes LOW when pressed.
# This "active low" scheme means no external resistors are needed.

# Button A — primary menu / navigation button.
# Also checked in boot.py to enable developer mode during boot.
# Must use INPUT_PULLUP; active low (LOW = pressed).
BUTTON_A = board.GP0

# Button B — confirm / set / secondary action button.
# Must use INPUT_PULLUP; active low (LOW = pressed).
BUTTON_B = board.GP1

# Footswitch — external 1/4" TS jack for a latching or momentary footswitch.
# Wired the same as the buttons: tip to GPIO, sleeve to GND, INPUT_PULLUP.
# Active low (LOW = footswitch closed / pressed).
FOOTSWITCH = board.GP13

# ---------------------------------------------------------------------------
# Digital Outputs — Status LEDs
# ---------------------------------------------------------------------------
# Two LEDs provide visual feedback about MIDI activity.
# They are driven directly from GPIO pins through current-limiting resistors.
# HIGH = LED on, LOW = LED off.

# LED 1 — MIDI input activity indicator.
# Blinks or lights when a MIDI message is received on the DIN-5 input.
LED1 = board.GP2

# LED 2 — MIDI output activity indicator.
# Blinks or lights when a MIDI message is transmitted on the DIN-5 output.
LED2 = board.GP3

# ---------------------------------------------------------------------------
# I2C Bus — Display and Peripherals
# ---------------------------------------------------------------------------
# I2C is used for the OLED display (and potentially other peripherals like
# an external EEPROM or real-time clock in future revisions).
# The Pico 2 has two I2C controllers; we use I2C0 here on these pins.

# I2C clock line. All I2C devices share this signal.
I2C_SCL = board.GP21

# I2C data line. All I2C devices share this signal.
I2C_SDA = board.GP20

# ---------------------------------------------------------------------------
# UART — DIN-5 Hardware MIDI
# ---------------------------------------------------------------------------
# Hardware MIDI uses a 31,250-baud UART. CircuitPython's busio.UART is
# configured with this baud rate and connected to optocoupler circuits
# on the MIDI IN and MIDI OUT DIN-5 jacks.
#
# MIDI OUT: TX pin drives a current loop through the DIN-5 connector.
# MIDI IN:  RX pin receives the signal after the optocoupler isolates it.

# UART transmit — connects to the MIDI OUT DIN-5 jack via a driver circuit.
UART_TX = board.GP4

# UART receive — connects to the MIDI IN DIN-5 jack via an optocoupler.
UART_RX = board.GP5

# ---------------------------------------------------------------------------
# SPI Bus — High-Speed Peripherals
# ---------------------------------------------------------------------------
# SPI is used for any peripherals that need faster data rates than I2C can
# provide, such as a color display, DAC, or SD card. All SPI devices share
# the clock, MOSI, and MISO lines; each device gets its own chip-select (CS)
# line to determine which one is active at any moment.

# SPI clock — shared by all SPI devices.
SPI_CLK = board.GP18

# SPI MOSI (Master Out Slave In) — data FROM the Pico TO the peripheral.
SPI_MOSI = board.GP19

# SPI MISO (Master In Slave Out) — data FROM the peripheral TO the Pico.
SPI_MISO = board.GP16

# ---------------------------------------------------------------------------
# USB Host Interface
# ---------------------------------------------------------------------------
# The Pico 2 can act as a USB host (connecting USB MIDI keyboards, etc.)
# using a MAX3421E or similar USB host controller chip over SPI.
# The chip-select and interrupt pins below are specific to that controller.
# The SPI data lines (SPI_CLK, SPI_MOSI, SPI_MISO above) are shared.

# USB host controller chip-select.
# Drive LOW to select the USB host chip on the SPI bus; HIGH to deselect it.
USB_HOST_CS = board.GP22

# USB host controller interrupt.
# The host chip drives this pin LOW when it has data ready for the Pico.
# Configure as INPUT (no pull needed if push-pull, INPUT_PULLUP if open-drain).
# NOTE: Cannot use GP26 here — that is board.A0 (WHEEL_A analog input).
USB_HOST_INT = board.GP14
