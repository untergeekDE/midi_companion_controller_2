# midi_companion_controller_2
Code for a Teensy-based MIDI controller with two wheels, two buttons, two LEDs, and an OLED display

![Companion Controller V2.0 hardware](https://www.untergeek.de/wp-content/uploads/2017/08/20170823_071628-560x315.jpg)

## Files:
- *MIDI-Controller-v2.0.ino:* The Teensyduino sketch
- *midicompanion2-0.fzz:* Schematic and breadboard file for Fritzing
- *midicompanion2-0_Schematic.png*
- *midicompanion2-0_breadboard-plan.png*

Please note that the Fritzing breadboard diagram is missing a ground connection for left pin of the left LED (and the switch); it is there in the schematic, but remember this when soldering. 

Needs my adapted OLED library to work - library can be found in this repository: 
https://github.com/untergeekDE/OLED_I2C_128x64_Monochrome_Library
(Store files to a subfolder called OLED_I2C_128x64_Monochrome in the Arduino/Teensy Libraries folder)

Project described on https://www.untergeek.de/2017/09/v2-0-a-teensy-based-midi-controller/
