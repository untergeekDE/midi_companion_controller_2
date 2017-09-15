
// ------------------------------------------------------------------------
// Simple MIDI Controller, based on Teensy 3.2
// Derived from V1.0 with SparkFun DangerShield and Arduino
// CC-BY untergeek.de                              V2.0, 9-Aug-2017
//
// Crude and simple code which turns the DangerShield into a MIDI controller
// with pitch bend and CC wheels (Modwheel per default), and three 
// performance controller keys. MIDI input is read for the MIDI channel used
// and echoed to MIDI Out. Generated MIDI data (Pitch Bend, ATO, and CCs) is merged
// with echoed MIDI. 
//
// Todo: 
// - Debounce library
// - DOUBLE keypress for configuration menu
// - Configure wheels - center position for PB wheel and upper/lower range for both wheels
// - Configuration saved to EEPROM; read back on startup if chksum is correct
// - Adapt display library
// -- Consistent cursor handling
// -- String handler
// --- Overload printf function??
// -- UTF-8 conversion? 
// --- (Chipset is CP437 at the moment, maybe CP1252 later)
// Nice-to-have: 
// - Optimize standard Arduino libraries for performance
// - Interrupt-base clocking of input scanning and MIDI output for more 
// - Setup/configuration with EEPROM storage 
// ----------------------------------------------------------

// V0.4: 
// - Bounce2 debounce library
// - Detect double click to call menu 
// V0.3:
// - Menu routine to configure EEPROM parameters
// - Adjustable deadband
// - Negative characters (white background)
// V0.2: 
// - Generalized Wheel and Button handling routines (no hard-coded CC)
// - MIDI Thru parameter
// - LEDs fixed: LED1 signals MIDI input, LED2 signals MIDI output (w/o thru)

const byte VERSION[] PROGMEM   = "FW V0.3a";


// MIDI library includes - for the time being, only serial MIDI, no USB as a class-compliant device

#include <MIDI.h>
#include <SPI.h>
#include <Wire.h>
#include <Bounce2.h>


#include <avr/pgmspace.h>
#include <OLED_I2C_128x64_Monochrome.h>

#define OLED_RESET 4

// Constructor display


// MIDI constructor micros: 
// Set serial MIDI I/O as "midiA"
// Set MIDI USB as "midiB" 

    MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midiA);
    MIDI_CREATE_INSTANCE(HardwareSerial, SerialUSB, midiB);

// DangerShield CapSense proximity sensor, read with the "CapSense" library
// Definitions enabling the CapSense function which we do not use yet
// #include <CapSense.h>

// CONSTANTS
// Wheel definitions

#define WHEEL_A  22 //Left Wheel, Pitch Bend 
#define WHEEL_B  23 //Right Wheel

#define BUTTON_A  17 //Left button
#define BUTTON_B  16 //Right button
#define BUTTON_C  15 //External button

#define LED1  15 // Top LED signals MIDI input
#define LED2  14 // Bottom LED signals generated MIDI output


// MIDI CC definitions
#define CCSLIDER_B 1 // Mod Wheel
#define CCSLIDER_C 2 // Breath

#define CCBUTTON_B 64 // Sustain
#define CCBUTTON_C 66 // Sostenuto

#define DEFAULT_MIDI_CH 1

// "Deadband" zones for Pitch Bend: Only 7/8 of each half of the fader
// produce changes; values around the middle are nulled
#define DEADBAND 5

// String constants in PROGMEM 
//
const byte t1[] PROGMEM = "MidiControl V2.0";
const byte t2[] PROGMEM = "----------------";
const byte t3[] PROGMEM = "Ch";
const byte t4[] PROGMEM = "www.untergeek.de";
   
// Global variables
byte MidiRxCh = 0;
byte MidiTxCh = DEFAULT_MIDI_CH;
int wheel_a_delta, wheel_a_center; 
int wheel_b_delta, wheel_b_center;
int value;


boolean RefreshFlag = false;

// EEPROM Read/Write Routines

#include <EEPROM.h>

const byte nvrInits[32] = {
  0,  //CHKSUM
  128,  //WheelA CC (0x80=Pitch Bend)
  1,  //WheelB CC
  64, //SwitchA CC
  128, //SwitchB CC (0x80 = ATO)
  64, //SwitchC CC
  05, 02, //WheelA Delta -> 517
  
  146, 02,  //08 WheelA Center = 658
  0, 01,  //10 WheelA Hi = 256
  188, 02,  //12 WheelB Lo = 700
  241, 03,  //14 WheelB Center = 909

  0, 01,  //16 WheelB Hi = 256
  1,    // 18 TX Default Channel
  64,   // 19 ATO Default Value
  0,    // 20 MIDI Thru
  5,    // 21 Deadband width
  0,0,
  
  0,0,0,0,
  0,0,0,0   //unused
};

#define CHKSUM_OK 0x3F //Arbitrary value you won't find in the EPROM by accident

byte nvrChksum()
{
  byte chksum = 0; 
  // chksum is simple add
  for (int i = 1;i<32;i++)
  {
    chksum += EEPROM[i];
  }
  return chksum-CHKSUM_OK;
}

void nvrUpdate(byte i, byte v)
{
  EEPROM.update(i,v); 
  EEPROM.update(0,nvrChksum()-CHKSUM_OK);
}



void nvrInit()
{
  for (int i=1;i<32;i++)
  {
    EEPROM.write(i,nvrInits[i]);
  }
  EEPROM.update(0,nvrChksum()-CHKSUM_OK);  
}

/*
Code taken from http://jmsarduino.blogspot.de/2009/10/4-way-button-click-double-click-hold.html
MULTI-CLICK: One Button, Multiple Events

Oct 12, 2009
Run checkButton() to retrieve a button event:
Click
Double-Click
Hold and Long Hold are not used here. 
*/

// Button timing variables
int debounce = 20; // ms debounce period to prevent flickering when pressing or releasing the button
int DCgap = 250; // max ms between clicks for a double click event
int holdTime = 2000; // ms hold period: how long to wait for press+hold event
int longHoldTime = 5000; // ms long hold period: how long to wait for press+hold event

// Other button variables
boolean buttonVal = HIGH; // value read from button
boolean buttonLast = HIGH; // buffered value of the button's previous state
boolean DCwaiting = false; // whether we're waiting for a double click (down)
boolean DConUp = false; // whether to register a double click on next release, or whether to wait and click
boolean singleOK = true; // whether it's OK to do a single click
long downTime = -1; // time the button was pressed down
long upTime = -1; // time the button was released
boolean ignoreUp = false; // whether to ignore the button release because the click+hold was triggered
boolean waitForUp = false; // when held, whether to wait for the up event
boolean holdEventPast = false; // whether or not the hold event happened already
boolean longHoldEventPast = false;// whether or not the long hold event happened already


Bounce button1 = Bounce();
Bounce button2 = Bounce();


// the setup routine runs once when you press reset:
void setup() {
  //
  oled.initialize();
  oled.rotateDisplay180();
  oled.setBrightness(120);
  oled.clearDisplay();
  oled.fontsize = 1;
  print(t2);
  oled.invert(true);
  print(t1);
  print(t4);
  oled.invert(false);
  print(t2);
  oled.fontsize = 2;
  print(VERSION);
  delay(1500);
  oled.clearDisplay();
  // EEPROM chksum OK? If no, initialize.
  if (nvrChksum() != EEPROM[0])
    nvrInit();
  // Read wheel offsets into global variables for speed
  wheel_a_delta = EEPROM[6]+256*EEPROM[7];
  wheel_a_center = EEPROM[8]+256*EEPROM[9];
  wheel_b_delta = EEPROM[12]+256*EEPROM[13];
  wheel_b_center = EEPROM[14]+256*EEPROM[15];
  
  
  // initialize the digital pin as an output.



  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  button1.attach(BUTTON_A);
  button1.interval(5); // interval in ms
  button2.attach(BUTTON_B);
  button2.interval(5);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);

  // Initialize NoteOn handler and MIDI
  
  // De-activate MIDI echo; we are doing this manually
  midiA.turnThruOff(); 

  midiA.begin(MIDI_CHANNEL_OMNI); //Listen to all channels
  midiB.turnThruOff();
  midiB.begin(MIDI_CHANNEL_OMNI);

  menu();
  
  // Display default MIDI channel read from EEPROM
  oled.clearDisplay();
  oled.fontsize=3;
  print(t3);
  oled.printNum(MidiTxCh = EEPROM[18]);
  oled.fontsize=1;
  oled.printChar('A',0,3);
  oled.printChar('B',0,5);

}

// writeText routine nicked and repurposed from Adafruit and Deloarts code

void print(char *str)
{
  byte * s = (byte *) str; 
  print(s); 
}

void print(byte *str)
{
  // SET PROGMEM BUFFER
  unsigned char c;
  // PRINT CHAR FROM PROGMEM
  
  if (!str)
  {
    return;
  }
    while ((c = *str++))
    {
 
        oled.printChar(c);
    }
}

void print(const byte str[])
{
  // SET PROGMEM BUFFER
  unsigned char c;
  // PRINT CHAR FROM PROGMEM
  if (!str)
  {
    return;
  }
    while ((c = pgm_read_byte(str++)))
    {
 
        oled.printChar(c);
    }
}



int readWheel(int a, int  delta, boolean centered)
// Reads analog port and adjusts for calibration value delta
// I keep that one simple: The potentiometers in the wheel box
// have about just a bit more than a quarter of the full range to turn. 
// So I am assuming a maximal range of 256 (of the maximal 1024-value range)
// which allows for a simple division to get the output value.
// 
// Adjusted for the DEADBAND constant defining the zone which is read as 0
//  
// If it is a pitch wheel (centered = true), return signed 8-bit value,
// if it is a mod wheel (centered = false), return 7-bit positive value
{
  int v = analogRead(a) - delta;
  if (centered)
  { 
    // Assuming the pot normally covers just a bit more than 256 values-
    // and that center really means the middle value of the range
    
    if (v > -DEADBAND && v < DEADBAND) value = 0; // Deadband adjust
    if (v < -128) v = -128;
      else if (v > 127) v = 127; // Clip maximal values
    // No division necessary here
  } else { // not centered
    if (v < DEADBAND) v = 0; 
    if (v > 255) v= 255; 
    v >>= 1; // divide by two
  }
  return v;
}


const byte m0[] PROGMEM = "SETTINGS";
const byte m1[] PROGMEM = "______cc________";
const byte m2[] PROGMEM = "Wh A:    TxCh   ";
const byte m3[] PROGMEM = "Wh B:    Thru   ";
const byte m4[] PROGMEM = "BtnA:    Ato    ";
const byte m5[] PROGMEM = "BtnB:    DB     ";
const byte m6[] PROGMEM = "FtSw:    CAL    ";


void calibrate()
{
  return;
} 


void menu()
{
  oled.clearDisplay();
  oled.fontsize=2;
  oled.invert(true);
  print(m0);
  oled.invert(false);
  oled.fontsize=1;
  oled.setCursor(0,2);
// Crude but effective - 6 preformatted lines of text
  print(m1);
  print(m2);
  print(m3);
  print(m4);
  print(m5);
  print(m6);
  int menu = 10;
  for(;;)
  {
    button1.update();
    button2.update();
    for(int i = 0;i < 10;i++)
    {
      oled.setCursor(5+((i/5)* 8), (i % 5)+3);
      oled.invert(i == menu); 
      if (i<5)
      {
        if (i == menu) 
          nvrUpdate(i+1,readWheel(WHEEL_B,wheel_b_delta,false));
        oled.printNum(EEPROM[i+1]);
      }  
      switch (i) {
        case 5:
        {
          if (i == menu) 
            nvrUpdate(18,readWheel(WHEEL_B,wheel_b_delta,false)/8+1);
          oled.printNum(EEPROM[18]);
        } 
        break;
        case 6: // MIDI THRU
        {
          if (i == menu) 
            nvrUpdate(20,readWheel(WHEEL_B,wheel_b_delta,false)/64);
          oled.printNum(EEPROM[20]);           
        }
        break;
        case 7: // ATO
        {
          if (i == menu) 
            nvrUpdate(19,readWheel(WHEEL_B,wheel_b_delta,false));
          oled.printNum(EEPROM[19]);           
        }
        break;
        case 8: // Deadband 
        {
          if (i == menu) 
            nvrUpdate(21,readWheel(WHEEL_B,wheel_b_delta,false)/8);
          oled.printNum(EEPROM[21]);           
        }
        break;
        case 9: // Calibrate routine
          oled.printChar('G');
          oled.printChar('O');
          if (button1.fell())
            calibrate();
        break;
      }        
    }
    if (button1.fell())
    {
      if (menu != 10) menu = 0; 
      else return; 
    }
    if (button2.fell())
    {
      if (++menu > 10) menu = 0;       
    }
  }
}

// Callback Routines
void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  // UNUSED - doing this by hand as even with a callback routine, you'd still have to 
  // do a midiA.read() in the main loop. 

  // Try to keep your callbacks short (no delays ect)
  // otherwise it would slow down the loop() and have a bad impact
  // on real-time performance.
}



/*****************************************************************************/
// The fun starts here - the simple main routine
/*****************************************************************************/


int value_wheel_a = 0, value_wheel_b = 0;
boolean bvalue = false, value_button_a = false, value_button_b = false, value_button_c = false;

// the loop routine runs over and over again forever:
void loop() {

  //Adjust MIDI channel, display it, and light LED when data arrives
    midi::MidiType mt;
    char md1 = 0, md2 = 0;
  if (midiA.read())
  {
    digitalWrite(LED1, HIGH);
    MidiRxCh = midiA.getChannel();
    mt = midiA.getType();
    md1 = midiA.getData1();
    md2 = midiA.getData2();
    // MIDI Thru manually - this helps in dealing with USB MIDI later on
    if (EEPROM[20] != 0)  
    {
      midiA.send(mt,md1,md2,MidiRxCh);
      digitalWrite(LED2, HIGH);
      oled.fontsize=1;
      oled.setCursor(0,7);
      oled.printHex(MidiRxCh);
      oled.printChar(' ');
      oled.printHex(mt);
      oled.printChar(' ');
      oled.printHex(md1);
      oled.printChar(' ');
      oled.printHex(md2);
    }
    if (MidiTxCh != MidiRxCh)
    {
      RefreshFlag = true;
      MidiTxCh = MidiRxCh; // Set new channel
    // Display the new channel
      oled.fontsize=3;
      oled.px=0;  
      oled.py=0;
      print(t3);
      oled.printNum(MidiRxCh);
    }
  }
  

  // Wheel A
  boolean isPitch = (EEPROM[1] > 127);
  value = readWheel(WHEEL_A,isPitch ? wheel_a_center : wheel_a_delta, isPitch);
  if ((value_wheel_a != value)  || RefreshFlag)
  {
    digitalWrite(LED2,HIGH); 
    value_wheel_a = value;
    if (isPitch)
    {
//      With v4.2 of the MIDI library, positive values only
//      midiA.sendPitchBend((value+128)*64,MidiTxCh);
//      usbMIDI.sendPitchBend((value+128)*64,MidiTxCh);
// ...but with the most recent version of the MIDI library, V4.3:
      midiA.sendPitchBend((value)*64,MidiTxCh);
      usbMIDI.sendPitchBend((value)*64,MidiTxCh);
    } else {
    // Ignore ATO for the moment
      midiA.sendControlChange(EEPROM[1] % 0x7f,value,MidiTxCh);
      usbMIDI.sendControlChange(EEPROM[1] % 0x7f,value,MidiTxCh);  
    }
    
  // Display code
    oled.fontsize = 2; 
    oled.setCursor(4,3);
    oled.printNum(value); 
  }

  // Wheel B
  isPitch = (EEPROM[2] > 127);
  
  value = readWheel(WHEEL_B,isPitch ? wheel_b_center : wheel_b_delta, isPitch);
  if ((value_wheel_b != value)  || RefreshFlag)
  {
    digitalWrite(LED2,HIGH); 
    value_wheel_b = value;
    if (isPitch)
    {
      midiA.sendPitchBend(value*64,MidiTxCh);
      usbMIDI.sendPitchBend(value*64,MidiTxCh);
    } else {
    // Ignore ATO for the moment
      midiA.sendControlChange(EEPROM[2] % 0x7f,value,MidiTxCh);
      usbMIDI.sendControlChange(EEPROM[2] % 0x7f,value,MidiTxCh);  
    }
    
  // Display code
    oled.fontsize = 2; 
    oled.setCursor(4,5);
    oled.printNum(value); 
  }


  // Button A
  bvalue = !digitalRead(BUTTON_A);
  if ((value_button_a != bvalue)  || RefreshFlag)
  {
    digitalWrite(LED2,HIGH);
    if (EEPROM[3] > 0x80) // Aftertouch?
    {
      midiA.sendAfterTouch(bvalue*EEPROM[19],MidiTxCh);
      usbMIDI.sendAfterTouch(bvalue*EEPROM[19],MidiTxCh);     
    } else {
      midiA.sendControlChange(EEPROM[3],bvalue ? 127 : 0,MidiTxCh);
      usbMIDI.sendControlChange(EEPROM[3],bvalue ? 127 : 0,MidiTxCh);
    }
    value_button_a = bvalue;
    oled.fontsize=1;
    oled.printChar(bvalue ? '*' : ' ',12,3); 
  }
  
  // Button B
  bvalue = !digitalRead(BUTTON_B);
  if ((value_button_b != bvalue)  || RefreshFlag)
  {
    digitalWrite(LED2,HIGH);
    if (EEPROM[4] > 0x80) // Aftertouch?
    {
      midiA.sendAfterTouch(bvalue*EEPROM[19],MidiTxCh);
      usbMIDI.sendAfterTouch(bvalue*EEPROM[19],MidiTxCh);     
    } else {
      midiA.sendControlChange(EEPROM[4],bvalue ? 127 : 0,MidiTxCh);
      usbMIDI.sendControlChange(EEPROM[4],bvalue ? 127 : 0,MidiTxCh);
    }
    value_button_b = bvalue;
    oled.fontsize=1;
    oled.printChar(bvalue ? '*' : ' ',12,5); 
  }
  
  // Button C (Footswitch)
  bvalue = !digitalRead(BUTTON_C);
  if ((value_button_c != bvalue)  || RefreshFlag)
  {
    digitalWrite(LED2,HIGH);
    if (EEPROM[5] > 0x80) // Aftertouch?
    {
      midiA.sendAfterTouch(bvalue*EEPROM[19],MidiTxCh);
      usbMIDI.sendAfterTouch(bvalue*EEPROM[19],MidiTxCh);     
    } else {
      midiA.sendControlChange(EEPROM[5],bvalue ? 127 : 0,MidiTxCh);
      usbMIDI.sendControlChange(EEPROM[5],bvalue ? 127 : 0,MidiTxCh);
    }
    value_button_c = bvalue;
    oled.fontsize=1;
    oled.printChar(bvalue ? '*' : ' ',12,7); 
  }
  
  
  //char * s =String(MidiRxCh)+":"+String(mt)+","+String(md1)+","+String(md2); 
  //oled.print(s);
  
  //Clean up
  RefreshFlag = false;
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

}
