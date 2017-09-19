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
// -- String handler
// --- Overload printf function??
// -- UTF-8 conversion? 
// --- (Chipset is CP437 at the moment, maybe CP1252 later)
// Nice-to-have: 
// - Interrupt-based clocking of input scanning and MIDI output for more 
// ----------------------------------------------------------

const byte VERSION[] PROGMEM = "FW V0.5";

// V0.6: (projected)
// - calibrate() routine extended
// V0.5:
// - Changed the display handling
// V0.4: 
// - Adjustable deadband
// - Bounce2 debounce library
// - Detect double click to call menu 
// V0.3:
// - Menu routine to configure EEPROM parameters
// - Negative characters (white background)
// V0.2: 
// - Generalized Wheel and Button handling routines (no hard-coded CC)
// - MIDI Thru parameter
// - LEDs fixed: LED1 signals MIDI input, LED2 signals MIDI output (w/o thru)




// MIDI library includes - for the time being, only serial MIDI, no USB as a class-compliant device

#include <MIDI.h>
#include <SPI.h>
#include <Wire.h>

#include <avr/pgmspace.h>

#include <Bounce2.h>
#include <EEPROM.h>

#include <OLED_I2C_128x64_Monochrome.h>

#define OLED_RESET 4

// Constructor display


// MIDI constructor micros: 
// Set serial MIDI I/O as "midiA"
// Use midiUSB commands for USB MIDI 

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

// Global variables
byte MidiRxCh = 0;
byte MidiTxCh = DEFAULT_MIDI_CH;
int wheel_a_delta, wheel_a_center; 
int wheel_b_delta, wheel_b_center;
byte deadband;
int value;
int value_wheel_a = 0, value_wheel_b = 0;

long doubleclick = 0; // millis tick for double-click detection

#define DC 250    // 250ms, two clicks in this time range count as double click

boolean RefreshFlag = false;

/********************************************** STRING CONSTANTS ***************************************************/
// The OLED library I extended has no proper print(); and println(); functions like its Adafruit_gfx counterpart.
// Strings are defined as PROGMEM constant and passed by a named referrer. Simple but effective. 

// Startup message
const byte t1[] PROGMEM = "LpCompanionCtrl2";
const byte t2[] PROGMEM = "----------------";
const byte t3[] PROGMEM = "Ch ";
const byte t4[] PROGMEM = "www.untergeek.de";

// Menu texts
const byte m0[] PROGMEM = "LCC2 ";
const byte m1[] PROGMEM = "------cc------xx";
const byte m2[] PROGMEM = "Wh A:    TxCh   ";
const byte m3[] PROGMEM = "Wh B:    Thru   ";
const byte m4[] PROGMEM = "BtnA:    Ato    ";
const byte m5[] PROGMEM = "BtnB:    DB     ";
const byte m6[] PROGMEM = "FtSw:    CAL    ";

const byte m7[] PROGMEM = "PWH"; 
const byte m8[] PROGMEM = "ATO";
const byte m9[] PROGMEM = " ON";
const byte ma[] PROGMEM = "OFF";


// Calibration texts
const byte c0[] PROGMEM = "Calibrate Wheels";
const byte c1[] PROGMEM = "Release Pitch Wh";
const byte c2[] PROGMEM = "and return MODWh";
const byte c3[] PROGMEM = "to zero position";
const byte c4[] PROGMEM = "and press button";
const byte c5[] PROGMEM = "OK";


   
/********************************************** PRINT ROUTINES ***************************************************/


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

// This is called/shown in normal operation: MIDI channel, values read from the wheels, buttons/foot switch
// Might add small MIDI IN display routine later
void workscreen()
{
  // Display default MIDI channel read from EEPROM
  oled.clearDisplay();
  oled.fontsize=3;
  print(t3);
  oled.printChar(EEPROM[18] < 10 ? ' ' : '1');
  oled.printChar(0x30 + EEPROM[18] % 10);
  oled.fontsize=1;
  oled.printChar('A',0,3);
  oled.printChar('B',0,5);
  RefreshFlag = true; 
}

/********************************************** Button and Wheel handling ********************************************/
//
// Using the Bounce2 library - this library samples the attached buttons with a timestamp, and locks out changes
// within a pre-defined time window (5-10ms). It also offers edge detections via the rose() and fell() methods. 
// Reading the wheels is adjusted for type - centered pitch wheel, or uncentered mod wheel? - and adjusted for offset
// and deadband. 

Bounce button1 = Bounce();
Bounce button2 = Bounce();
Bounce button3 = Bounce();


int readWheel(int a, int  delta, boolean centered)
// Reads analog port and adjusts for calibration value delta
// I keep that one simple: The potentiometers in the wheel box
// have about just a bit more than a quarter of the full range to turn. 
// So I am assuming a maximal range of 256 (of the maximal 1024-value range)
// which allows for a simple division to get the output value.
// 
// Adjusted for the deadband constant defining the zone which is read as 0
//  
// If it is a pitch wheel (centered = true), return signed 8-bit value,
// if it is a mod wheel (centered = false), return 7-bit positive value
{
  int v = analogRead(a) - delta;
  if (centered)
  {
    // Assuming the pot normally covers just a bit more than 256 values-
    // and that center really means the middle value of the range
    
    if (v > 0)
    {
      if (v < deadband) v = 0; else v -= deadband;
      if (v > 127) v = 127;       // clip upper band 
    } else { // v < 0
      if (v > -deadband) v = 0; else v += deadband;
      if (v < -128) v = -128;       // clip lower band
    } // if (v > 0)
    // No division necessary here
  } else { // not centered
    if (v < deadband) v = 0; else v -= deadband; 
    if (v > 255) v= 255; 
    v >>= 1; // divide by two
  }
  return v;
}

/********************************************** Parameter and NON-VOLATILE RAM (NVR) HANDLING ***********************************************/
// i.e.: EEPROM Read/Write Routines

// These are the default values to be written to EEPROM memory if there is nothing sensible yet. 

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
  10,    // 21 Deadband width
  0,0,
  
  0,0,0,0,
  0,0,0,0   //unused
};

#define CHKSUM_OK 0x3A //Arbitrary value you won't find in the EPROM by accident


// Build checksum by adding all bytes 
byte nvrChksum() 
{
  byte chksum = 0; 
  // chksum is simple add
  for (int i = 1;i<32;i++)
  {
    chksum += EEPROM[i];
  }
  return chksum - CHKSUM_OK;      
  // Substract the predefined constant - what you get from this is then stored in EEPROM[0] as chksum.
  // I.e., if EEPROM is valid, these conditions must apply: 
  // EEPROM[0] == nvrChksum()
}

void nvrUpdate(byte i, byte v)
{
  // Write updated value to EEPROM and adapt checksum)
  if (EEPROM[i] != v)
  {
    EEPROM.write(i,v);
    EEPROM.write(0,nvrChksum());     
  }
}

// Declare calibrate() to be able to use it
void calibrate(void);

void nvrInit()
// Write default values to NVRAM
{
  for (int i=1;i<32;i++)
  {
    EEPROM.write(i,nvrInits[i]);
  }
  EEPROM.write(0,nvrChksum()); // Set correct chksum to indicate that EEPROM is valid now
  // nvrInit() is usually called on the very first run of the hardware.
  // So let's ask the user for configuration settings (or at least calibrate the wheels): 
  calibrate();  
}

/************************************** Calibration and menu ***************************************************/

void calibrate()
// Set the offset/center value for the wheels, and try to discern a deadband range
{
  // TODO
  oled.clearDisplay();
  oled.fontsize = 1;
  oled.invert(true); 
  print(c0);
  oled.invert(false);
  print(c1);
  print(c2);
  print(c3);
  print(c4);
  print(t2);
  oled.invert(true);
  oled.fontsize=2;
  oled.setCursor(6,6);
  print(c5);
  while(!(button2.update()&& button2.fell())) /*wait for keypress*/ ;
  oled.clearDisplay();
  oled.fontsize = 2;
  wheel_a_center = analogRead(WHEEL_A);
  wheel_a_delta = wheel_a_center - 128;
  nvrUpdate(8,wheel_a_center % 256);
  nvrUpdate(9,wheel_a_center / 256);
  nvrUpdate(6,wheel_a_delta % 256);
  nvrUpdate(7,wheel_a_delta / 256);
  oled.printHex(EEPROM[7]);
  oled.printHex(EEPROM[6]);
  oled.printChar('\n');
  oled.printHex(EEPROM[9]);
  oled.printHex(EEPROM[8]);
  oled.printChar('\n');
  wheel_b_delta = analogRead(WHEEL_B);
  wheel_b_center = wheel_b_delta + 128;
  nvrUpdate(12,wheel_b_delta % 256);
  nvrUpdate(13,wheel_b_delta / 256);  
  oled.printHex(EEPROM[13]);
  oled.printHex(EEPROM[12]);
  oled.printChar('\n');
  nvrUpdate(14,wheel_b_center % 256);
  nvrUpdate(15,wheel_b_center / 256);  
  oled.printHex(EEPROM[15]);
  oled.printHex(EEPROM[14]);
  while(!(button2.update()&& button2.fell())) /*wait for keypress*/ ;
  return;
} 



void menu()
// User input of the parameters - usually called by double-klicking button 1.
// You step through the options with button 1, and set them with the right wheel or button 2. 
{
  oled.clearDisplay();
  oled.fontsize=1;
  oled.invert(true);
  print(m0);
  print(VERSION);
  oled.invert(false);
  oled.setCursor(0,1);
  print(t2);
// Crude but effective - 6 preformatted lines of text 
  print(m1);
  print(m2);
  print(m3);
  print(m4);
  print(m5);
  print(m6);
  int m= 10; // Variable keeps Where are we in the menu?
  boolean wheelMoved = false; 
  int wheelOld = readWheel(WHEEL_B,wheel_b_delta,false);
  for(;;)   // Loop until returning from routine
  {
    button1.update();
    button2.update();
    // Determine if wheel has moved; if yes, set flag
    int v = readWheel(WHEEL_B,wheel_b_delta,false);
    if (!wheelMoved)    // If wheel hasn't been moved yet
      wheelMoved = ((v - wheelOld) > 10) || ((wheelOld - v) > 10);
        
    // Stepping through the display positions and print the parameters
    // If parameter is selected, press 
    for(int i = 0;i < 11;i++)
    {
      int vv;                                   // Temporary value, either read from EEPROM or from wheel
      oled.setCursor(5+((i/5)* 8), (i % 5)+3);  // Step through two columns of five rows each
      oled.invert(i == m);                      // Display inverted if selected 
      if (i<5)                                  // Display/set CC for menu selections 0..4 
      {
        vv = ((i == m) && wheelMoved) ? v+1 : EEPROM.read(i+1);            // If selected: 
        if (button2.fell() && m == i) 
        {
          vv++;
          if (vv > 128) vv = 0;  
        }
        nvrUpdate(i+1,vv);        // This does nothing if value hasn't changed
        // Replace 0x80 with matching string, either "PWH" or "ATO"
        if (vv == 128) 
          print((i>1) ? m8 : m7) ;
        else
          oled.printByte(vv);          
      }  
      switch (i) {
        case 5: // MIDI Tx Channel
        {
          vv = ((i == m) && wheelMoved) ? (v+15)/8 : EEPROM.read(18);
          if (button2.fell() && m == i) 
          {
            vv++;
            if (vv > 16) vv = 1;  
          }
          nvrUpdate(18,vv);
          oled.printByte(vv);
        } 
        break;
        case 6: // MIDI THRU
        {
          vv = ((i == m) && wheelMoved) ? (v/64) : EEPROM.read(20);
          if (button2.fell() && m == i) 
            vv = !vv;
          nvrUpdate (20,vv);
          print(vv ? m9 : ma);
        }
        break;
        case 7: // ATO
        {
          vv = ((i == m) && wheelMoved) ? v : EEPROM.read(19);
          if (button2.fell() && m == i) 
          {
            vv++;
            if (vv > 127) vv = 0;  
          }
          nvrUpdate(19,vv);
          oled.printByte(vv);           
        }
        break;
        case 8: // Deadband, 0..63
        {
          vv = ((i == m) && wheelMoved) ? (v / 2) : EEPROM.read(21);
          if (button2.fell() && m == i) 
          {
            vv++;
            if (vv > 63) vv = 0;  
          }
          nvrUpdate(21,deadband = vv);
          oled.printByte(vv);           
        }
        break;
        case 9: // Calibrate routine
          oled.printChar('G');
          oled.printChar('O');
          if (button2.fell() && m == i)
          {
            calibrate();
            menu(); // recursive call to get menu back on
            return; // Leave routine here
          }
        break;
        case 10: // Put a litte X in the upper right corner and leave if selected
        {
          oled.setCursor(15,0);
          oled.printChar('X');
          if (button2.fell() && m == i) return;
        } 
      }        
    }
    // Button 1 steps through the options
    if (button1.fell())
    {
      if (++m > 10) m = 0;       
      wheelMoved = false;
      wheelOld = v; 
    }
  }
}

/********************************************** Additional MIDI - unused **********************************************/

// Callback Routines
void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  // UNUSED - doing this by hand as even with a callback routine, you'd still have to 
  // do a midiA.read() in the main loop. 

  // Try to keep your callbacks short (no delays ect)
  // otherwise it would slow down the loop() and have a bad impact
  // on real-time performance.
}

/********************************************************* Setup *****************************************************/
// The setup routine runs once when you press reset.
// It initializes communication with the display, ties the buttons to the bounce2 library, and displays a welcome screen.
// It sets up the MIDI interface and turns the MIDI library's MIDI Thru function off. 
// If EEPROM is valid, the offset values are recalled from NVR into variables, and operation commences. 
// If NVR EEPROM is corrupt, init values are written to it, and the calibrate routine is called.  

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
  delay(500);
  oled.clearDisplay();
  // EEPROM chksum OK? If no, initialize.
  if (nvrChksum() != EEPROM[0])
    nvrInit();
  
  // Read wheel offsets into global variables for speed
  wheel_a_delta = EEPROM[6]+256*EEPROM[7];
  wheel_a_center = EEPROM[8]+256*EEPROM[9];
  wheel_b_delta = EEPROM[12]+256*EEPROM[13];
  wheel_b_center = EEPROM[14]+256*EEPROM[15];
  deadband = EEPROM[21];
   
  // initialize the digital pin as an output w/ pullup.
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  button1.attach(BUTTON_A);
  button1.interval(10); // interval in ms
  button2.attach(BUTTON_B);
  button2.interval(10);
  button3.attach(BUTTON_C);
  button3.interval(10);

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

  workscreen();
}


/*****************************************************************************/
// The fun starts here - the simple main routine
/*****************************************************************************/

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
      oled.printChar(MidiRxCh < 10 ? ' ' : '1');
      oled.printChar(0x30 + MidiRxCh % 10);
    }
  }
  


  // Button A
  if (button1.update()  || RefreshFlag)
  {
    digitalWrite(LED2,HIGH);
    if (EEPROM[3] > 0x80) // Aftertouch?
    {
      midiA.sendAfterTouch(!button1.read()*EEPROM[19],MidiTxCh);
      usbMIDI.sendAfterTouch(!button1.read()*EEPROM[19],MidiTxCh);     
    } else {
      midiA.sendControlChange(EEPROM[3],!button1.read() * 127,MidiTxCh);
      usbMIDI.sendControlChange(EEPROM[3],!button1.read() * 127,MidiTxCh);
    }
    oled.fontsize=1;
    oled.printChar(button1.read() ? ' ' : '*',12,3);
    // double click code
    if (button1.fell())
    {
      if (millis()- doubleclick < DC)
      { // Double click detected!
        menu();
        workscreen();
      }        
      else
      // Single click, save time of event and restart detection    
        doubleclick = millis(); 
    } //if button1.fell     
  }
  
  // Button B
  if (button2.update()  || RefreshFlag)
  {
    digitalWrite(LED2,HIGH);
    if (EEPROM[4] > 0x80) // Aftertouch?
    {
      midiA.sendAfterTouch(!button2.read()*EEPROM[19],MidiTxCh);
      usbMIDI.sendAfterTouch(!button2.read()*EEPROM[19],MidiTxCh);     
    } else {
      midiA.sendControlChange(EEPROM[4],button2.read() ? 127 : 0,MidiTxCh);
      usbMIDI.sendControlChange(EEPROM[4],button2.read() ? 127 : 0,MidiTxCh);
    }
    oled.fontsize=1;
    oled.printChar(button2.read() ? ' ' : '*',12,5); 
  }
  
  // Button C (Footswitch)
  if (button3.update() || RefreshFlag)
  {
    digitalWrite(LED2,HIGH);
    if (EEPROM[5] > 0x80) // Aftertouch?
    {
      midiA.sendAfterTouch(!button3.read()*EEPROM[19],MidiTxCh);
      usbMIDI.sendAfterTouch(!button3.read()*EEPROM[19],MidiTxCh);     
    } else {
      midiA.sendControlChange(EEPROM[5],button3.read() ? 127 : 0,MidiTxCh);
      usbMIDI.sendControlChange(EEPROM[5],button3.read() ? 127 : 0,MidiTxCh);
    }
    oled.fontsize=1;
    oled.printChar(button3.read() ? ' ' : '*',12,7); 
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
    // Send MIDI CC
      midiA.sendControlChange(EEPROM[1] % 0x7f,value,MidiTxCh);
      usbMIDI.sendControlChange(EEPROM[1] % 0x7f,value,MidiTxCh);  
    }
    
  // Display code
    oled.fontsize = 2; 
    oled.setCursor(4,3);
    oled.printByte(value); 
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
    oled.printByte(value); 
  }
  
  //Clean up
  RefreshFlag = false;
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
}
