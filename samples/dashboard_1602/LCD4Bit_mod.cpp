/*
LCD4Bit v0.1 16/Oct/2006 neillzero http://abstractplain.net

What is this?
An arduino library for comms with HD44780-compatible LCD, in 4-bit mode (saves pins)

Sources:
- The original "LiquidCrystal" 8-bit library and tutorial
    http://www.arduino.cc/en/uploads/Tutorial/LiquidCrystal.zip
    http://www.arduino.cc/en/Tutorial/LCDLibrary
- DEM 16216 datasheet http://www.maplin.co.uk/Media/PDFs/N27AZ.pdf
- Massimo's suggested 4-bit code (I took initialization from here) http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1144924220/8
See also:
- glasspusher's code (probably more correct): http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1160586800/0#0

Tested only with a DEM 16216 (maplin "N27AZ" - http://www.maplin.co.uk/Search.aspx?criteria=N27AZ)
If you use this successfully, consider feeding back to the arduino wiki with a note of which LCD it worked on.

Usage:
see the examples folder of this library distribution.

*/

#include "LCD4Bit_mod.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//command bytes for LCD
#define CMD_CLR 0x01
#define CMD_RIGHT 0x1C
#define CMD_LEFT 0x18
#define CMD_HOME 0x02

// --------- PINS -------------------------------------
//is the RW pin of the LCD under our control?  If we're only ever going to write to the LCD, we can use one less microcontroller pin, and just tie the LCD pin to the necessary signal, high or low.
//this stops us sending signals to the RW pin if it isn't being used.
int USING_RW = false;

//RS, RW and Enable can be set to whatever you like
int RS = 8;
int RW = 11;
int Enable = 9;
//DB should be an unseparated group of pins  - because of lazy coding in pushNibble()
int DB[] = {4, 5, 6, 7};  //wire these to DB4~7 on LCD.

//--------------------------------------------------------

//how many lines has the LCD? (don't change here - specify on calling constructor)
int g_num_lines = 2;

//pulse the Enable pin high (for a microsecond).
//This clocks whatever command or data is in DB4~7 into the LCD controller.
void LCD4Bit_mod::pulseEnablePin(){
  digitalWrite(Enable,LOW);
  delayMicroseconds(1);
  // send a pulse to enable
  digitalWrite(Enable,HIGH);
  delayMicroseconds(1);
  digitalWrite(Enable,LOW);
  delay(1);  // pause 1 ms.  TODO: what delay, if any, is necessary here?
}

//push a nibble of data through the the LCD's DB4~7 pins, clocking with the Enable pin.
//We don't care what RS and RW are, here.
void LCD4Bit_mod::pushNibble(int value){
  int val_nibble= value & 0x0F;  //clean the value.  (unnecessary)

  for (int i=DB[0]; i <= DB[3]; i++) {
    digitalWrite(i,val_nibble & 01);
    val_nibble >>= 1;
  }
  pulseEnablePin();
}

//push a byte of data through the LCD's DB4~7 pins, in two steps, clocking each with the enable pin.
void LCD4Bit_mod::pushByte(int value){
  int val_lower = value & 0x0F;
  int val_upper = value >> 4;
  pushNibble(val_upper);
  pushNibble(val_lower);
}


//stuff the library user might call---------------------------------
//constructor.  num_lines must be 1 or 2, currently.
LCD4Bit_mod::LCD4Bit_mod (int num_lines) {
  g_num_lines = num_lines;
  if (g_num_lines < 1 || g_num_lines > 2)
  {
    g_num_lines = 1;
  }
}

void LCD4Bit_mod::commandWriteNibble(int nibble) {
  digitalWrite(RS, LOW);
  if (USING_RW) { digitalWrite(RW, LOW); }
  pushNibble(nibble);
}


void LCD4Bit_mod::commandWrite(int value) {
  digitalWrite(RS, LOW);
  if (USING_RW) { digitalWrite(RW, LOW); }
  pushByte(value);
  //TODO: perhaps better to add a delay after EVERY command, here.  many need a delay, apparently.
}




//print the given character at the current cursor position. overwrites, doesn't insert.
void LCD4Bit_mod::print(int value) {
  //set the RS and RW pins to show we're writing data
  digitalWrite(RS, HIGH);
  if (USING_RW) { digitalWrite(RW, LOW); }

  //let pushByte worry about the intricacies of Enable, nibble order.
  pushByte(value);
}


//print the given string to the LCD at the current cursor position.  overwrites, doesn't insert.
//While I don't understand why this was named printIn (PRINT IN?) in the original LiquidCrystal library, I've preserved it here to maintain the interchangeability of the two libraries.
void LCD4Bit_mod::printIn(const char* msg) {
  uint8_t i;  //fancy int.  avoids compiler warning when comparing i with strlen()'s uint8_t
  uint8_t l = strlen(msg);
  for (i=0; i < l; i++){
    if (msg[i] >= 20) print(msg[i]);
  }
}


//send the clear screen command to the LCD
void LCD4Bit_mod::clear(){
  commandWrite(CMD_CLR);
  delay(1);
}


// initiatize lcd after a short pause
//while there are hard-coded details here of lines, cursor and blink settings, you can override these original settings after calling .init()
void LCD4Bit_mod::init () {
  pinMode(Enable,OUTPUT);
  pinMode(RS,OUTPUT);
  if (USING_RW) { pinMode(RW,OUTPUT); }
  pinMode(DB[0],OUTPUT);
  pinMode(DB[1],OUTPUT);
  pinMode(DB[2],OUTPUT);
  pinMode(DB[3],OUTPUT);

  delay(50);

  //The first 4 nibbles and timings are not in my DEM16217 SYH datasheet, but apparently are HD44780 standard...
  commandWriteNibble(0x03);
  delay(5);
  commandWriteNibble(0x03);
  delayMicroseconds(100);
  commandWriteNibble(0x03);
  delay(5);

  // needed by the LCDs controller
  //this being 2 sets up 4-bit mode.
  commandWriteNibble(0x02);
  commandWriteNibble(0x02);
  //todo: make configurable by the user of this library.
  //NFXX where
  //N = num lines (0=1 line or 1=2 lines).
  //F= format (number of dots (0=5x7 or 1=5x10)).
  //X=don't care

  int num_lines_ptn = (g_num_lines - 1) << 3;
  int dot_format_ptn = 0x00;      //5x7 dots.  0x04 is 5x10

  commandWriteNibble(num_lines_ptn | dot_format_ptn);
  delayMicroseconds(60);

  //The rest of the init is not specific to 4-bit mode.
  //NOTE: we're writing full bytes now, not nibbles.

  // display control:
  // turn display on, cursor off, no blinking
  commandWrite(0x0C);
  delayMicroseconds(60);

  //clear display
  commandWrite(0x01);
  delay(3);

  // entry mode set: 06
  // increment automatically, display shift, entire shift off
  commandWrite(0x06);

  delay(1);//TODO: remove unnecessary delays
}


//non-core stuff --------------------------------------
//move the cursor to the given absolute position.  line numbers start at 1.
//if this is not a 2-line LCD4Bit_mod instance, will always position on first line.
void LCD4Bit_mod::cursorTo(int line_num, int x){
  //first, put cursor home
  commandWrite(CMD_HOME);

  //if we are on a 1-line display, set line_num to 1st line, regardless of given
  if (g_num_lines==1){
    line_num = 1;
  }
  //offset 40 chars in if second line requested
  if (line_num == 2){
    x += 40;
  }
  //advance the cursor to the right according to position. (second line starts at position 40).
  for (int i=0; i<x; i++) {
    commandWrite(0x14);
  }
}

//scroll whole display to left
void LCD4Bit_mod::leftScroll(int num_chars, int delay_time){
  for (int i=0; i<num_chars; i++) {
    commandWrite(CMD_LEFT);
    delay(delay_time);
  }
}

//Improvements ------------------------------------------------
//Remove the unnecessary delays (e.g. from the end of pulseEnablePin()).
//Allow the user to pass the pins to be used by the LCD in the constructor, and store them as member variables of the class instance.
//-------------------------------------------------------------
