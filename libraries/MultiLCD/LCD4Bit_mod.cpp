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

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "LCD4Bit_mod.h"

//command bytes for LCD
#define CMD_CLR 0x01
#define CMD_RIGHT 0x1C
#define CMD_LEFT 0x18
#define CMD_HOME 0x02

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
void LCD4Bit_mod::pushNibble(byte value)
{
  byte val_nibble= value & 0x0F;  //clean the value.  (unnecessary)

  for (byte i=DB[0]; i <= DB[3]; i++) {
    digitalWrite(i,val_nibble & 01);
    val_nibble >>= 1;
  }
  pulseEnablePin();
}

//push a byte of data through the LCD's DB4~7 pins, in two steps, clocking each with the enable pin.
void LCD4Bit_mod::pushByte(byte value)
{
  byte val_lower = value & 0x0F;
  byte val_upper = value >> 4;
  pushNibble(val_upper);
  pushNibble(val_lower);
}

void LCD4Bit_mod::commandWriteNibble(byte nibble)
{
  digitalWrite(RS, LOW);
  if (USING_RW) { digitalWrite(RW, LOW); }
  pushNibble(nibble);
}


void LCD4Bit_mod::commandWrite(byte value)
{
  digitalWrite(RS, LOW);
  if (USING_RW) { digitalWrite(RW, LOW); }
  pushByte(value);
  //TODO: perhaps better to add a delay after EVERY command, here.  many need a delay, apparently.
}




//print the given character at the current cursor position. overwrites, doesn't insert.
size_t LCD4Bit_mod::write(uint8_t c)
{
  //set the RS and RW pins to show we're writing data
  digitalWrite(RS, HIGH);
  if (USING_RW) { digitalWrite(RW, LOW); }

  //let pushByte worry about the intricacies of Enable, nibble order.
  pushByte(c);

  return 1;
}


//send the clear screen command to the LCD
void LCD4Bit_mod::clear()
{
  commandWrite(CMD_CLR);
  delay(1);
}


// initiatize lcd after a short pause
//while there are hard-coded details here of lines, cursor and blink settings, you can override these original settings after calling .init()
void LCD4Bit_mod::begin ()
{
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

  byte num_lines_ptn = (num_lines - 1) << 3;
  byte dot_format_ptn = 0x00;      //5x7 dots.  0x04 is 5x10

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
void LCD4Bit_mod::setCursor(byte x, byte line)
{
  //first, put cursor home
  commandWrite(CMD_HOME);
  //offset 40 chars in if second line requested
  x += line * 40;

  //advance the cursor to the right according to position. (second line starts at position 40).
  for (byte i=0; i<x; i++) {
    commandWrite(0x14);
  }
}

//scroll whole display to left
void LCD4Bit_mod::leftScroll(byte num_chars, unsigned int delay_time)
{
  for (byte i=0; i<num_chars; i++) {
    commandWrite(CMD_LEFT);
    delay(delay_time);
  }
}

//Improvements ------------------------------------------------
//Remove the unnecessary delays (e.g. from the end of pulseEnablePin()).
//Allow the user to pass the pins to be used by the LCD in the constructor, and store them as member variables of the class instance.
//-------------------------------------------------------------
