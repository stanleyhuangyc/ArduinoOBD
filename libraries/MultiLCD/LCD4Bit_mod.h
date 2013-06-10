#ifndef LCD4Bit_mod_h
#define LCD4Bit_mod_h

#include <inttypes.h>

class LCD4Bit_mod : public Print {
public:
    LCD4Bit_mod(byte num_lines = 2):USING_RW(false),RS(8),RW(11),Enable(9)
    {
      DB[0] = 4;
      DB[1] = 5;
      DB[2] = 6;
      DB[3] = 7;
      this->num_lines = num_lines;
    }
    void commandWrite(byte value);
    void begin();
    size_t write(uint8_t c);
    void clear();
    //non-core---------------
    void setCursor(byte x, byte line);
    void leftScroll(byte chars, unsigned int delay_time);
    //end of non-core--------

    //4bit only, therefore ideally private but may be needed by user
    void commandWriteNibble(byte nibble);
private:
    void pulseEnablePin();
    void pushNibble(byte nibble);
    void pushByte(byte value);
    // --------- PINS -------------------------------------
    //is the RW pin of the LCD under our control?  If we're only ever going to write to the LCD, we can use one less microcontroller pin, and just tie the LCD pin to the necessary signal, high or low.
    //this stops us sending signals to the RW pin if it isn't being used.
    bool USING_RW;
    //RS, RW and Enable can be set to whatever you like
    byte RS;
    byte RW;
    byte Enable;
    //DB should be an unseparated group of pins  - because of lazy coding in pushNibble()
    byte DB[4];  //wire these to DB4~7 on LCD.
    //how many lines has the LCD? (don't change here - specify on calling constructor)
    int num_lines;

};

#endif
