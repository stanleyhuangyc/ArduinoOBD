#ifndef LCD4Bit_mod_h
#define LCD4Bit_mod_h

#include <inttypes.h>

class LCD4Bit_mod {
public:
  LCD4Bit_mod(int num_lines);
  void commandWrite(int value);
  void init();
  void print(int value);
  void printIn(const char* value);
  void clear();
  //non-core---------------
  void cursorTo(int line_num, int x);
  void leftScroll(int chars, int delay_time);
  //end of non-core--------

  //4bit only, therefore ideally private but may be needed by user
  void commandWriteNibble(int nibble);
private:
  void pulseEnablePin();
  void pushNibble(int nibble);
  void pushByte(int value);
};

#endif
