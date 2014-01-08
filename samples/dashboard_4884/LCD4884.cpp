/*
Modified by Lauren
version 0.3

Any suggestions are welcome.

Editors    : Lauren from DFRobot <Lauran.pan@gmail.com>
             Stanley Huang <stanleyhuangyc@gmail.com>
Date       : Feb. 11, 2012

* Added LCD_putchar for basic console display
* Have the back light under control.
* Update the library and sketch to compatible with IDE V1.0 and earlier

*/

#include "LCD4884.h"
#include "font_6x8.h"
#include "font_big.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "WConstants.h"
#endif


extern "C" 
{
#include <avr/pgmspace.h>
}

#define NUM_COL 14
#define NUM_ROW 5

LCD4884::LCD4884():prev_char(0),char_mode(MENU_NORMAL)
{};

LCD4884 lcd = LCD4884();

void LCD4884::backlight(unsigned char dat)
{
	if(dat==1)
		digitalWrite(LCD_BL,HIGH);
	else
		digitalWrite(LCD_BL,LOW);
}

void LCD4884::LCD_init(void)
{
	for(unsigned char i = 2; i < 8; i++) {
		pinMode(i,OUTPUT);
		digitalWrite(i,LOW);
	}

	digitalWrite(LCD_RST,LOW);
	delayMicroseconds(1);
	digitalWrite(LCD_RST,HIGH);
	
	digitalWrite(SPI_CS,LOW);
	delayMicroseconds(1);
	digitalWrite(SPI_CS,HIGH);
	delayMicroseconds(1);
	digitalWrite(LCD_BL,HIGH);

	LCD_write_byte(0x21, 0);
	LCD_write_byte(0xc0, 0);
	LCD_write_byte(0x06, 0);
	LCD_write_byte(0x13, 0);
	LCD_write_byte(0x20, 0);
	LCD_clear();
	LCD_write_byte(0x0c, 0);
	
	digitalWrite(SPI_CS,LOW);
}
  
void LCD4884::LCD_write_byte(unsigned char dat, unsigned char dat_type)
{
	digitalWrite(SPI_CS,LOW);
    if (dat_type == 0)
		digitalWrite(LCD_DC,LOW);
    else
		digitalWrite(LCD_DC,HIGH);

	for(unsigned char i=0; i<8; i++) { 
		if(dat & 0x80) {
			digitalWrite(SPI_MOSI,HIGH);
		} else {
			digitalWrite(SPI_MOSI,LOW);
		} 
		digitalWrite(SPI_SCK,LOW);
		dat = dat << 1; 
		digitalWrite(SPI_SCK,HIGH);
	} 
	digitalWrite(SPI_CS,HIGH);
}

void LCD4884::LCD_draw_bmp_pixel(unsigned char X,unsigned char Y,unsigned char *map,
                  unsigned char Pix_x,unsigned char Pix_y)
{
    unsigned int i,n;
    unsigned char row;
    
    if (Pix_y%8==0)
		row=Pix_y/8;
	else
        row=Pix_y/8+1;
    
    for (n=0;n<row;n++) {
		LCD_set_XY(X,Y);
		for(i=0; i<Pix_x; i++) {
			LCD_write_byte(map[i+n*Pix_x], 1);
		}
        Y++;                       
	}      
}

void LCD4884::LCD_write_string(unsigned char X,unsigned char Y,char *s, char mode)
  {
    LCD_set_XY(X,Y);
    while (*s) {
		LCD_write_char(*s, mode);
		s++;
	}
  }

void LCD4884::LCD_write_string(char *s, char mode)
  {
    while (*s) {
		LCD_write_char(*s, mode);
		s++;
	}
  }

void LCD4884::LCD_write_chinese(unsigned char X, unsigned char Y,unsigned char *c,unsigned char ch_with,unsigned char num,unsigned char line,unsigned char row) 
{ 
    LCD_set_XY(X,Y);                             
    for (unsigned char i=0;i<num;) { 
      for (unsigned char n=0; n<ch_with*2; n++) {  
          if (n==ch_with) { 
              if (i==0)
				  LCD_set_XY(X,Y+1);
              else
                  LCD_set_XY((X+(ch_with+row)*i),Y+1); 
          } 
          LCD_write_byte(c[(i*ch_with*2)+n],1); 
        } 
      i++; 
      LCD_set_XY((X+(ch_with+row)*i),Y); 
    } 
}


void LCD4884::LCD_write_string_big ( unsigned char X,unsigned char Y, char *string, char mode )
{
    while ( *string ){
        LCD_write_char_big( X, Y, *string , mode );
        if(*string++ == '.')
          X += 5;
        else
          X += 12;
    }
}

/* write char in big font */ 
void LCD4884::LCD_write_char_big (unsigned char X,unsigned char Y, unsigned char ch, char mode)
{
   unsigned char i, j;
   unsigned char *pFont;
   unsigned char ch_dat;
   
   pFont = (unsigned char *) big_number;
   
   switch (ch) {
   case '.':
    ch = 10;
	break;
   case '+':
    ch = 11;
	break;
   case '-':
    ch = 12;
	break;
   case ' ':
	   for(i=0;i<3;i++) {	
		 LCD_set_XY ( X, Y+i);
		 for(j=0; j<16; j++) {
		   LCD_write_byte( (mode == MENU_NORMAL)? 0 : 0xff, 1);
		 }
	   }
	   return;
	break;
   default:
    ch = ch & 0x0f;
   }
	
   for(i=0;i<3;i++) {	
	 LCD_set_XY ( X, Y+i);
     for(j=0; j<16; j++){
       ch_dat =  pgm_read_byte(pFont+ch*48 + i*16 +j);
       LCD_write_byte( (mode == MENU_NORMAL)? ch_dat : (ch_dat^0xff), 1);
     }
   } 
   
  
}
  
void LCD4884::LCD_write_char(unsigned char c, char mode)
{
    unsigned char line;
    unsigned char *pFont;
    byte ch;
    
    pFont = (unsigned char *)font6_8;
    c -= 32;

    for (line=0; line<6; line++) {
		ch = pgm_read_byte(pFont+c*6+line);
		LCD_write_byte( (mode==MENU_NORMAL)? ch: (ch^ 0xff) , 1);
    }
	 x = (x + 1) % NUM_COL;
}

  
void LCD4884::LCD_set_XY(unsigned char X, unsigned char Y)
  {
    LCD_write_byte(0x40 | Y, 0);		// column
    LCD_write_byte(0x80 | X, 0);         // row
	x = X;
  }


void LCD4884::LCD_clear(void)
  {
    unsigned int i;

    LCD_write_byte(0x0c, 0);			
    LCD_write_byte(0x80, 0);			

    for (i=0; i<504; i++)
    LCD_write_byte(0, 1);

	x = 0;
  }

void LCD4884::LCD_write_title(char* title)
{
	LCD_set_XY(0, 0);
	for (char n = 0; n < NUM_COL; n++)
       LCD_write_char(' ', MENU_HIGHLIGHT);
	LCD_write_string((NUM_COL - strlen(title)) * 3, 0, title, MENU_HIGHLIGHT);
	LCD_set_XY(0, 1);
}

void LCD4884::LCD_putchar(char c)
{
	if (prev_char == 27) {
		switch (c) {
		case '0':
			LCD_clear();
			break;
		case '1':
			LCD_set_XY(0, 0);
			break;
		case '2':
			char_mode = MENU_NORMAL;
			break;
		case '3':
			char_mode = MENU_HIGHLIGHT;
			break;
		case '4':
			backlight(ON);
			break;
		case '5':
			backlight(OFF);
			break;
		}
	} else {
		switch (c) {
		case '\r':
			for (char n = x; n < NUM_COL; n++) {
				LCD_write_char(' ', char_mode);
			}
			break;
		case '\n':
			break;
		default:
			LCD_write_char(c, char_mode);
		}
	}
	prev_char = c;
}