/*************************************************************************
* Arduino Text & Bitmap Display Library for color LCDs
* Distributed under GPL v2.0
* Developed by Stanley Huang <stanleyhuangyc@gmail.com>
* For more information, please visit http://arduinodev.com
*************************************************************************/

#include <UTFT.h>

#if defined(__AVR_ATmega168P__)
#define MEMORY_SAVING
#endif

#ifdef __arm__
#define PROGMEM
#endif

typedef enum {
    FONT_SIZE_SMALL = 0,
    FONT_SIZE_MEDIUM,
    FONT_SIZE_LARGE,
    FONT_SIZE_XLARGE
} FONT_SIZE;

#define FLAG_PAD_ZERO 1
#define FLAG_PIXEL_DOUBLE_H 2
#define FLAG_PIXEL_DOUBLE_V 4
#define FLAG_PIXEL_DOUBLE (FLAG_PIXEL_DOUBLE_H | FLAG_PIXEL_DOUBLE_V)

#define RGB16(r,g,b) (((uint16_t)(r >> 3) << 11) | ((uint16_t)(g >> 2) << 5) | (b >> 3))

#define RGB16_RED 0xF800
#define RGB16_GREEN 0x7E0
#define RGB16_BLUE 0x1F
#define RGB16_YELLOW 0xFFE0
#define RGB16_CYAN 0x7FF
#define RGB16_PINK 0xF81F
#define RGB16_WHITE 0xFFFF


extern const PROGMEM unsigned char font5x8[][5];
extern const PROGMEM unsigned char digits8x8[][8] ;
extern const PROGMEM unsigned char digits16x16[][32];
extern const PROGMEM unsigned char digits16x24[][48];
extern const PROGMEM unsigned char font8x16_doslike[][16];
extern const PROGMEM unsigned char font8x16_terminal[][16];

class LCD_Common
{
public:
    LCD_Common():m_font(FONT_SIZE_SMALL),m_flags(0) {}
    void setFontSize(FONT_SIZE size) { m_font = size; }
    void setFlags(byte flags) { m_flags = flags; }
    virtual void setBackLight(byte brightness) {}
    virtual void draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height) {}
    virtual void draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height, byte scaleX, byte scaleY = 0) {}
    virtual void draw4bpp(const PROGMEM byte* buffer, uint16_t width, uint16_t height) {}
    virtual size_t write(uint8_t c) { return 0; }
    virtual byte getLines() { return 0; }
    virtual byte getCols() { return 0; }
    virtual void clearLine(byte line) {}
    virtual void clear() {}
    virtual void begin() {}
    virtual void setCursor(byte column, byte line) {}
    void printInt(uint16_t value, int8_t padding = -1);
    void printLong(uint32_t value, int8_t padding = -1);
    void printSpace(byte n)
    {
        for (byte m = 0; m < n; m++) write(' ');
    }
protected:
    virtual void writeDigit(byte n) {}
    byte m_font;
    byte m_flags;
    uint16_t m_x;
    uint16_t m_y;
};

#define TFT_LINE_HEIGHT 8

class LCD_ILI9325D : public LCD_Common, public Print
{
public:
    LCD_ILI9325D() { m_font = FONT_SIZE_MEDIUM; }
    void setCursor(uint16_t column, uint8_t line)
    {
        m_y = column;
        m_x = (uint16_t)line * TFT_LINE_HEIGHT;
    }
    void setXY(uint16_t x, uint16_t y)
    {
        m_y = x;
        m_x = y;
    }
    void setColor(uint16_t color)
    {
        m_color[1] = color;
    }
    void setColor(uint8_t R, uint8_t G, uint8_t B)
    {
        m_color[1] = ((uint16_t)R << 11) | ((uint16_t)G << 5) | B;
    }
    void setBackColor(uint16_t color)
    {
        m_color[0] = color;
    }
    void setBackColor(uint8_t R, uint8_t G, uint8_t B)
    {
        m_color[0] = ((uint16_t)R << 11) | ((uint16_t)G << 5) | B;
    }
    void begin();
    void clear(uint16_t x = 0, uint16_t y = 0, uint16_t width = 320, uint16_t height = 240);
    void draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height);
    void draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height, byte scaleX, byte scaleY = 0);
    void draw4bpp(const PROGMEM byte* buffer, uint16_t width, uint16_t height);
    size_t write(uint8_t);
    void clearLine(byte line)
    {
        clear(0, line * TFT_LINE_HEIGHT, 320, 8);
    }
    byte getLines() { return 53; }
    byte getCols() { return 30; }
private:
    void setXY(uint16_t x0,uint16_t x1,uint16_t y0,uint16_t y1);
    void writeDigit(byte n);
    void clearPixels(uint16_t pixels);
    void WriteData(uint16_t c);
    void WriteData(byte l, byte h);
    void WriteCommandData(uint16_t cmd,uint16_t dat);
    void Enable();
    void Disable();
    void SetCommandMode();
    void SetDataMode();
    uint16_t m_x;
    uint16_t m_y;
    uint16_t m_color[2];
    byte lastData;
};

class LCD_ILI9341 : public LCD_Common, public Print
{
public:
    LCD_ILI9341() { m_font = FONT_SIZE_MEDIUM; }
    void setCursor(uint16_t column, uint8_t line)
    {
        m_x = column;
        m_y = (uint16_t)line * TFT_LINE_HEIGHT;
    }
    void setXY(uint16_t x, uint16_t y)
    {
        m_x = x;
        m_y = y;
    }
    void setColor(uint16_t color)
    {
        m_color[1][0] = color & 0xff;
        m_color[1][1] = color >> 8;
    }
    void setColor(uint8_t R, uint8_t G, uint8_t B)
    {
        uint16_t color = ((uint16_t)R << 11) | ((uint16_t)G << 5) | B;
        m_color[1][0] = color & 0xff;
        m_color[1][1] = color >> 8;
    }
    void setBackColor(uint16_t color)
    {
        m_color[0][0] = color & 0xff;
        m_color[0][1] = color >> 8;
    }
    void setBackColor(uint8_t R, uint8_t G, uint8_t B)
    {
        uint16_t color = ((uint16_t)R << 11) | ((uint16_t)G << 5) | B;
        m_color[0][0] = color & 0xff;
        m_color[0][1] = color >> 8;
    }
    void clearLine(byte line)
    {
        fill(0, line * TFT_LINE_HEIGHT, 320, 8);
    }
    void begin (void);
    void setPixel(uint16_t poX, uint16_t poY,uint16_t color);
    void fill(uint16_t XL,uint16_t XR,uint16_t YU,uint16_t YD,uint16_t color = 0);
    void clear(void);
    size_t write(uint8_t);
    void setBackLight(byte brightness);
    void draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height);
    void draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height, byte scaleX, byte scaleY = 0);
private:
    void setXY(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1);
    void sendPixelData(byte d);
    void writeDigit(byte n);
    void clearPixels(uint16_t pixels);
    void setCol(uint16_t StartCol,uint16_t EndCol);
    void setPage(uint16_t StartPage,uint16_t EndPage);
    void sendCMD(uint8_t index);
    void WRITE_Package(uint16_t *data,uint8_t howmany);
    void WRITE_DATA(uint8_t data);
    void sendData(uint16_t data);
    uint8_t Read_Register(uint8_t Addr,uint8_t xParameter);
    uint8_t readID(void);
    uint8_t m_color[2][2];
};

class LCD_SSD1289 : public UTFT, public LCD_Common, public Print
{
public:
    LCD_SSD1289()
    {
        m_font = FONT_SIZE_MEDIUM;
        disp_x_size = 239;
        disp_y_size = 319;
	 orient = LANDSCAPE;
        display_transfer_mode = 16;
        display_model = ITDB32S;
        __p1 = 38;
        __p2 = 39;
        __p3 = 40;
        __p4 = 41;
        __p5 = 0;

		P_RS	= portOutputRegister(digitalPinToPort(38));
		B_RS	= digitalPinToBitMask(38);
		P_WR	= portOutputRegister(digitalPinToPort(39));
		B_WR	= digitalPinToBitMask(39);
		P_CS	= portOutputRegister(digitalPinToPort(40));
		B_CS	= digitalPinToBitMask(40);
		P_RST	= portOutputRegister(digitalPinToPort(41));
		B_RST	= digitalPinToBitMask(41);
    }
    void setCursor(uint16_t column, uint8_t line)
    {
        m_x = column;
        m_y = (uint16_t)line * TFT_LINE_HEIGHT;
    }
    void setXY(uint16_t x, uint16_t y)
    {
        m_x = x;
        m_y = y;
    }
    void begin();
    void clear(uint16_t x = 0, uint16_t y = 0, uint16_t width = 319, uint16_t height = 239);
    void draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height);
    void draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height, byte scaleX, byte scaleY = 0);
    void draw4bpp(const PROGMEM byte* buffer, uint16_t width, uint16_t height);
    size_t write(uint8_t);
    void clearLine(byte line)
    {
        clear(0, line * TFT_LINE_HEIGHT, disp_y_size, 8);
    }
    void setBackLight(byte brightness);
private:
    void setXY(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    void writeDigit(byte n);
    void clearPixels(uint32_t pixels);
    void Enable();
    void Disable();
};

class LCD_R61581 : public UTFT, public LCD_Common, public Print
{
public:
    LCD_R61581()
    {
        m_font = FONT_SIZE_MEDIUM;
        disp_x_size = 319;
        disp_y_size = 479;
	 orient = LANDSCAPE;
        display_transfer_mode = 16;
        display_model = CTE35IPS;
        __p1 = 38;
        __p2 = 39;
        __p3 = 40;
        __p4 = 41;
        __p5 = 0;

		P_RS	= portOutputRegister(digitalPinToPort(38));
		B_RS	= digitalPinToBitMask(38);
		P_WR	= portOutputRegister(digitalPinToPort(39));
		B_WR	= digitalPinToBitMask(39);
		P_CS	= portOutputRegister(digitalPinToPort(40));
		B_CS	= digitalPinToBitMask(40);
		P_RST	= portOutputRegister(digitalPinToPort(41));
		B_RST	= digitalPinToBitMask(41);
    }
    void setCursor(uint16_t column, uint8_t line)
    {
        m_x = column;
        m_y = (uint16_t)line * TFT_LINE_HEIGHT;
    }
    void setXY(uint16_t x, uint16_t y)
    {
        m_x = x;
        m_y = y;
    }
    void begin();
    void clear(uint16_t x = 0, uint16_t y = 0, uint16_t width = 479, uint16_t height = 319);
    void draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height);
    void draw(const PROGMEM byte* buffer, uint16_t width, uint16_t height, byte scaleX, byte scaleY = 0);
    void draw4bpp(const PROGMEM byte* buffer, uint16_t width, uint16_t height);
    size_t write(uint8_t);
    void clearLine(byte line)
    {
        clear(0, line * TFT_LINE_HEIGHT, disp_y_size, 8);
    }
    void setBackLight(byte brightness);
private:
    void setXY(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    void writeDigit(byte n);
    void clearPixels(uint32_t pixels);
    void Enable();
    void Disable();
};

