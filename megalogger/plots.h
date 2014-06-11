#ifndef PLOTS_H
#define PLOTS_H

#include <Arduino.h>

class Point {
public:
	int16_t x;
	int16_t y;
	Point (int16_t x, int16_t y) : x(x), y(y) { }
	Point() : x(0), y(0) { }
};


class BarPlot {
	private: 

		int16_t maxValue;
		LCD_ILI9325D* lcd;
		int16_t positionX, positionY;
		int16_t prevValue;
		int16_t prevY;
		int16_t redAreaValue;
		int16_t redY;	
		char unitName[6];	

		uint16_t mapping(int16_t x, int16_t y, int16_t prevValue, int16_t actValue);
		void changeVal(int16_t val);

	public:
		//lcd should be initialized
		//positionX - X coordinate where top-left corner of plot should be placed
		//positionY - Y coordinate where top-left corner of plit should be placed
		//maxValue - maximum value which should be able to display on screen
		//redAreaValue - smallest value when plot should be collered red
		//title - title of plot to display
		//unitName - name of unit displaying with plot
		void init(LCD_ILI9325D* lcd, int16_t positionX, int16_t positionY, int16_t maxValue, int16_t redAreaValue, char title[10], char unitName[5]);
		void init(LCD_ILI9325D* lcd, int16_t positionX, int16_t positionY, int16_t maxValue, int16_t redAreaValue, char unitName[5]);
		//val - value which should be displayed on plot
		void displayValue(int16_t val);
};

class CirclePlot {
	private: 

		LCD_ILI9325D* lcd;
		static const int16_t x0 = 70, y0 = 74;//center of circle of plot
		static const int16_t rS = 55, rL = 65;//short and long radius of scale
		int16_t maxValue;
		int16_t positionX, positionY;
		int16_t prevValue;
		float prevAngle;
		static const float a = 0.01286, b = 2.42; //factor and constant in linear equation mapping [0, 255] to angle
        Point prevPoints[2];
        char unitName[6];

		bool ccw(Point a, Point b, Point c);
		int16_t inline myMax(int16_t a, int16_t b, int16_t c, int16_t d);
		int16_t inline myMin(int16_t a, int16_t b, int16_t c, int16_t d);
		uint16_t mapping(int16_t x, int16_t y, Point cmpPointHigher, Point cmpPointLower, int16_t prevValue, int16_t actValue);
		void changeVal(int16_t val);

	public:
		//lcd should be initialized
		//positionX - X coordinate where top-left corner of plot should be placed
		//positionY - Y coordinate where top-left corner of plit should be placed
		//maxValue - maximum value which should be able to display on screen
		//title - title of plot to display
		//unitName - name of unit displaying with plot
		void init(LCD_ILI9325D* lcd, int16_t positionX, int16_t positionY, int16_t maxValue, char title[10], char unitName[5]);
		void init(LCD_ILI9325D* lcd, int16_t positionX, int16_t positionY, int16_t maxValue, char unitName[5]);
		//val - value which should be displayed on plot
		void displayValue(int16_t val);
};

class GPlot {
	private: 

		LCD_ILI9325D* lcd;
		int16_t maxValue;
		uint16_t positionX, positionY;
		int16_t prevValueX, prevValueY;
		int16_t redAreaValue;
		bool invertX;
		bool invertY;
		char unitName[6];	

		uint16_t mapping(int16_t val, int16_t prevValue, int16_t actValue);
		void changeVal(int16_t valX, int16_t valY);
		void changeValOneAxle(int16_t val, int16_t prev, int16_t act);
		void drawRectangleMapping(uint16_t x, uint16_t width, uint16_t y, uint16_t height, int16_t prev, int16_t act, int16_t ref, bool useWidth);

	public:
		//lcd should be initialized
		//positionX - X coordinate where top-left corner of plot should be placed
		//positionY - Y coordinate where top-left corner of plit should be placed
		//maxValue - maximum value which should be able to display on screen
		//redAreaValue - smallest value when plot should be collered red
		//unitName - name of unit displaying with plot
		void init(LCD_ILI9325D* lcd, int16_t positionX, int16_t positionY, int16_t maxValue, int16_t redAreaValue, bool invertX, bool invertY, char unitName[5]);
		//valX - value which should be displayed on plot in X axle
		//valY - value which should be displayed on plot in Y axle
		void displayValue(int16_t valX, int16_t valY);
};

#endif
