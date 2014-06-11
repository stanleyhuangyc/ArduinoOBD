#include <MultiLCD.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include "images.h"
#include "plots.h"

void BarPlot::init(LCD_ILI9325D* lcdArg, int16_t positionXArg, int16_t positionYArg, int16_t maxValueArg, int16_t redAreaValueArg, char title[10], char unitNameArg[5]){
	lcd = lcdArg;
	positionX = positionXArg;
	positionY = positionYArg;
	maxValue = maxValueArg;

	for (int i=0; i<5; i++){
		unitName[i] = unitNameArg[i];
	}
	unitName[5] = 0;

	char titleToWrite[11];
	for (int i=0; i<10; i++){
		titleToWrite[i] = title[i];
	}
	titleToWrite[10]=0;

	lcd->setColor(RGB16_WHITE);
	lcd->setXY(positionX, positionY);
	lcd->draw(barPlotBase, 96, 120);

	prevValue = 0;
	prevY = 109;
	redY = 109 - ((int32_t)redAreaValueArg * (109 - 9 + 1))/maxValue;

	lcd->setFontSize(FONT_SIZE_SMALL);
    lcd->setCursor(positionX+30, (positionY+104)/8);
	lcd->print(titleToWrite);

	lcd->setFontSize(FONT_SIZE_SMALL);
    lcd->setCursor(positionX+60, (positionY+72)/8);
	lcd->print(unitNameArg);
}

void BarPlot::init(LCD_ILI9325D* lcdArg, int16_t positionXArg, int16_t positionYArg, int16_t maxValueArg, int16_t redAreaValueArg, char unitNameArg[5]){
	init(lcdArg, positionXArg, positionYArg, maxValueArg, redAreaValueArg, "", unitNameArg);
}

void BarPlot::changeVal(int16_t val){

	if (val==0) return;
	int16_t actValue = prevValue + val;
	int16_t upY, downY;
	if (val>0){
		upY = prevY - val;
		downY = prevY;
		prevY = upY;
	} else {
		upY = prevY;
		downY = prevY - val;
		prevY = downY;
	}

    lcd->setXY(positionY+upY, positionY+downY, positionX+9, positionX+25 - 1);

	for (int16_t x=25-9+1; x>=0; x--){
    	for (int16_t y=0; y<downY-upY+1; y++){
			lcd->WriteData(mapping(x + 9, y + upY, prevValue, actValue));
    	}
    }

    prevValue = actValue;
}

void BarPlot::displayValue(int16_t valArg){
	if (valArg > maxValue) valArg = maxValue;
	if (valArg < 0) valArg = 0;

	int32_t val = ((int32_t)valArg * (109 - 9 + 1))/maxValue;

	changeVal((int16_t)val - prevValue);

    lcd->setFontSize(FONT_SIZE_XLARGE);
    lcd->setCursor(positionX+30, (positionY+40)/8);
    lcd->printInt(valArg, 3);
}

uint16_t BarPlot::mapping(int16_t x, int16_t y, int16_t prevValue, int16_t actValue){
	if (actValue < prevValue) return 0x0;
	if (y >= redY) return RGB16_GREEN;
	return RGB16_RED;
}

#define MY_PI1 1.570796 // PI * 1/2
#define MY_PI2 3.141592 // PI * 1
#define MY_PI3 4.712389 // PI * 3/2
#define MY_PI4 6.283185 // PI * 2


bool CirclePlot::ccw(Point a, Point b, Point c){
    int32_t f = (b.x - a.x) * (c.y - b.y);
    int32_t s = (c.x - b.x) * (b.y - a.y);
    return f - s >= 0;
}

int16_t inline CirclePlot::myMax(int16_t a, int16_t b, int16_t c, int16_t d){
	int16_t maks = a;
	if (b>maks) maks = b;
	if (c>maks) maks = c;
	if (d>maks) maks = d;
	return maks;
}

int16_t inline CirclePlot::myMin(int16_t a, int16_t b, int16_t c, int16_t d){
	int16_t minim = a;
	if (b<minim) minim = b;
	if (c<minim) minim = c;
	if (d<minim) minim = d;
	return minim;
}

uint16_t CirclePlot::mapping(int16_t x, int16_t y, Point cmpPointHigher, Point cmpPointLower, int16_t prevValue, int16_t actValue){
    byte base = pgm_read_byte_far(circlePlotBase + 128 * (y >> 3) + x);
    byte pointerGreen = pgm_read_byte_far(circlePlotPointerGreen + 128 * (y >> 3) + x);
    byte pointerRed = pgm_read_byte_far(circlePlotPointerRed + 128 * (y >> 3) + x);
    // is base pixel
    if ((base >> (y%8)) & 1) return RGB16_WHITE; 
    // is not plot pixel
	if (prevValue < actValue){
        if ((pointerGreen >> (y%8)) & 1){
        	if (ccw(Point(x0, y0), cmpPointHigher, Point(x, y))) return 0x0;
        	else return RGB16_GREEN;
        } else if ((pointerRed >> (y%8)) & 1){
        	if (ccw(Point(x0, y0), cmpPointHigher, Point(x, y))) return 0x0;
        	else return RGB16_RED;
        } 
	} else {
        if ((pointerGreen >> (y%8)) & 1){
        	if (ccw(Point(x0, y0), cmpPointLower, Point(x, y))) return 0x0;
        	else return RGB16_GREEN;
        } else if ((pointerRed >> (y%8)) & 1){
        	if (ccw(Point(x0, y0), cmpPointLower, Point(x, y))) return 0x0;
        	else return RGB16_RED;
        } 

	}
    return 0x0;
}

void CirclePlot::changeVal(int16_t val){

    float actAngle = a * (prevValue + val) + b;
    int actValue = prevValue + val;
	float minAngle, maxAngle;
    
    Point points[4];
    if (val>0){
    	points[0] = prevPoints[0];
    	points[1] = prevPoints[1];
    	points[2] = Point(x0 + rL * cos(actAngle), y0 + rL * sin(actAngle));
    	points[3] = Point(x0 + rS * cos(actAngle), y0 + rS * sin(actAngle)); //zrobić mapowanie tych punktów gdzieś w pamięci FLASH
    	minAngle = prevAngle;
    	maxAngle = actAngle;
    } else {
    	points[0] = Point(x0 + rS * cos(actAngle), y0 + rS * sin(actAngle)); //zrobić mapowanie tych punktów gdzieś w pamięci FLASH
    	points[1] = Point(x0 + rL * cos(actAngle), y0 + rL * sin(actAngle));
    	points[2] = prevPoints[1];
    	points[3] = prevPoints[0];
    	minAngle = actAngle;
    	maxAngle = prevAngle;
    }

    int16_t minX = myMin(points[0].x, points[1].x, points[2].x, points[3].x);
    if (minAngle < MY_PI2 && maxAngle > MY_PI2) minX = x0-rL;
    int16_t minY = myMin(points[0].y, points[1].y, points[2].y, points[3].y);
    if (minAngle < MY_PI3 && maxAngle > MY_PI3) minY = y0-rL;
    int16_t maxX = myMax(points[0].x, points[1].x, points[2].x, points[3].x);
    int16_t maxY = myMax(points[0].y, points[1].y, points[2].y, points[3].y);
    if (minAngle < MY_PI1 && maxAngle > MY_PI1) maxY = y0+rL;

    int16_t sizeX = maxX - minX + 1;
    int16_t sizeY = maxY - minY + 1;

    lcd->setXY(positionY+minY, positionY+minY + sizeY - 1, positionX+minX, positionX+minX + sizeX - 1);

    for (int16_t x=sizeX-1; x>=0; x--){
    	for (int16_t y=0; y<sizeY; y++){
			lcd->WriteData(mapping (minX+x, minY+y, points[2], points[1], prevValue, actValue));
    	}
    }

    if (val>0){
    	prevPoints[0] = points[3];
    	prevPoints[1] = points[2];
	} else {
    	prevPoints[0] = points[0];
    	prevPoints[1] = points[1];

	}

    prevValue = actValue;
    prevAngle = actAngle;
}

void CirclePlot::init(LCD_ILI9325D* lcdArg, int16_t positionXArg, int16_t positionYArg, int16_t maxValueArg, char title[10], char unitNameArg[5]){
	lcd = lcdArg;
	positionX = positionXArg;
	positionY = positionYArg; 
	maxValue = maxValueArg;

    for (int i=0; i<5; i++){
        unitName[i] = unitNameArg[i];
    }
    unitName[5] = 0;

	char titleToWrite[11];
	for (int i=0; i<10; i++){
		titleToWrite[i] = title[i];
	}
	titleToWrite[10]=0;

	prevAngle = b;
    prevValue = 0;

    lcd->setColor(RGB16_WHITE);
    lcd->setXY(positionX, positionY);
    lcd->draw(circlePlotBase, 128, 120);

    prevPoints[0] = Point(x0 + rS * cos(b), y0 + rS * sin(b));
    prevPoints[1] = Point(x0 + rL * cos(b), y0 + rL * sin(b));

    lcd->setFontSize(FONT_SIZE_SMALL);
    lcd->setCursor(positionX+90, (positionY+80)/8);
    lcd->print(unitName);

	lcd->setFontSize(FONT_SIZE_SMALL);
    lcd->setCursor(positionX+60, (positionY+104)/8);
	lcd->print(titleToWrite);
}

void CirclePlot::init(LCD_ILI9325D* lcdArg, int16_t positionXArg, int16_t positionYArg, int16_t maxValueArg, char unitNameArg[5]){
	init(lcdArg, positionXArg, positionYArg, maxValueArg, "", unitNameArg);
}

void CirclePlot::displayValue(int16_t valArg){
	if (valArg > maxValue) valArg = maxValue;
	if (valArg < 0) valArg = 0;

	int32_t val = ((int32_t)valArg * 255)/maxValue;

	int16_t chVal = (int16_t)val - prevValue;

	while (chVal != 0){
		if (abs(chVal) > 70){
			chVal = 70 * ((val - prevValue < 0) ? -1 : (val - prevValue > 0));
		} else {
			chVal = val - prevValue;
		}
		changeVal(chVal);
		chVal = val - prevValue;
	}

    lcd->setFontSize(FONT_SIZE_XLARGE);
    lcd->setCursor(positionX+35, (positionY+64)/8);
    lcd->printInt(valArg, 3);
}

void GPlot::init(LCD_ILI9325D* lcdArg, int16_t positionXArg, int16_t positionYArg, int16_t maxValueArg, int16_t redAreaValueArg, bool invertXArg, bool invertYArg, char unitNameArg[5]){
	lcd = lcdArg; 
	positionX = positionXArg; 
	positionY = positionYArg; 
	invertX = invertXArg;
	invertY = invertYArg;
	maxValue = maxValueArg;

	for (int i=0; i<5; i++){
		unitName[i] = unitNameArg[i];
	}
	unitName[5] = 0;

	lcd->setColor(RGB16_WHITE);
	lcd->setXY(positionX, positionY);
	lcd->draw(gPlotBase, 128, 120);

	prevValueX = 0;
	prevValueY = 0;

	redAreaValue = (int16_t) ((int32_t) redAreaValueArg * 44)/maxValue;

	lcd->setFontSize(FONT_SIZE_SMALL);
    lcd->setCursor(positionX+8+44+2+12+2+5, (positionY+8+36)/8);
	lcd->print(unitName);
}

void GPlot::displayValue(int16_t valArgX, int16_t valArgY){

	if (invertX) valArgX = -valArgX;
	if (invertY) valArgY = -valArgY;

	if (valArgX > maxValue) valArgX = maxValue;
	if (valArgX < -maxValue) valArgX = -maxValue;
	int32_t valX = ((int32_t) valArgX * 44)/maxValue;

	if (valArgY > maxValue) valArgY = maxValue;
	if (valArgY < -maxValue) valArgY = -maxValue;
	int32_t valY = ((int32_t) valArgY * 44)/maxValue;

	changeVal((uint16_t)valX - prevValueX, (int16_t)valY - prevValueY);

	char buf[10];
	sprintf(buf, "%4d", valArgY);
	lcd->setFontSize(FONT_SIZE_MEDIUM);
    lcd->setCursor(positionX+8+44-42, (positionY+12)>>3);
    lcd->print(buf);


	sprintf(buf, "%4d", valArgX);
	lcd->setFontSize(FONT_SIZE_MEDIUM);
    lcd->setCursor(positionX+8+44+2+12+2+15, (positionY+8+44+2+12+2+4)>>3);
    lcd->print(buf);
}

uint16_t GPlot::mapping(int16_t val, int16_t prevValue, int16_t actValue){
	if (actValue > prevValue){
		if (val >= 0){
			if (val>redAreaValue) return RGB16_RED;
			else return RGB16_GREEN;
		} else {
			return 0x0;
		}
	} else {
		if (val < 0){
			if (val < -redAreaValue) return RGB16_RED;
			else return RGB16_GREEN;
		} else {
			return 0x0;
		}
	}
	return 0x0;
}

void GPlot::drawRectangleMapping(uint16_t x, uint16_t width, uint16_t y, uint16_t height, int16_t prev, int16_t act, int16_t ref, bool useWidth){
	lcd->setXY(positionY+y, positionY+y+height-1, positionX+x, positionX+x+width-1);
	for (int16_t i = width-1; i>=0; i--){
		for (int16_t j = 0; j<height; j++){
			lcd->WriteData(mapping(ref+(!useWidth)*j + useWidth*i, prev, act));
		}
	}
}

void GPlot::changeVal(int16_t valX, int16_t valY){

	int16_t actValueX = prevValueX + valX;
	int16_t actValueY = prevValueY + valY;

	if (prevValueY < 0 && valY > 0) {
		int16_t length = min(-prevValueY, valY);
		drawRectangleMapping(
			8 + 44 + 2, 
			12, 
			8 + 44 + 2 + 12 + 2 - prevValueY - length, 
			length, 
			-prevValueY, 
			-actValueY,
			-prevValueY - length,
			false);

		valY -= length;
		prevValueY += length;
	} else if (prevValueY > 0 && valY < 0){
		int16_t length = min(prevValueY, -valY);
		drawRectangleMapping(
			8 + 44 + 2, 
			12, 
			8 + 44 - prevValueY, 
			length, 
			-prevValueY, 
			-actValueY,
			-prevValueY,
			false);
		valY += length;
		prevValueY -= length;
	} 

	if (valY > 0) { //prevValueY >= 0
		drawRectangleMapping(
			8+44+2, 
			12, 
			8+44-valY-prevValueY, 
			valY, 
			-prevValueY, 
			-actValueY, 
			-prevValueY-valY,
			false);
	} else if (valY < 0){ //prevValueY <= 0
		drawRectangleMapping(
			8+44+2, 
			12, 
			8+44+2+12+2-prevValueY, 
			-valY, 
			-prevValueY, 
			-actValueY,
			-prevValueY,
			false);
	}

	if (prevValueX > 0 && valX < 0) {
		int16_t length = min(prevValueX, -valX);
		drawRectangleMapping(
			8 + 44 + 2 + 12 + 2 + prevValueX - length, 
			length, 
			8 + 44 + 2, 
			12, 
			prevValueX, 
			actValueX,
			prevValueX,
			true);

		valX += length;
		prevValueX -= length;
	} else if (prevValueX < 0 && valX > 0){
		int16_t length = min(-prevValueX, valX);
		drawRectangleMapping(
			8 + 44 + prevValueX, 
			length, 
			8 + 44 + 2, 
			12, 
			prevValueX, 
			actValueX,
			prevValueX,
			true);
		valX -= length;
		prevValueX += length;
	}

	if (valX > 0){ //prevValueX >= 0
		drawRectangleMapping(
			8+44+2+12+2+prevValueX, 
			valX, 
			8+44+2, 
			12, 
			prevValueX, 
			actValueX,
			prevValueX,
			true);
	} else if (valX < 0) { //prevValueX <= 0
		drawRectangleMapping(
			8+44+prevValueX+valX, 
			-valX, 
			8+44+2, 
			12, 
			prevValueX, 
			actValueX, 
			prevValueX+valX,
			true);
	} 

	prevValueY = actValueY;
	prevValueX = actValueX;
}
