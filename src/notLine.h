#ifndef NOT_LINE_INCLUDE
#define NOT_LINE_INCLUDE

#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include "endpoint.h"
#include "incrementalLine.h"

using namespace std;

class notLine {
public:
	void addFront(float xVal, float yVal);
	void addRear(float xVal, float yVal);
	void setEndpoints();
	int getSize();
	void reverseNL();
	float getXPoint(int i);
	float getYPoint(int i);
	void clearNL();
	void clearPoint(int i);
	float getEndPtX1();
	float getEndPtY1();
	float getEndPtX2();
	float getEndPtY2();
	void print();
	void lineToNL(line a);
	void mergeFLine(notLine a);
	float endPAngle(int num);
	float getLength();
	bool isCandle();
	bool isFurniture();
private:
	endpoint fEnd1;
	endpoint fEnd2;
	vector <float> x;
	vector <float> y;
	float length;
	bool candle;
	bool furniture;

};


#endif
