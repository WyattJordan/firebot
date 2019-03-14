#ifndef INCREMENTAL_LINE_INCLUDE
#define INCREMENTAL_LINE_INCLUDE

#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include "endpoint.h"
#include <algorithm>

using namespace std;

class line {
public:
	void setFloats();
	//least squared fit
	float getSlope();
	//returns slope
	float getIntercept();
	//returns intercept
	void setSlope(float s);
	void setIntercept (float i);
	float getXPoint(int point);
	float getYPoint(int point);
	void addPointEnd(float xVal, float yVal);
	void addPointStart(float xVal, float yVal);
	//adds a point to the line
	float findDist(float xPoint, float yPoint);
	//finds the distance from a given point to the line
	void clearLine();

	
	//empties the vectors of the line
	void clearPoint(int i);
	void printLine();
	//prints all the points of the line
	float lineSize();
	//returns the number of points in the line
	void reverseLine();
	void setEndpts(float x1, float y1, float x2, float y2);
	float getEndPtX1();
	float getEndPtY1();
	float getEndPtX2();
	float getEndPtY2();
	void mergeLines(line a);
	float endPAngle(int num);
	float endPRad(int num);
	float getLineDist();
	void setGood(bool a);
	bool isGoodLine();
	float getLength();
	bool isCandle();
	bool isFurniture();

private:
	vector <float> x;
	vector <float> y;
	float slope;
	float intercept;
	endpoint end1;
	endpoint end2;
	float endPtX1;
	float endPtY1;
	float endPtX2;
	float endPtY2;
	float lineDist;
	bool isLine;
	float length;
	bool candle;
	bool furniture;
};



//void leastSquareFit(vector <float> &x, vector <float> &y, float &slope, float &b);
//x includes all of the xpoints in the line
//y includes all of the ypoints in the line

void findLine(vector <float> xReal, vector <float> yReal);
bool canMerge(line a, line b);
float pt2PtDist(float x1, float y1, float x2, float y2);
float myAngle(float x, float y);
float myRad(float x, float y);



#endif
