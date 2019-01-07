#ifndef INCREMENTAL_LINE_INCLUDE
#define INCREMENTAL_LINE_INCLUDE

#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

class line {
public:
	void setFloats();
	//least squared fit
	float getSlope();
	//returns slope
	float getIntercept();
	//returns intercept
	void addPoint(float xVal, float yVal);
	//adds a point to the line
	float findDist(float xPoint, float yPoint);
	//finds the distance from a given point to the line
	void clearLine();
	//empties the vectors of the line
	void printLine();
	float lineSize();
private:
	vector <float> x;
	vector <float> y;
	float slope;
	float intercept;
};



//void leastSquareFit(vector <float> &x, vector <float> &y, float &slope, float &b);
//x includes all of the xpoints in the line
//y includes all of the ypoints in the line

void findLine(vector <float> xReal, vector <float> yReal);

#endif