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
	float findDist(float xPoint, float yPoint);
private:
	vector <float> x;
	vector <float> y;
	float slope;
	float intercept;
};



//void leastSquareFit(vector <float> &x, vector <float> &y, float &slope, float &b);
//x includes all of the xpoints in the line
//y includes all of the ypoints in the line

void findLine(vector <float> xVal, vector <float> yVal);

#endif