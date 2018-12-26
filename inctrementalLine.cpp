#include <iostream>
#include <cmath>
#include <vector>
#include "incrementalLine.h"

using namespace std;

int main() {
	/*
	float myD;
	pair <float, float> point1;
	pair <float, float> point2;
	pair <float, float> point3;
	point1.first = 1;
	point1.second = 1;
	point2.first = 2;
	point2.second = 2;
	point3.first = 1;
	point3.second = 0;
	int exit;
	myD = findDist(point1.first, point1.second, point2.first, point2.second, point1.first, point1.second);
	cout << myD << endl << "Press any key to continue";
	cin >> exit;
	*/

	return 0;

}



/*
void leastSquareFit(vector <float> &x, vector <float> &y, float &slope, float &b) {
	//pass by reference needed to be added
	float xAvg = 0, yAvg = 0;
	for (int i = 0; i < x.size; i++) {
		xAvg += x[i];
		yAvg += y[i];
	};
	xAvg /= (x.size);
	yAvg /= (y.size);

	float num = 0, denum = 0;

	for (int i = 0; i < x.size; i++) {
		num += (x[i] - xAvg)*(y[i] - yAvg);
		denum += pow(x[i] - xAvg, 2);
	};
	slope = num / denum;
	b = yAvg - slope * xAvg;
};
*/

void findLine(vector <float> xVal, vector <float> yVal) {
	vector <line> myLines;
	for (int i = 0; i < xVal.size; i++) {
		if (i <= 1) {
			myLines.push_back(line());
		}
	}
};
void line::setFloats() {
	float xAvg = 0, yAvg = 0;
	for (int i = 0; i < x.size; i++) {
		xAvg += x[i];
		yAvg += y[i];
	};
	xAvg /= (x.size);
	yAvg /= (y.size);

	float num = 0, denum = 0;

	for (int i = 0; i < x.size; i++) {
		num += (x[i] - xAvg)*(y[i] - yAvg);
		denum += pow(x[i] - xAvg, 2);
	};
	slope = num / denum;
	intercept = yAvg - slope * xAvg;
};

float line::getIntercept() {

	return intercept;
};

float line::getSlope() {

	return slope;
};

void line::addPoint(float xVal, float yVal) {
	x.push_back(xVal);
	y.push_back(yVal);
};

float line::findDist(float xPoint, float yPoint) {
	float a, b, c, dy, dx, m;
	a = slope;
	b = -1;
	c = intercept;
	float num, denum;
	num = abs(a*xPoint + b * yPoint + c);
	denum = sqrt(pow(a, 2) + pow(b, 2));
	float d;
	d = num / denum;

	return d;
};