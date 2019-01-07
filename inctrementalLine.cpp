#include <iostream>
#include <cmath>
#include <vector>
#include "incrementalLine.h"

using namespace std;

int main() {
	
	vector <float> myX;
	vector <float> myY;
	line myLine;
	float myDist;

	myX.push_back(1);
	myX.push_back(2);
	myX.push_back(3);
	myY.push_back(5);
	myY.push_back(10);
	myY.push_back(15);
	myX.push_back(2);
	myX.push_back(3);
	myX.push_back(4);
	myY.push_back(2);
	myY.push_back(3);
	myY.push_back(4);


	for (int i = 0; i < myX.size(); i++) {
		myLine.addPoint(myX[i], myY[i]);
	}

	myLine.setFloats();

	cout << "Slope: " << myLine.getSlope() << "  Intercept: " << myLine.getIntercept() << endl;

	myLine.printLine();

	myDist = myLine.findDist(100, 0);

	cout << myDist << endl;

	findLine(myX, myY);

	cout << "Press any key to continue:";
	int myEnd;
	cin >> myEnd;
	

	

	return 0;

}

void findLine(vector <float> xReal, vector <float> yReal){
	vector <line> myLines;
	line tempLine;
	int numLines = 0;			// how many lines detected
	int scopeSize = 0;			// keeps track of where the line breaks
	int i = 0;					// iterator
	float distToLine = 0;		// distance from a point to a line
	bool twoPointLine = false;  // checks to see if the line has only 2 points
	while (i < xReal.size()) {
		for (i; i < xReal.size(); i++) {
			if (i <= scopeSize + 1) {
				tempLine.addPoint(xReal[i], yReal[i]);

			}
			else if (i == scopeSize + 2) {
				

				tempLine.setFloats();
				distToLine = tempLine.findDist(xReal[i], yReal[i]);


				//if the point is within 1 cm to the line, add the point, otherwise end the line
				if (distToLine < 1) {
					myLines.push_back(tempLine);
					myLines[numLines].addPoint(xReal[i], yReal[i]);
				}
				else {

					i--;
					scopeSize = i;
					twoPointLine = true;
					break;
				}
			}
			else {
				myLines[numLines].setFloats();
				distToLine = myLines[numLines].findDist(xReal[i], yReal[i]);


				//if the point is within 1 cm to the line, add the point, otherwise end the line
				if (distToLine < 1) {
					myLines[numLines].addPoint(xReal[i], yReal[i]);
				}
				else {
					i--;
					scopeSize = i;
					break;
				}
			}
		}

		tempLine.clearLine();
		if (twoPointLine == false) {numLines++;}
		twoPointLine = false;
	}

	for (int j = 0; j < myLines.size(); j++) {
		cout << endl << "Line: " << j + 1 << " size: " << myLines[j].lineSize() << endl;
		myLines[j].printLine();
	}
};

void line::setFloats() {
	float xAvg = 0, yAvg = 0;
	for (int i = 0; i < x.size() - 1; i++) {
		xAvg += x[i];
		yAvg += y[i];
	};
	xAvg /= (x.size());
	yAvg /= (y.size());

	float num = 0, denum = 0;

	for (int i = 0; i < x.size() - 1; i++) {
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
	float a, b, c;
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

void line::clearLine() {
	x.clear();
	y.clear();
	slope = 0;
	intercept = 0;
};

void line::printLine() {
	for (int i = 0; i < x.size(); i++) {
		std::cout << x[i] << "   " << y[i] << endl;
	}
};

float line::lineSize() {
	return x.size();
}