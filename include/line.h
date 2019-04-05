#ifndef LINE_INCLUDE
#define LINE_INCLUDE

#include "Endpoint.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>

using namespace std;

class line {
public:
        line();
        void buildLine();
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
        int numPts();
        //returns the number of points in the line
        void reverseLine();
        void setEndpts(float x1, float y1, float x2, float y2);
        float getEndPtX1();
        float getEndPtY1();
        float getEndPtX2();
        float getEndPtY2();

		// Functions to be added for localizing in room
		float getCenterX();
		float getCenterY();
		float getCenterTheta();
		float getCenterRadius();

        void mergeLines(line a);
        bool canMerge(line a);
        float endPAngle(int num);
        float endPRad(int num);
        float getLineDist();
        void setGood(bool a);
        bool isGoodLine();
        float getLength();
        bool isCandle();
        bool isFurniture();
		float radDist(int i);

private:
        vector <float> x;
        vector <float> y;
        float slope;
        float intercept;
        EndPoint end1;
        EndPoint end2;
		EndPoint center;
        float lineDist;
        bool isLine;
        float length;
        bool candle;
        bool furniture;
        float distBetPts = 6*3.14159*180/590;
	float xAvg;
	float yAvg;
	float xSum;
	float ySum;
	float num;
	float denum;
};

#endif
