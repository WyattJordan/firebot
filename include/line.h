#ifndef LINE_H
#define LINE_H

#include "Endpoint.h"
#include "definitions.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include <stdlib.h>
#include <time.h>

using namespace std;

class line {
public:
        line(); 

	// Given x and y vectors determine the line model eqn (slope + intercept)
	// The endpoints, the center point. 
        void buildLine();

	// Calculates the R^2 for a line that's been built
	float makeRSquared();
	float getRSquared();

	// Get/Set slope or intercept
        float getSlope(); 		
        float getIntercept(); 	
        void setSlope(float s); 
        void setIntercept (float i);

	// Get specific points based on idx
        float getXPoint(int point);
        float getYPoint(int point);

	// Add points to beginning or end
        void addPointEnd(float xVal, float yVal);
        void addPointStart(float xVal, float yVal);

	// finds the distance from a point to the line
        float findDist(float xPoint, float yPoint); 

        void  clearLine(); 		// erase all data
        void  clearPoint(int i);// erase a specific point by idx
	float radDist(int i);	// get radius of pt by idx
        void  printLine(); 		// prints all the points of the line to console
        int   numPts(); 		// returns the number of points in the line
        void  reverseLine(); 	// switches the order of the x and y vectors

	// Get/Set endpoints of line
        void setEndpts(float x1, float y1, float x2, float y2);
        float getEndPtX1();
        float getEndPtY1();
        float getEndPtX2();
        float getEndPtY2();
        float endPAngle(int num);
        float endPRad(int num);
        float getLength(); // set by buildline()

	// Functions to be added for localizing in room
	// To be used when matching line to world frame based on 
	// center point but ended up not using this.
	float getCenterX();
	float getCenterY();
	float getCenterTheta();
	float getCenterRadius();
	float getClosestRadius();
	float getClosestAngle();

	// Determine if line a and this are similar enough to merge from:
	// Slopes, distance of endpoints, and if 3 random points fit the model
        bool canMerge(line a);

	// copy the data from a into line this
        void mergeLines(line a);

	// returns length set from setting the endpoints
        float getLineDist();
	// Get/Set classification
        void setGood(bool a);
        bool isGoodLine();
        bool isCandle();
        bool isFurniture();

private:
        vector <float> x; 	// all x values of points on the line
        vector <float> y; 	// all y values of points on the line
        float slope, intercept, RSquare;
        EndPoint end1, end2, center;
	float length;		// set by buildline() 
	float lineDist; 	// set if endpoints are directly set
	bool isLine, horiz;
	bool candle, furniture;
	float xAvg, yAvg, xSum, ySum;
	float num, denum;
};

#endif
