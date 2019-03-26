#ifndef MY_CLIENT_INCLUDE
#define MY_CLIENT_INCLUDE

#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include "endpoint.h"
#include <algorithm>
#include "line.h"

using namespace std;

//void leastSquareFit(vector <float> &x, vector <float> &y, float &slope, float &b);
//x includes all of the xpoints in the line
//y includes all of the ypoints in the line

void findLine(vector <float> xReal, vector <float> yReal);
bool canMerge(line a, line b);
float pt2PtDist(float x1, float y1, float x2, float y2);
float myAngle(float x, float y);
float myRad(float x, float y);
void findRoom(vector <line> lineVec);
void findStartLocation(endpoint endR1, endpoint endR2, endpoint endG1, endpoint endG2);

#endif
