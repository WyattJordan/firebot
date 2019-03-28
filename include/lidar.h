#ifndef LIDAR_INCLUDE
#define LIDAR_INCLUDE

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "line.h"
#include "endpoint.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include <ctime>
#include "math.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define POLAR2XCART(x, y) ((x)*cos((y)*M_PI/180.)) //get the x component when given a distance and angle in degrees
#define POLAR2YCART(x, y) ((x)*sin((y)*M_PI/180.)) //get the y component when given a distance and angle in degrees

using namespace std;

//void leastSquareFit(vector <float> &x, vector <float> &y, float &slope, float &b);
//x includes all of the xpoints in the line
//y includes all of the ypoints in the line

class Lidar{
	public:
		float pt2PtDist(float x1, float y1, float x2, float y2);
		vector<line> findLine(vector <float> xReal, vector <float> yReal);
		bool canMerge(line a, line b);
		float myAngle(float x, float y);
		float myRad(float x, float y);
		void findRoom(vector <line> lineVec);
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
		void findStartLocation(endpoint endR1, endpoint endR2, endpoint endG1, endpoint endG2);
	private:

};

#endif
