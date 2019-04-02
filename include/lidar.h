#ifndef LIDAR_INCLUDE
#define LIDAR_INCLUDE

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_broadcaster.h>
#include "Robot.h"
#include "Nav.h"
#include "line.h"
#include "refpoint.h"
#include "Endpoint.h"
#include "definitions.h"
#include <Eigen/Core>
#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include <ctime>
#include "math.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define POLAR2XCART(r, t) ((r)*cos((t)*M_PI/180.)) //get the x component when given a distance and angle in degrees
#define POLAR2YCART(r, t) ((r)*sin((t)*M_PI/180.)) //get the y component when given a distance and angle in degrees

using namespace std;

//void leastSquareFit(vector <float> &x, vector <float> &y, float &slope, float &b);
//x includes all of the xpoints in the line
//y includes all of the ypoints in the line

class Lidar{
	public:
		Lidar();
//		Lidar(Robot *robRef, Nav *navRef);
		float pt2PtDist(float x1, float y1, float x2, float y2);
		vector<line> findLine(vector <float> xReal, vector <float> yReal);
		bool canMerge(line a, line b);
		float myAngle(float x, float y);
		float myRad(float x, float y);
		void findRoom();
		void findRoomFromJumps();
		void findJumps();
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
		void findStartLocation(refpoint endR1, refpoint endR2, refpoint endG1, refpoint endG2);
	private:
		vector<float> xVal;
		vector<float> yVal;
		vector<float> degrees;
		vector<float> rad;
		vector<int> jump;
		void removePt(int i);
		int getEndIdx(int s);
		void room4Localization(vector<int> closeJumps);
		void room1Localization();
		void localizeFromPt(EndPoint l, EndPoint g);
		Vector3f prevOdom_;
		Nav* nav_;
//		Robot* rob_;

};

#endif
