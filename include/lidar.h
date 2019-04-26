#ifndef LIDAR_INCLUDE
#define LIDAR_INCLUDE

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/LaserScan.h"
#include <Eigen/Core>
#include "line.h"
#include "Endpoint.h"
#include "definitions.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include <ctime>
#include "math.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define POLAR2XCART(r, t) ((r)*cos((t)*M_PI/180.)) //get the x component when given a distance and angle in degrees
#define POLAR2YCART(r, t) ((r)*sin((t)*M_PI/180.)) //get the y component when given a distance and angle in degrees
#define XYTORADIUS(x, y) (pow(x*x + y*y, 0.5))


using namespace std;
using namespace Eigen;
class Robot;
class Nav;

//void leastSquareFit(vector <float> &x, vector <float> &y, float &slope, float &b);
//x includes all of the xpoints in the line
//y includes all of the ypoints in the line

class Lidar{
	public:
		Lidar();
		Lidar(Nav *navRef);
		Lidar(Robot *robRef, Nav *navRef);
		void setNav(Nav *nav);
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
		int findCandle();
		void locateCandle();
		void input();
		void findJumps(bool findBig); // finds either big jumps and furniture jumps or only furn jumps
		int classifyRoomFromJumps();
		void findLines(bool pubSegmets=false);

		vector<line> findLine();
		bool canMerge(line a, line b);
		void findRoom();
		void findStartLocation(EndPoint endR1, EndPoint endR2, EndPoint endG1, EndPoint endG2);
		bool canMerge(line a);
		float getMaxRSquare();

	private:
		Vector3f prevOdom_;
		Nav* nav_;
		Robot* rob_;
		bool executing_; // to stop callback from running again if it's already running
		bool checkCandle_, pauseUpdates_;
		int startCount_, localRoom_;
		vector<int> startRooms_, outliers_;
		vector<EndPoint> candleLocs_;
		bool started_, keypress_;
		unsigned int tickCount_;

		// Point data is stored in 4 vectors with cartesian and polar coordinates
		vector<float> xVal_, yVal_, rad_, degrees_;
		// jump_ is a list of idxs at the start of a major difference in polar length
		// i.e. rad_[jump[j]] and rad_[jump[j] + 1] are different by at least DoorJumpDist
		// also note that due to loop around never use jump[j] + 1 but rather getEndIdx(jump[j])
		vector<int> jump_, cJumps_;
		// acts just like jump_ except with smaller threshold of SmallJumpDist
		vector<int> smallJump_, furnJumpsConfirmed_;
		// list of endpoints that should be the center of furniture
		vector<EndPoint> furns_;
		// idxs of the furniture so they can be ignore in findLine
		vector<int> furnIdxs_;
		vector<line> lines_;

		void processData(const sensor_msgs::LaserScan::ConstPtr& scan);
		bool checkLocalize();
		// Knowing in room 4 determine location
		void room1Localization();
		void room2Localization();
		void room3Localization();
		vector<int> outsideWalls_;
		void room4Localization(bool down);

		// Given a point in the lidar frame (l) and global frame(g) and the pose (t) localize the bot
		void localizeFromPt(EndPoint l, EndPoint g, float t);
		// Determine average of points before and after center using offset number of points
		void getAveragePrePost(float &pre, float &post, int center, int offset,bool degug=false);
		// Build furn_ based on furnJump_
		void findFurniture();

		// A value in jump_ or smallJump_ is the first index of 2 indices in the data vectors that
		// make up a jump, these functions get various info about those two points
		int getEndIdx(int s); // for loop around
		int getCloserJumpPt(int i);
		int getFurtherJumpPt(int i);
		float getCloserJumpRadius(int i);
		float getFurtherJumpRadius(int i);


		void cleanBigJumps();
		bool jumpAway(int i);
		float pt2PtDist(float x1, float y1, float x2, float y2);
		void removePt(int i);
		int modeRoom();
		float getMapPoint();
		void defaults();
};

#endif
