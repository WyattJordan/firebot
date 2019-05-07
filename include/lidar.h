#ifndef LIDAR_H
#define LIDAR_H

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

// Polar <--> Cartesian conversions
#define RAD2DEG(x) ((x)*180./M_PI)
#define POLAR2XCART(r, t) ((r)*cos((t)*M_PI/180.)) 
#define POLAR2YCART(r, t) ((r)*sin((t)*M_PI/180.)) 
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
		// constructors, main one used needs pointers for both Robot and Lidar classes
		Lidar();
		Lidar(Nav *navRef);
		Lidar(Robot *robRef, Nav *navRef);
		void setNav(Nav *nav);

		// Runs everytime a Lidar sensor_msgs/LaserScan message is published
		// Has 3 main modes:
		// 1. Find the candle
		// 		finds the jumps in the room and adds to the candleLocs_ vector
		// 2. Classify room and localize from random start
		// 		broken into 2 sections:
		// 		1. classify the room 10 times (counted by startCount_) then set localRoom
		// 		2. localize based on localRoom found, runs until successful
		// 3. Update position in maze
		// 		Checks if linear movement
		// 		Finds the jumps, furniture, and lines (publishes all in rviz)
		//		Calls the nav class to update the position from the line data
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

		// Called by robot class
		// Sets lidar in mode 1. Waits for 50 scans to be processed (guarantee candle is found)
		// If there are at least 20 detections averages them to create candle location
		// Calls the nav class with the candle location in the lidar frame (Nav::foundCandle)
		int findCandle();

		// Determines the candle location based on the data from one scan by looking thru the jumps
		void locateCandle();

		// detects keypress for testing
		void input();

		// Finds jumps in the laser scan (when two points next to eachother have a significant
		// difference in polar radius). Populate smallJump_ and jump_
		void findJumps(bool findBig); // flag determines if jump_ is populated

		// Determine which room it's in based on large jumps (> ~40cm). 
		int classifyRoomFromJumps();

		// Incremental line finding algorithm. Also merges similar lines. Stored in lines_ vector.
		void findLines(bool pubSegmets=false);

		// Uses two lidar endpoints known in the global and local frames to localize
		// Decided to not use this due to lidar inaccuracies in determinign the endpoints
		// or in case the endpoints were blocked by furniture.
		void findStartLocation(EndPoint endR1, EndPoint endR2, EndPoint endG1, EndPoint endG2);

		// determine if two lines are most likely the same one based on slope, endpoint distances
		// and if at least 3 points from one line are within a threshold of the other line's model
		bool canMerge(line a);
		
		// return the maximum R^2 value in the lines vector
		float getMaxRSquare();

	private:
		Vector3f prevOdom_; // to check if linear movement
		Nav* nav_;
		Robot* rob_;
		bool executing_; // to stop callback from running again if still processing last scan
		bool checkCandle_, pauseUpdates_;   // flags for scanCallback
		bool started_, keypress_;			// flags for scanCallback
		int startCount_, localRoom_; 		// for initial room classification 
		vector<int> startRooms_; 			// save several classifications then pick mode
		vector<int> outliers_; 				// jump points that fail average pre/post test
		vector<EndPoint> candleLocs_; 		// store locations of candle 
		unsigned int tickCount_;			// count number of scans read

		// Point data is stored in 4 vectors with cartesian and polar coordinates
		vector<float> xVal_, yVal_, rad_, degrees_;
		// jump_ is a list of idxs at the start of a major difference in polar length
		// i.e. rad_[jump[j]] and rad_[jump[j] + 1] are different by at least DoorJumpDist
		// Also note that due to loop around never use jump[j] + 1 but rather getEndIdx(jump[j])
		vector<int> jump_, cJumps_;
		// acts just like jump_ except with smaller threshold of SmallJumpDist
		vector<int> smallJump_, furnJumpsConfirmed_;
		// list of endpoints for the center of furniture
		vector<EndPoint> furns_;
		// idxs of the lidar points on furniture so they can be ignore in findLine
		vector<int> furnIdxs_;
		// vector of lines that are populated from the findLines() algo
		vector<line> lines_;

		// Populate xVal_,yVal_,rad_, and degrees_ from the scan.
		// Remove the points hitting the support studs between the two levels.
		void processData(const sensor_msgs::LaserScan::ConstPtr& scan);

		// Determine if the scan is good enough to localize.
		// If it is call appropriate localization function. 
		bool checkLocalize();
		// Knowing in room x determine location
		void room1Localization();
		void room2Localization();
		void room3Localization();
		void room4Localization(bool down); // reconfigurable room
		vector<int> outsideWalls_; // find walls outside the room (used for room4Localization)

		// Given a point in the lidar frame (l) and global frame(g) and the pose (t) localize the bot
		void localizeFromPt(EndPoint l, EndPoint g, float t);

		// Determine average of points before and after center using offset number of points
		// This is called at jump pts to determine if the jump was just a bad laser reading.
		void getAveragePrePost(float &pre, float &post, int center, int offset,bool degug=false);

		// Build furn_ based on furnJump_ based on width between jumps and if there are points
		// between the jumps with a smaller polar radius (it's curved)
		void findFurniture();

		// A value in jump_ or smallJump_ is the first index of 2 indices in the data vectors that
		// make up a jump, these functions get various info about those two points
		int getEndIdx(int s); // for loop around
		int getCloserJumpPt(int i);
		int getFurtherJumpPt(int i);
		float getCloserJumpRadius(int i);
		float getFurtherJumpRadius(int i);

		// Remove from jump_ if it was a piece of furniture.
		void cleanBigJumps();
		// Tell if a jump is going away from the bot (first point is closer)
		bool jumpAway(int i);
		// Returns distance between two points
		float pt2PtDist(float x1, float y1, float x2, float y2);
		// Delete a point from all 4 data vectors
		void removePt(int i);
		// Find the mode of the startRooms_ vector
		int modeRoom();
		// Set default values for all variables, called by constructors
		void defaults();
};

#endif
