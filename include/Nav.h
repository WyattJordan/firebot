/* "Nav.h" 
 * Handles all the path planning and navigation. Publishes markers in RVIZ.
 */
#ifndef NAV_H
#define NAV_H

#include "ros/ros.h"
#include <ros/console.h> 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include "Endpoint.h"
#include "line.h"
#include "Endpoint.h"
#include "Eigen/Core"

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <deque>
#include <time.h>
#include <chrono>
#include <numeric>
#include <cmath>
using std::vector;
using std::cout;
using std::string;
using namespace Eigen;


class Robot; // forward declaration since both include eachother
class Lidar;

struct color{
	float r,g,b;
};

class Nav{
	public:

		Nav(); // don't use
		Nav(int lvl, ros::Publisher *pub, Robot* rob); 	// read map and way from file given level
		Nav(int lvl, ros::Publisher *pub);   			// read map and way from file given level

		int foundCandle(float x, float y); // given a candle location, make new waypoint in front of candle and return its ID

		void setOdomLoc(Vector3f od);	 // rob obj sends data over
		void setSmallRoomUpper(bool up); // reconfigure the smaller room based on where the door is
		void setBigRoomUpper(bool up);   // reconfigure the larger room based on where the door is

		// Called from lidar class. Checks if a line is viable for update (adjacent, close proximity, & parallel). 
		// Find the coordinate of the wall (X or Y) and get the update based on distance to the wall.
		// Send update to the Robot class to integrate into the driveloop in Robot::calculateOdom()
		bool updatePosition(vector<line> lns, Vector3f pos, Ref<Vector3f> travelDist);

		// PNXY is 1:4 meaning if updating X or Y and adding or subtracting from the wall location. Horiz is if the wall
		// is horizontal in the global frame. Returns the coordinate of the wall (Y if horiz, X if !horiz).
		float findWallValue(int PNXY, Vector3f pos, bool horiz);

		bool pubWays_, pubMap_, pubRob_;// flags for publishLoop() set by rob obj
		void publishLoop(); 			// calculate marks and publish as flags are set
		void publishLoopContinual(); 	// calculate marks and publish every 2 seconds

		// call outputGraph for wayPoints and mapPoints
		void outputWays(); 
		void outputMap();
		
		// calculate RVIZ markers for the mapPoints, wayPoints, Furniture points, Lidar lines, and candle point
		void makeMapMarks(string NS); 
		void makeWayMarks(string NS); 
		void makeFurnMarks(vector<EndPoint> furns); 
		void makeLineMarks(vector<line> lines, bool merged, bool addIDs=false);
		void makeCandleMark(EndPoint ep);

		// change the color of waypoints being used by the navstack
		void highlightWays(deque<EndPoint> pts);

		// publish markers over ROS network
		void publishMapAndWays();
		 
		vector<int> findWay(int start, int end);

		// Endpoint manipulation functions
		bool removePoint(int id, vector<EndPoint> &pts);
		float getDistance(EndPoint &ep1, EndPoint &ep2);
		EndPoint getMapPoint(int id);
		EndPoint getWayPoint(int id);
		EndPoint& getPoint(int id, vector<EndPoint> &pts);
		EndPoint& getBadPoint();
		
	private:
		Robot* rob_; // pointer to Robot object for sending updated location back to it
		bool smallRoomConf_, bigRoomConf_; 	// flags to know when rooms have been configured 
		vector<EndPoint> mapPoints_;	// graph of map corners and key points
		vector<EndPoint> wayPoints_;	// graph of robot drive-to locations
		vector<int> usedForUpdate_;     // IDs of most recent map pts used for updating location
		vector<int> expectedIDs_;	    // ids of markers expected to be visible
		EndPoint safeZone_, candle1_, candle2_; // key location markers
		EndPoint badPt_;		// used when an EndPoint isn't found
		color cmapLine_, cmapMark_, cwayLine_, cwayMark_; // colors set in constr

		visualization_msgs::MarkerArray mapMarks_, wayMarks_, robMarks_, furnMarks_, lineMarks_, globPtMarks_;// for rviz
		ros::Publisher *markerPub_; 	// publisher for all MarkerArrays
		Vector3f odomWorldLocCpy_;		// copy of loc in world frame from Robot, READ ONLY
		
		void defaults(); // set default values for all variables (called by constructors)
		void loadFiles(int lvl); // get data from config files (mapPoints and wayPoints)

		// Initialize and calculate the update for the Robot marker (blue disc with red arrow to show pose).
		void initRobotMarks();   // sets type, ns, frame, color
		void calcRobotMarks();   // sets the xyz position

		// highlights in green the mapMarks in usedForUpdate_ and sets all others to default color
		void highlightUsedMapMarks();

		// get the robot position from Robot*, find the closest waypoint and return its ID
		int getNearestWayID();

		// make a line from the map data that is closest to what was sensed by the lidar
		line makeClosestLine(Vector3f gPt, bool horiz);

		// cout a graph of endpoints
		void outputGraph(vector<EndPoint> &pts);

		// copy the neighI'th neighbor of point with id=ID into &neigh, return false if DNE
		bool getNeighbor(int ID, int neighI, EndPoint &neigh, vector<EndPoint> &pts);

		// given starting ids and pts graph return vec of ids between the two specifying shortest path
		vector<int> findPath(int start, int end, vector<EndPoint> &pts);
		
		// set isVisible for each Endpoint in pts given a robot position. Which mapPoints should be visible to the 'bot.
		void findExpected(float Rx, float Ry, vector<EndPoint> &pts);
		// removes all points hidden by wall between ep1 and ep2 given a robot position (Rx, Ry)
		void eliminatePts(EndPoint &ep1,EndPoint &ep2, float Rx,
			float Ry, vector<EndPoint> &pts);

		// make the marks for either mapPoints_ or wayPoints_ (specified by which string)
		void populateMarks(string which, string NS,
			visualization_msgs::MarkerArray &marks, color lncol, color markcol);
};
#endif
