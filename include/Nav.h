/* "Nav.h" 
 * Handles all the path planning and navigation. Also used for most of the logic
 * concerning the robot's actions.
 */
#pragma once
#include "Endpoint.h"
#include "ros/ros.h"
#include "Eigen/Core"
#include <visualization_msgs/MarkerArray.h>
#include <string>
using std::string;
using namespace Eigen;

struct color{
	float r,g,b;
};

class Nav{
	public:
		bool pubWays_, pubMap_, pubRob_; // flags for publishLoop() set by rob obj

		Nav(); // don't use
		Nav(int lvl, ros::Publisher *pub); // read map and way from file given level

		void setOdomLoc(Vector3f od);	 // rob obj sends data over
		void setSmallRoomUpper(bool up); // reconfigure the rooms
		void setBigRoomUpper(bool up);   // "

		void publishLoop(); // calculate marks and publish as flags are set
		void publishLoopContinual(); // calculate marks and publish every 2 seconds
		void outputWays();  
		void outputMap();
		void makeMapMarks(string NS);
		void makeWayMarks(string NS);
		void makeFurnMarks(vector<EndPoint> furns);
		void makeLineMarks(vector<line> lines);
		void publishMapAndWays();

		bool removePoint(int id, vector<EndPoint> &pts);
		float getDistance(EndPoint &ep1, EndPoint &ep2);
		EndPoint getMapPoint(int id);
		EndPoint getWayPoint(int id);
		EndPoint& getPoint(int id, vector<EndPoint> &pts);
		EndPoint& getBadPoint();
		
	private:
		bool smallRoomConf_, bigRoomConf_; 	// passed to reconfigure rooms
		string worldFrame_;		// specify world frame label
		vector<EndPoint> mapPoints_;	// graph of map corners and key points
		vector<EndPoint> wayPoints_;	// graph of robot drive-to locations
		vector<int> expectedIDs_;	// ids of markers expected to be visible
		EndPoint safeZone_, candle1_, candle2_; // key location markers
		EndPoint badPt_;		// used when an EndPoint isn't found
		color cmapLine_, cmapMark_, cwayLine_, cwayMark_; // colors set in constr

		visualization_msgs::MarkerArray mapMarks_, wayMarks_, robMarks_, furnMarks_;// for rviz
		ros::Publisher *markerPub_; 	// publisher for all MarkerArrays
		Vector3f odomWorldLocCpy_;		// copy of loc in world frame from Robot, READ ONLY

		void loadFiles(int lvl); // get data from config files
		void initRobotMarks();   // sets type, ns, frame, color
		void calcRobotMarks();   // sets the xyz

		// cout a graph of endpoints
		void outputGraph(vector<EndPoint> &pts);

		// copy the neighI'th neighbor of point with id=ID into &neigh, return false if DNE
		bool getNeighbor(int ID, int neighI, EndPoint &neigh, vector<EndPoint> &pts);

		// given starting ids and pts graph return vec of ids between the two specifying shortest path
		vector<int> findPath(int start, int end, vector<EndPoint> &pts);
		
		// set isVisible for each Endpoint in pts given a robot position
		void findExpected(float Rx, float Ry, vector<EndPoint> &pts);
		// removes all points hidden by wall between ep1 and ep2 given a robot position (Rx, Ry)
		void eliminatePts(EndPoint &ep1,EndPoint &ep2, float Rx,
			float Ry, vector<EndPoint> &pts);

		// make the marks for either mapPoints_ or wayPoints_ (specified by which string)
		void populateMarks(string which, string NS,
			visualization_msgs::MarkerArray &marks, color lncol, color markcol);
};
