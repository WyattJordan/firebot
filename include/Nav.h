/* "Nav.h" 
 * Handles all the path planning and navigation. Also used for most of the logic
 * concerning the robot's actions.
 */
#pragma once
#include "Endpoint.h"
#include "ros/ros.h"
#include <visualization_msgs/MarkerArray.h>
#include <string>
using std::string;

struct color{
	float r,g,b;
};

class Nav{
	public:
		bool pubWays_, pubMap_;
		Nav();
		Nav(int lvl, ros::Publisher *pub); // read map and way from file given level
		void findExpected(float Rx, float Ry, vector<EndPoint> &pts);
		void publishLoop();
		void publishBot(float Rx, float Ry);
		void publishMapAndWays(float Rx, float Ry);
		void setSmallRoomUpper(bool up);
		void setBigRoomUpper(bool up);
		vector<int> findPath(int start, int end, vector<EndPoint> &pts);

		void outputWays();
		void outputMap();
		void makeMapMarks(string NS, string frame);
		void makeWayMarks(string NS, string frame);

		EndPoint& getPoint(int id, vector<EndPoint> &pts);
		EndPoint& getBadPoint();
		float getDistance(EndPoint &ep1, EndPoint &ep2);
		bool removePoint(int id, vector<EndPoint> &pts);
		void setRun(bool t);
		void outputGraph(vector<EndPoint> &pts);
		void run();
		vector<EndPoint>* getMap();
		vector<EndPoint>* getWays();

	private:
		EndPoint safeZone_, candle1_, candle2_; // key location markers
		vector<EndPoint> mapPoints_;
		vector<EndPoint> wayPoints_;
		bool smallRoomConf_, bigRoomConf_, runBool_;
		vector<int> expectedIDs_;
		visualization_msgs::MarkerArray mapMarks_, wayMarks_;
		ros::Publisher *markerPub_;
		color cmapLine_, cmapMark_, cwayLine_, cwayMark_;
		EndPoint badPt_;

		void populateMarks(string which, string NS, string frame,
			visualization_msgs::MarkerArray &marks, color lncol, color markcol);

		void publishGraph(float Rx, float Ry, string NS, 
			vector<EndPoint> &pts, color lncol, color  markcol);

		bool getNeighbor(int startID, int neighI, EndPoint &neigh, vector<EndPoint> &pts);
		void eliminatePts(EndPoint &ep1,EndPoint &ep2, float Rx,
			float Ry, vector<EndPoint> &pts);

		// save the generated markerarray for any vec of Endpoints
		void makeMarks(vector<EndPoint> &pts, color lncol, 
				color markcol, string NS, string frame);	
};
