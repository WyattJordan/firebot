/* "Nav.h" 
 *
 *
 */
#pragma once
#include "Endpoint.h"
#include <string>
using std::string;

struct color{
	float r,g,b;
};

class Nav{
	private:
		EndPoint safeZone, candle1, candle2; // key location markers
		vector<EndPoint> mapPoints;
		vector<EndPoint> wayPoints;
		//vector<polarPoint> polarPoints;
		bool smallRoomConf, bigRoomConf, runBool;
		vector<int> expectedIDs;
		bool getNeighbor(int startID, int neighI, EndPoint &neigh, vector<EndPoint> &pts);
		void eliminatePts(EndPoint &ep1,EndPoint &ep2, float Rx, float Ry, vector<EndPoint> &pts);
		EndPoint badPt;
	public:
		Nav();
		Nav(string mapfile, string wayfile); // read from file
		void findExpected(float Rx, float Ry, vector<EndPoint> &pts);
		void publishBot(float Rx, float Ry);
		void publishMapAndWays(float Rx, float Ry);
		void publishGraph(float Rx, float Ry, string NS, vector<EndPoint> &pts, color lncol, color  markcol);
		void setSmallRoomUpper(bool up);
		void setBigRoomUpper(bool up);
		vector<int> findPath(int start, int end, vector<EndPoint> &pts);

		EndPoint& getPoint(int id, vector<EndPoint> &pts);
		EndPoint& getBadPoint();
		float getDistance(EndPoint &ep1, EndPoint &ep2);
		void removePoint(int id, vector<EndPoint> &pts);
		void setRun(bool t);
		void outputGraph(vector<EndPoint> pts);
		void run();
		vector<EndPoint>* getMap();
		vector<EndPoint>* getWays();
};
