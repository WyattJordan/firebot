/* "Nav.h" 
 *
 *
 */
#pragma once
#include "Endpoint.h"
#include <string>
using std::string;


class Nav{
	private:
		EndPoint safeZone, candle1, candle2; // key location markers
		vector<EndPoint> mapPoints;
		vector<EndPoint> wayPoints;
		//vector<polarPoint> polarPoints;
		bool smallRoomConf, bigRoomConf, runBool;
		vector<int> expectedIDs;
		bool getNeighbor(int startID, int neighI, EndPoint &neigh);
		void eliminatePts(EndPoint &ep1,EndPoint &ep2, float Rx, float Ry);
		EndPoint badPt;
	public:
		Nav();
		Nav(string mapfile, string wayfile); // read from file
		void findExpected(float Rx, float Ry, vector<EndPoint> &pts);
		void publishBot(float Rx, float Ry);
		void publishGraph(float Rx, float Ry, string NS, vector<EndPoint> &pts);
		void setSmallRoomUpper(bool up);
		void setBigRoomUpper(bool up);

		EndPoint& getPoint(int id);
		EndPoint& getBadPoint();
		void removePoint(int id);
		void setRun(bool t);
		void outputGraph(vector<EndPoint> pts);
		void run();
		vector<EndPoint>* getMap();
		vector<EndPoint>* getWays();
};
