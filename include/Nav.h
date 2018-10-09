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
		bool room1Conf, room4Conf;
		vector<int> expectedIDs;
		bool getNeighbor(int startID, int neighI, EndPoint &neigh);
		void eliminatePts(EndPoint &ep1,EndPoint &ep2, float Rx, float Ry);
	public:
		Nav();
		Nav(string mapfile); // read from file
		void findExpected(float Rx, float Ry);
		void publishMap(float Rx, float Ry);
		EndPoint getPoint(int id);
		EndPoint getBadPoint();
		int getSize();
		void outputMapPoints();
		void run();
};
