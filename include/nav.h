/* "nav.h" 
 *
 *
 */
#pragma once
#include "endpoint.h"
#include <string>
using std::string;

struct polarPoint{
	float theta, R;
	polarPoint(float t, float r) : theta(t), R(r){ }
};


class Nav{
	private:
		EndPoint safeZone, candle1, candle2; // key location markers
		vector<EndPoint> mapPoints;
		vector<EndPoint> wayPoints;
		vector<polarPoint> polarPoints;
		vector<int> expectedMarkerIdxs;
		bool room1Conf, room4Conf;

		bool getNeighbor(int startID, int neighNum, EndPoint &neigh);
	public:
		Nav();
		Nav(string mapfile); // read from file
		void findExpected(float Rx, float Ry, float theta);
		void publishMap();
		EndPoint getPoint(int id);
		EndPoint getBadPoint();
		int getSize();
		void outputMapPoints();
};
