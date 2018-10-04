/* Maps.h stores all the potential endpoints that the robot might encounter.
 * Some points are deleted based on the lidar callback running the robot::determineMap()
 * which reconfigures some of the points to account for the different maps that can be
 * encountered. 
 */
#pragma once 
#include "endpoint.h"
#include <vector>
#include <string>

using std::vector;
using std::string;
struct polarPoint{
	float theta, R;
	polarPoint(float t, float r) : theta(t), R(r){ }
};

class Map{
	private:
		vector<EndPoint> mapPoints;
		vector<EndPoint> wayPoints;
		vector<polarPoint> polarPoints;
		vector<int> expectedMarkerIdxs;
		bool room1Conf, room4Conf;
	public:
		void printCode(int code);
		Map();
		Map(string file);
		void getExpectedMarkers(float Rx, float Ry, float theta);
		void publishMap();
		EndPoint getPoint(int id);
		EndPoint getBadPoint();
		int getSize();

};

