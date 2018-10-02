#pragma once 
#include "partialmap.h"
#include <vector>
#include <string>

using std::vector;
using std::string;
/* Maps.h stores all the potential maps that the robot might encounter.
 * From these PartialMap objects the full real map is constructed.
 * 
 */
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

};

