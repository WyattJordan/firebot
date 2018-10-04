/* endpoint.h
 * Simple class that stores the data for an endpoint on a map.
 *
 *
 *
 */

#pragma once
class Map;
#include <vector>
using std::vector;
struct polar{
	float R, theta;
	polar(float r, float t) : R(r), theta(t) {}
};

class EndPoint{
	private:
		Map *parent;
		float x,y;
		// type edge; // edge, edge/outer, inner classification?
		int id; // unique from csv
		vector<int> neighborIDs;

	public:
		EndPoint();
		EndPoint(float X, float Y, int ID, vector<int> neighs);
		EndPoint(Map *Parent, float X, float Y, int ID, vector<int> neighs);
		polar getPolarFromRobot(float Rx, float Ry); 
		float getx();
		float gety();
		int getID();
		int getNumNeighbors();
		bool getNeighbor(int index, EndPoint &ep);
	
};

