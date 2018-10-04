/* endpoint.h
 * Simple class that stores the data for an endpoint on a map.
 *
 *
 *
 */

#pragma once
#include <vector>
using std::vector;
struct polar{
	float R, theta;
	polar(float r, float t) : R(r), theta(t) {}
};

class EndPoint{
	private:
		float x,y;
		// type edge; // edge, edge/outer, inner classification?
		int id; // unique from csv
		vector<int> neighborIDs;

	public:
		EndPoint();
		EndPoint(float X, float Y, int ID, vector<int> neighs);
		EndPoint getBadPoint();	
		polar getPolarFromRobot(float Rx, float Ry); 
		float getx();
		float gety();
		int getID();
		int getNumNeighbors();
		int getNeighborID(int neighNum);
	
};

