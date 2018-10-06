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
	polar(){ R = 0; theta = 0;}
	polar(float r, float t) : R(r), theta(t) {}
};

class EndPoint{
	private:
		float x,y;
		polar pp;
		// type edge; // edge, edge/outer, inner classification?
		int id; // unique from csv
		vector<int> neighborIDs;

	public:
		EndPoint();
		EndPoint(float X, float Y, int ID, vector<int> neighs);
		void getPolar(float Rx, float Ry); 
		float getx();
		float gety();
		int getID();
		int getNumNeighbors();
		int getNeighborID(int neighNum);
//		bool IDLess(const EndPoint &rhs) const;	
//		bool IDGreater(const EndPoint &rhs) const;	
		//static bool RLess(const EndPoint &lhs, const EndPoint &rhs) const;	
		//
//		static bool RLess(const EndPoint &lhs,const EndPoint &rhs) {
//		  return lhs.pp.R<rhs.pp.R;
//		  }
	
//		bool RGreater(const EndPoint &rhs) const;	
		float getR() const;
};

