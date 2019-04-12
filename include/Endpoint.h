/* Endpoint.h
 * Simple class that stores the data for an endpoint on a map.
 *
 *
 *
 */

#pragma once
#include <vector>
#include "definitions.h"
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
		bool visible, done;

	public:
		EndPoint();
		EndPoint(const EndPoint &ep2);
		EndPoint(float X, float Y, int ID, vector<int> neighs);
		EndPoint(float X, float Y);
		void calcPolar(float Rx, float Ry); 
		int getNumNeighbors() const;
		int getNeighborID(int neighI) const;
		vector<int> getNeighborList() const;
		void setNeighbors(int n, ...);
		//void setNeighbors(int n1, int n2);
		float distBetween(EndPoint ep);
		float distBetween(float X, float Y);

		int getID() const;
		float getX() const;
		float getY() const;
		float getCalculatedR() const;
		float getCalculatedTheta() const;
		bool getDone() const;
		void setDone(bool d);
		bool isVisible() const;
		void setVisible(bool s);

		// migrated from refpoint
		void setCart(float xIn, float yIn);
		float findAngle() const;
		float findRad() const;
		void clear();
};

