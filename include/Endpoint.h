/* Endpoint.h
 * Simple class that stores the data for an endpoint on a map.
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
		float x,y; 		// location
		polar pp;  		// polar representation
		int id;  	  	// unique to vector 
		vector<int> neighborIDs; // vector of connected pts
	   	bool done; 		// to mark if checked for visibility 
		bool visible;   // to mark if visible to robot from location


	public:
		// constructors
		EndPoint();
		EndPoint(const EndPoint &ep2);
		EndPoint(float X, float Y, int ID, vector<int> neighs);
		EndPoint(float X, float Y);

		// Set pp member given an origin in the world frame.
		// Defaults to finding as if it's in the same frame as its x,y 
		void calcPolar(float Rx=0, float Ry=0); 

		// Returns length(neighborIDs)
		int getNumNeighbors() const;
		// Returns ID of the neighbor at idx neighI
		int getNeighborID(int neighI) const;
		// Returns the entire neighborIDs variable
		vector<int> getNeighborList() const;
		// Sets all the neighbors, variable length parameter list
		void setNeighbors(int n, ...);
		// Return distance from this to ep
		float distBetween(EndPoint ep);
		// Return dist from this to location (X,Y)
		float distBetween(float X, float Y);

		// Getters
		int getID() const;
		float getX() const;
		float getY() const;
		float getCalculatedR() const; 		// returns pp.R
		float getCalculatedTheta() const;	// returns pp.theta
		bool getDone() const;
		bool isVisible() const;
		// Setters
		void setDone(bool d);
		void setVisible(bool s);
		void setCart(float xIn, float yIn);

		// migrated from refpoint
		float findAngle() const; 
		float findRad() const;
		void clear();
};
