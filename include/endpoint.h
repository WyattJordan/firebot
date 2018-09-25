/* endpoint.h
 * Simple class that stores the data for an endpoint on a map.
 *
 *
 *
 */

#pragma once
struct polar{
	float R, thet;
};

class Endpoint{
	private:
		float x,y;
		// type edge; // edge, edge/outer, inner classification?
	public:
		polar getPolarFrom(float Rx, float Ry); 
	
};

