/* partialmap.h
 * Stores a single piece of a map. Some maps may be offset a set amount to 
 * represent the 2nd arena across the hallway. That or they may be ignored
 *
 *
 *
 */
#pragma once
#include "endpoint.h"
#include <vector>
#include <utility>

using std::vector;
using std::pair;
class PartialMap{
	private:
		vector<EndPoint> mapPoints;
		vector<int> expectedMarkerIdxs;
		vector<pair< float,float> > polarPoints;
	public:

};
