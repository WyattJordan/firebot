/* "nav.h" 
 *
 *
 */
#pragma once
#include "map.h"
#include "endpoint.h"

class Nav{
	private:
		Map universalMap;
		EndPoint safeZone, candle1, candle2; // key location markers
	public:
		void loadMap(string file); // read from file
		void findExpected(float Rx, float Ry, float theta);
		Map* getMap();
};
