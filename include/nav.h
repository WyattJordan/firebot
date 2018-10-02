
#pragma once
#include "map.h"
#include "endpoint.h"

class Nav{
	private:
		Map unversalMap;
		EndPoint safeZone, candle1, candle2; // key location markers
	public:

		void loadMap(); // read from file
		
		void findExpected(float Rx, float Ry, float theta);
};