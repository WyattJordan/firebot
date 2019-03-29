#ifndef REFPOINT_INCLUDE
#define REFPOINT_INCLUDE

#include <iostream>
#include <cmath>
#include <vector>
#include <utility>

using namespace std;

class refpoint {
public:
	void setCart(float xIn, float yIn);
	float getX();
	float getY();
	float findAngle();
	float findRad();
	void clear();
private:
	float x;
	float y;
};


#endif


