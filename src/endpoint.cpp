#include "endpoint.h"
#include <utility>
#include <cmath>

EndPoint::EndPoint(){

}

EndPoint::EndPoint(float X, float Y, int ID, vector<int> neighs)
: x(X), y(Y), id(ID)
{
	for(int i=0; i<neighs.size(); i++){
		neighborIDs.push_back(neighs[i]);
	}
}	

polar EndPoint::getPolarFromRobot(float Rx, float Ry){
	float diffx = x-Rx;
	float diffy = y-Ry;
	float R = pow(diffx*diffx + diffy*diffy, 0.5);
	float theta = atan2(diffy, diffx) * 180 / 3.14159;	
	return polar(R,theta);
}

float EndPoint::getx(){ return x;}
float EndPoint::gety(){ return y;}

