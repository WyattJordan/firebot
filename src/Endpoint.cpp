#include "Endpoint.h"
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
	visible = done = false;
}	
float EndPoint::getR() const{
	return pp.R;
}

int   EndPoint::getNeighborID(int neighNum){
	if(neighNum > neighborIDs.size()) return -1;
	return neighborIDs[neighNum - 1];	
}

void EndPoint::getPolar(float Rx, float Ry, float theta){
	float diffx = x-Rx;
	float diffy = y-Ry;
	pp.R = pow(diffx*diffx + diffy*diffy, 0.5);
	pp.theta = theta - atan2(diffy, diffx) * 180 / 3.14159;	
}

float EndPoint::getx(){  return x;}
float EndPoint::gety(){  return y;}
int   EndPoint::getID(){ return id;}
int   EndPoint::getNumNeighbors(){ return neighborIDs.size();}
bool  EndPoint::isVisible() {return visible;}
void  EndPoint::setVisible(bool s) {visible = s;}
bool  EndPoint::getDone() {return done;}
void  EndPoint::setDone(bool d) {done = d;}
