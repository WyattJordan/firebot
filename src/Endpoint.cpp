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

int   EndPoint::getNeighborID(int neighNum) const{
	if(neighNum > neighborIDs.size()) return -1;
	return neighborIDs[neighNum - 1];	
}

void EndPoint::getPolar(float Rx, float Ry, float theta){
	float diffx = x-Rx;
	float diffy = y-Ry;
	pp.R = pow(diffx*diffx + diffy*diffy, 0.5);
	pp.theta = theta - atan2(diffy, diffx) * 180 / 3.14159;	
}

float EndPoint::getx()  const {  return x;}
float EndPoint::gety()  const {  return y;}
float EndPoint::getTheta() const{ return pp.theta;}
int   EndPoint::getID() const { return id;}
bool  EndPoint::getDone() const {return done;}
bool  EndPoint::isVisible() const {return visible;}
int   EndPoint::getNumNeighbors() const{ return neighborIDs.size();}
void  EndPoint::setDone(bool d) {done = d;}
void  EndPoint::setVisible(bool s) {visible = s;}

