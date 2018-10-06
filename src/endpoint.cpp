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
/*
bool EndPoint::IDLess(const EndPoint &rhs) const {
	return id<rhs.id;
}

bool EndPoint::IDGreater(const EndPoint &rhs) const {
	return id>rhs.id;
}
*/

/*static  bool EndPoint::RLess(const EndPoint &lhs,const EndPoint &rhs) const {
	return lhs.pp.R<rhs.pp.R;
}*/

float EndPoint::getR() const{
	return pp.R;
}
/*
bool EndPoint::RGreater(const EndPoint &rhs) const {
	return pp.R>rhs.pp.R;
}*/
void EndPoint::getPolar(float Rx, float Ry){
	float diffx = x-Rx;
	float diffy = y-Ry;
	pp.R = pow(diffx*diffx + diffy*diffy, 0.5);
	pp.theta = atan2(diffy, diffx) * 180 / 3.14159;	
}

float EndPoint::getx(){  return x;}
float EndPoint::gety(){  return y;}
int   EndPoint::getID(){ return id;}
int   EndPoint::getNumNeighbors(){ return neighborIDs.size();}
int   EndPoint::getNeighborID(int neighNum){
	if(neighNum > neighborIDs.size()) return -1;
	return neighborIDs[neighNum - 1];	
}

