#include "endpoint.h"
#include "map.h"
#include <utility>
#include <cmath>

EndPoint::EndPoint(){

}
EndPoint::EndPoint(float X, float Y, int ID, vector<int> neighs)
: x(X), y(Y), id(ID)
{
	parent = NULL;
	for(int i=0; i<neighs.size(); i++){
		neighborIDs.push_back(neighs[i]);
	}
}	


EndPoint::EndPoint(Map *Parent, float X, float Y, int ID, vector<int> neighs)
: parent(Parent), x(X), y(Y), id(ID)
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

float EndPoint::getx(){  return x;}
float EndPoint::gety(){  return y;}
int   EndPoint::getID(){ return id;}
int   EndPoint::getNumNeighbors(){ return neighborIDs.size();}
bool EndPoint::getNeighbor(int index, EndPoint &ep){ 
	if(neighborIDs.size() >= index && parent!=NULL){
		 ep = parent->getPoint(neighborIDs[0]);	
		 return true;
	}	
	else {
		Map tmp;
		ep = tmp.getBadPoint(); 
		return false;
	}
}

