#include "Endpoint.h"
#include <utility>
#include <cmath>
#include <iostream>
#include <stdarg.h>

EndPoint::EndPoint(){
	
}
EndPoint::EndPoint(const EndPoint &ep2){
	x = ep2.x;
	y = ep2.y;
	id = ep2.id;
	visible = ep2.visible;
	done = ep2.done;
	pp = ep2.pp;
	neighborIDs.resize(0);
	for(int i=0; i<ep2.neighborIDs.size(); i++){
		neighborIDs.push_back(ep2.neighborIDs[i]);
	}
}

vector<int> EndPoint::getNeighborList() const{
	return neighborIDs;
}

EndPoint::EndPoint(float X, float Y, int ID, vector<int> neighs)
: x(X), y(Y), id(ID)
{
	for(int i=0; i<neighs.size(); i++){
		neighborIDs.push_back(neighs[i]);
	}
	visible = done = false;
}

void EndPoint::setNeighbors(int n, ...){
//void EndPoint::setNeighbors(int n1, int n2){

	va_list neighs;
	va_start(neighs, n);
	neighborIDs.resize(0);
	for(int i=0; i<n; i++){
		int t = va_arg(neighs,int);
		neighborIDs.push_back(t);
	}
}

int EndPoint::getNeighborID(int neighI) const{
	if(neighI > neighborIDs.size() - 1) return -1;
	return neighborIDs[neighI];	
}

void EndPoint::calcPolar(float Rx, float Ry){
	float diffx = x-Rx;
	float diffy = y-Ry;
	pp.R = pow(diffx*diffx + diffy*diffy, 0.5);
	float t = atan2(diffy, diffx) * 180 / 3.14159;
	pp.theta = t<0 ? t + 360 : t; // polar vals range 0:360
}

float EndPoint::getx()  const {  return x;}
float EndPoint::gety()  const {  return y;}
float EndPoint::getTheta() const{ return pp.theta;}
float EndPoint::getR() const{ return pp.R; }
int   EndPoint::getID() const { return id;}
bool  EndPoint::getDone() const {return done;}
bool  EndPoint::isVisible() const {return visible;}
int   EndPoint::getNumNeighbors() const{ return neighborIDs.size();}
void  EndPoint::setDone(bool d) {done = d;}
void  EndPoint::setVisible(bool s) {visible = s;}

