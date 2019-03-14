#include "notLine.h"
#include <iostream>


using namespace std;

void notLine::addFront(float xVal, float yVal){
	x.pushback(xVal);
	y.pushback(yVal);
}
void notLine::addRear(float xVal, float yVal) { 
	x.insert(x.begin(), xVal);
	y.insert(y.begin(), yVal);
}

void setEndpoints(){
	end1.setCart(x[0], y[0]);
	end2.setCart(x[x.size()-1], y[y.size()-1]);
}

int notLine::getSize(){
	return x.size();
}
void notLine::reverseNL(){
	reverse(x.begin(), x.end());
	reverse(y.begin(), y.end());
}
float notLine::getXPoint(int i){
	return x[i];
}
float notLine::getYPoint(int i){
	return y[i];
}
void notLine::clearNL(){
	x.clear();
	y.clear();
	end1.clear();
	end2.clear();
}
void notLine::clearPoint(int i){
	x.erase(x.begin() + i);
	y.erase(y.begin() + i);
}
float notLine::getEndPtX1(){
	return end1.getX();
}
float notLine::getEndPtY1(){
	return end1.getY();
}
float notLine::getEndPtX2(){
	return end2.getX();
}
float notLine::getEndPtY2(){
	return end2.getY();
}
void notLine::print(){
	cout << "Point count: " << x.size() << endl;
	cout << "First endpoint cartesian:  (" << end1.getX() << ", " << end1.getY() << ")" << endl;
	cout << "Second endpoint cartesian: (" << end2.getX() << ", " << end2.getY() << ")" << endl;
	cout << endl << "First endpoint polar:   R: " << end1.findRad() << " Angle: " << end1.findAngle() endl;
	cout << "Second endpoint polar:  R: " << end2.findRad() << " Angle: " << end2.findAngle() endl;
	cout << "Points" << endl;
	for(int i = 0; i < x.size(); i++){
		cout << "(" << x[i] << ", " << y[i] << ")" << endl;
	}
}
void notLine::lineToNL(line a){
	for(int i = 0; i < a.lineSize(); i++){
		x.push_back(a.getXPoint(i));
		y.push_back(a.getYPoint(i));
		//x.insert(x.begin(), a.getXPoint(a.lineSize() - i));
		//y.insert(y.begin(), a.getYPoint(a.lineSize() - i));
		//still have to delete the line a in findLine
	}
	a.clearLine();
}

