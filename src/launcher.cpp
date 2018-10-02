/* Launcher.cpp
 * Creates the main Robot object which handles pretty much everything.
 * Sets up all the publisher and subscriber methods for that Robot object.
 * May be responsible for launching some threads in the future.
 * Note: Robot obj may need to be warned when member data is being written
 * and read at the same time to avoid collisions!
 */

#include <iostream> 
#include "robot.h"


int main(){
	
	Robot rob;

	std::cout<<"running main launcher, going to create robot\n";
	rob.sendArduino(3);
	
	vector<int> temp;
	vector<float> xs, ys, Rx, Ry;
	xs.push_back(12.3);
	xs.push_back(-34.3);
	xs.push_back(3.34);
	xs.push_back(-12.3);
	xs.push_back(-34.3);
	xs.push_back(12.3);
	xs.push_back(-34.3);
	xs.push_back(3.34);
	xs.push_back(-12.3);
	xs.push_back(12.3);
	xs.push_back(-34.3);
	xs.push_back(3.34);
	xs.push_back(-12.3);

	ys.push_back(3.34);
	ys.push_back(123.2);
	ys.push_back(-34.3);
	ys.push_back(3.34);
	ys.push_back(-45.8);
	ys.push_back(-22.3);
	ys.push_back(4.3);
	ys.push_back(3.34);
	ys.push_back(3.34);
	ys.push_back(123.2);
	ys.push_back(-34.3);
	ys.push_back(3.34);
	ys.push_back(45.3);

	Rx.push_back(0);
	Rx.push_back(0);
	Rx.push_back(0);
	Rx.push_back(0);
	Rx.push_back(4);
	Rx.push_back(6);
	Rx.push_back(-3);
	Rx.push_back(-12);
	Rx.push_back(23);
	Rx.push_back(10);
	Rx.push_back(-20);
	Rx.push_back(-340);
	Rx.push_back(-340);

	Ry.push_back(0);
	Ry.push_back(1);
	Ry.push_back(0);
	Ry.push_back(0);
	Ry.push_back(-4);
	Ry.push_back(6);
	Ry.push_back(-3);
	Ry.push_back(12);
	Ry.push_back(-23);
	Ry.push_back(10);
	Ry.push_back(-20);
	Ry.push_back(340);
	Ry.push_back(-340);




	for(int i=0; i<xs.size(); i++){
		EndPoint compilethis(xs[i], ys[i], 342, temp);
//		std::cout<<"\n\nmade an EndPoint, testing polar conversion: \n";
		std::cout<<"x: "<<compilethis.getx();
		std::cout<<" y: "<<compilethis.gety();
		
		polar ptemp = compilethis.getPolarFromRobot(Rx[i],Ry[i]);
		std::cout<<" theta: "<<ptemp.theta;
		std::cout<<" R: " << ptemp.R << "\n";	
		}
	return 0;
}
