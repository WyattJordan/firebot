/* Robot.cpp
 *
 */

#include "Robot.h"
#include "Nav.h"
#include <string>
#include <iostream>
#include <fstream>
using std::string;

void Robot::sendArduino(int code){
	std::cout<<"the given code was: "<<code<<" \n";
}

void Robot::loadMapAndWayPoints(int lvl){
	if(lvl == 3){
		Nav tmp("lvl3_map.txt", "dummy");  //root is catkin ws
		beSmart = tmp;
	}	
	else {

		Nav tmp("/home/wyatt/cat_ws/src/firebot/lvl1_map.txt", 
				"/home/wyatt/cat_ws/src/firebot/wayPoints.txt");	
		beSmart = tmp;
	}
}

void Robot::serial(){
	std::cout<<"in thread. waiting for USB connection...\n";
	std::ifstream file("/dev/ttyUSB0");
	string line;
	if(file.is_open()){

		while(getline(file,line)){
			std::cout<<line<<"\n";
			std::cout<<"testing "<<tmp;
		}
	}
}

Nav* Robot::getNavPtr(){ return &beSmart;}
