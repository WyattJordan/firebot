/* Robot.cpp
 *
 */

#include "Robot.h"
#include "Nav.h"
#include <string>
#include <iostream>
using std::string;

void Robot::sendArduino(int code){
	std::cout<<"the given code was: "<<code<<" \n";
}

void Robot::loadMap(int lvl){
	if(lvl == 3){
		Nav tmp("lvl3_map.txt");  //root is catkin ws
		beSmart = tmp;
	}	
	else {
		Nav tmp("/home/wyatt/cat_ws/lvl1_map.txt");	
		beSmart = tmp;
	}
}

Nav* Robot::getNavPtr(){ return &beSmart;}
