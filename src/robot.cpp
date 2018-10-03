#include "robot.h"
#include "nav.h"
/*#include <string>
#include <iostream>
using std::string;
*/
void Robot::sendArduino(int code){
	std::cout<<"the given code was: "<<code<<" \n";
}

void Robot::loadMap(string file){
	beSmart.loadMap(file);
}
