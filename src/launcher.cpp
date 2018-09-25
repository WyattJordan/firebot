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
	
	return 0;
}
