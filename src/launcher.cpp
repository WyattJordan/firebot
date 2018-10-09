/* Launcher.cpp
 * Creates the main Robot object which handles pretty much everything.
 * Sets up all the publisher and subscriber methods for that Robot object.
 * May be responsible for launching some threads in the future.
 * Note: Robot obj may need to be warned when member data is being written
 * and read at the same time to avoid collisions!
 */

#include <iostream> 
#include "Robot.h"
#include <thread>     // 3 for thread and sharedptr
#include <functional>
#include <memory>
#include "ros/ros.h"
#include <pthread.h>
#include <ros/console.h>

int main(int argc, char **argv){
//	vector<ros::Publisher> navPub, mapPub;
	ros::init(argc,argv,"robot");

	ROS_INFO("running main launcher, going to create robot\n");
	Robot rob;

	rob.loadMap(1); // working dir is the catkin workspace
	
	std::shared_ptr<Nav> ptr(rob.getNavPtr());
	ptr->outputMapPoints();

	float x, y;
	//bool run = atoi(argv[1]) == 0; 
	if(argc == 3 ){ x = atof(argv[1]); y = atof(argv[2]);}
	else{x = 0; y = 0;}
//	std::thread thread1(bind(&Nav::run, ptr));
	std::thread thread1(bind(&Nav::publishMap,ptr, x,y));
	/*if(run){
		std::thread thread1(bind(&Nav::run, ptr));
	}
	else{
		std::thread thread1(bind(&Nav::publishMap,ptr, x,y));
	}*/

	ros::spin();
	return 0;
}
