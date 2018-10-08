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
#include <ros/console.h>

int main(int argc, char **argv){
//	vector<ros::Publisher> navPub, mapPub;
	ros::init(argc,argv,"robot");

	ROS_INFO("running main launcher, going to create robot\n");
	Robot rob;
	rob.sendArduino(3);

	vector<int> temp;	
	EndPoint compilethis(1, 1, 342, temp);
	std::cout<<"made an EndPoint, testing polar conversion: \n";
	std::cout<<"x: "<<compilethis.getx();
	std::cout<<" y: "<<compilethis.gety();

	rob.loadMap(1); // working dir is the catkin workspace
	
	std::shared_ptr<Nav> ptr(rob.getNavPtr());
	ptr->outputMapPoints();

	float x, y;
	if(argc == 3){ x = atof(argv[1]); y = atof(argv[2]);}
	else{x = 0; y = 0;}
	/*
	y = 30;
	unsigned int sleep = 100000;
	for( int i = 0; i<1000; i++){
		x = 230.0/1000.0 * i;
		ptr->publishMap(x,y);
		usleep(sleep);
	}	
	x = 230;
	for( int i = 0; i<1000; i++){
		y = 230.0/1000.0 * i;
		ptr->publishMap(x,y);
		usleep(sleep);
	}	
	y = 230;
	for( int i = 0; i<1000; i++){
		x = 230.0/1000.0 * (1000 - i);
		ptr->publishMap(x,y);
		usleep(sleep);
	}	
	x = 30;
	for( int i = 0; i<1000; i++){
		y = 230.0/1000.0 * (1000 - i);
		ptr->publishMap(x,y);
		usleep(sleep);
	}*/	
	
	std::thread thread1(bind(&Nav::publishMap,ptr, x,y));

	ros::spin();
	return 0;
}
