/* Launcher.cpp
 * Creates the main Robot object which handles pretty much everything.
 * Sets up all the publisher and subscriber methods for that Robot object.
 * May be responsible for launching some threads in the future.
 * Note: Robot obj may need to be warned when member data is being written
 * and read at the same time to avoid collisions!
 *
 * arguments are:
 * run {x, y} ways {big, small}
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
	rob.tmp = "specific to class!!\n";
	std::thread thread1;
	std::cout<<"starting thread\n";

//	thread1 = std::thread(boost::bind(&Robot::i2c, &rob));
//	thread1 = std::thread(boost::bind(&Robot::serial, &rob));
//	thread1 = std::thread(boost::bind(&Robot::spi, &rob));

	
	rob.loadMapAndWayPoints(1); // working dir is the catkin workspace
	
	std::shared_ptr<Nav> ptr(rob.getNavPtr());
	std::cout<<"outputting map graph\n";
	ptr->outputGraph(*ptr->getMap());
	std::cout<<"outputting ways graph\n";
	ptr->outputGraph(*ptr->getWays());


	float x=0, y=0;
	bool ways = false, run = false, big = false, small = false;
	if(argc >= 2){ run = atoi(argv[1]) == 1; }
	if(argc >= 4){ x = atof(argv[2]); y = atof(argv[3]);}
	if(argc >= 5){ ways = atoi(argv[4]) == 1;} 
	if(argc >= 7){ big = atoi(argv[5]) == 1; small = atoi(argv[6]) == 1;}

	vector<EndPoint>* tmp = ways ? ptr->getWays() : ptr->getMap();  
	ptr->setRun(run);

	// set map config, find expected marks
	//std::cout<<"before room config\n";
	//ptr->outputGraph(*ptr->getMap());

	ptr->setBigRoomUpper(big);
	ptr->setSmallRoomUpper(small);
	//std::cout<<"before findExpected\n";
	//ptr->outputGraph(*ptr->getMap());


	if(ways){ 
		std::cout<<"using waypoints!\n";}
	else{
		ptr->findExpected(x,y,*tmp); 
		std::cout<<"finding expected\n";
	}

	if(run){
	thread1 = std::thread(bind(&Nav::run, ptr));
	}
	else{
	thread1 = std::thread(bind(&Nav::publishGraph, ptr, x, y, "map_NS", *tmp));
	}

	ros::spin();
	return 0;
}
