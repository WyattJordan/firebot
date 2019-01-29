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
#include <unistd.h>

// for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <firebot/ReconConfig.h>
void callback(firebot::ReconConfig &config, uint32_t level){
	/*ROS_INFO("Reconfigure request (Kp,Ki,Kd,Min,Max: %f %f %f %f %f",
			config.double_param,config.double_param,
			config.double_param,config.double_param,
			config.double_param,config.double_param);
			*/
}

int main(int argc, char **argv){
	ros::init(argc,argv,"robot");

	float x=0, y=0;
	bool ways = false, run = false, big = false, small = false;
	if(argc >= 2){ run = atoi(argv[1]) == 1; }
	if(argc >= 4){ x = atof(argv[2]); y = atof(argv[3]);}
	if(argc >= 5){ ways = atoi(argv[4]) == 1;} 
	if(argc >= 7){ big = atoi(argv[5]) == 1; small = atoi(argv[6]) == 1;}

	ROS_INFO("running main launcher, going to create robot\n");
	Robot rob;
	rob.loadMapAndWayPoints(1); // working dir is the catkin workspace

	dynamic_reconfigure::Server<firebot::ReconConfig> server;
	dynamic_reconfigure::Server<firebot::ReconConfig>::CallbackType f;
	// callback recquires a function then an instance of the class (here Robot)
	f = boost::bind(&Robot::recon, &rob, _1, _2);
	server.setCallback(f);


	std::shared_ptr<Nav> ptr(rob.getNavPtr());
	std::cout<<"outputting map graph\n";
	ptr->outputGraph(*ptr->getMap());
	std::cout<<"outputting ways graph\n";
	ptr->outputGraph(*ptr->getWays());

	// code fore testing path planning
/*	std::cout<<"getting path between " << x <<" and "<<y<<"\n";
	vector<int> path = ptr->findPath(x, y, *ptr->getWays());
	std::cout<<"path is: \n";
	for(int i=0; i<path.size(); i++){
		std::cout<<path[i]<<"\n";
	}	
*/
	std::cout<<"\nnow go traverse\n";//*/
	std::thread thread1, driveLoop;
	
	rob.openI2C();
	driveLoop = std::thread(boost::bind(&Robot::driveLoop, &rob));	

//	thread1 = std::thread(boost::bind(&Robot::i2c, &rob));
	
	vector<EndPoint>* tmp = ways ? ptr->getWays() : ptr->getMap();  
	ptr->setRun(run);

	ptr->setBigRoomUpper(big);
	ptr->setSmallRoomUpper(small);
	//std::cout<<"before findExpected\n";
	//ptr->outputGraph(*ptr->getMap());

	if(ways){ 
		std::cout<<"using waypoints!\n";}
	else{
		//ptr->findExpected(x,y,*tmp); 
		std::cout<<"finding expected\n";
	}
/*
	if(run){
	thread1 = std::thread(bind(&Nav::run, ptr));
	}
	else{
	//thread1 = std::thread(bind(&Nav::publishGraph,ptr,x,y,"map_NS", *tmp));
	thread1 = std::thread(bind(&Nav::publishMapAndWays,ptr,x,y));
	}
*/
	ros::spin();
	return 0;
}
