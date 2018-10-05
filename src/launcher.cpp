/* Launcher.cpp
 * Creates the main Robot object which handles pretty much everything.
 * Sets up all the publisher and subscriber methods for that Robot object.
 * May be responsible for launching some threads in the future.
 * Note: Robot obj may need to be warned when member data is being written
 * and read at the same time to avoid collisions!
 */

#include <iostream> 
#include "robot.h"
#include <thread>     // 3 for thread and sharedptr
#include <functional>
#include <memory>
#include "ros/ros.h"

int main(int argc, char **argv){
//	vector<ros::Publisher> navPub, mapPub;
	ros::init(argc,argv,"robot");

	std::cout<<"\nrunning main launcher, going to create robot\n";
	Robot rob;
	rob.sendArduino(3);

	vector<int> temp;	
	EndPoint compilethis(1, 1, 342, temp);
	std::cout<<"made an EndPoint, testing polar conversion: \n";
	std::cout<<"x: "<<compilethis.getx();
	std::cout<<" y: "<<compilethis.gety();
	polar ptemp = compilethis.getPolarFromRobot(0,0);
	std::cout<<" theta: "<<ptemp.theta;
	std::cout<<" R: " << ptemp.R << "\n";	

	rob.loadMap(1); // working dir is the catkin workspace
	std::shared_ptr<Nav> ptr(rob.getNavPtr());
	ptr->outputMapPoints();
	std::thread thread1(bind(&Nav::publishMap,ptr));

	ros::spin();
	return 0;
}
