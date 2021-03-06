/* Launcher.cpp
  Creates the main Robot object which handles pretty much everything.
 * Sets up all the publisher and subscriber methods for that Robot object.
 * May be responsible for launching some threads in the future.
 * Note: Robot obj may need to be warned when member data is being written
 * and read at the same time to avoid collisions!
 *
 * arguments are:
 * run {x, y} ways {big, small}
 */

#include "ros/ros.h"
#include "Robot.h"
#include <ros/console.h>
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/MarkerArray.h>
#include <thread>     // 3 for thread and sharedptr
#include <iostream> 
using std::cout;
void setupServer(Robot rob);

int main(int argc, char **argv){
	ros::init(argc,argv,"robot"); // start the node with name "robot"

	// handle input arguments, p deprecated at this point
	float x=0, y=0;
	bool ways = false, run = false, big = false, small = false;
	if(argc >= 2){ run = atoi(argv[1]) == 1; }
	if(argc >= 4){ x = atof(argv[2]); y = atof(argv[3]);}
	if(argc >= 5){ ways = atoi(argv[4]) == 1;} 
	if(argc >= 7){ big = atoi(argv[5]) == 1; small = atoi(argv[6]) == 1;}

	ROS_INFO("running main launcher, going to create robot\n");
	Robot rob;

	// Create the Navigation object and set it up
	ros::NodeHandle n;
	ros::Publisher navPub = n.advertise<visualization_msgs::MarkerArray>("NavMarkers",1000);
	Nav nav(1, &navPub,&rob); // level 1	
	rob.setNav(&nav); // give the robot the nav object so they can chit chat
	nav.makeMapMarks("marker_ns"); // make initial sets for publishing
	nav.makeWayMarks("ways_ns");
	//nav.outputWays();
	//nav.setBigRoomUpper(big);
	//nav.setSmallRoomUpper(small);
	cout<<"made nav object and linked to rob\n";


	// Spin off threads
	std::thread mainLogic, driveLoop, publishNavLoop, protoThread;
	// loop for publishing marker arrays
	publishNavLoop = std::thread(boost::bind(&Nav::publishLoop, &nav));	
	//publishNavLoop = std::thread(boost::bind(&Nav::publishLoopContinual, &nav));	
	// loop for controlling motors w/ PID and odometry math
	driveLoop = std::thread(boost::bind(&Robot::driveLoop, &rob));	

	// loop for talking to odroid board
	//protoThread = std::thread(boost::bind(&Robot::pinThread, &rob));	
		
/*	while(1){
		nav.pubMap_ = true;
		nav.pubWays_ = true;
		sleep(1);
	}
	*/

	// Countdown with delay before starting mainLogic loop
	for(int i=0; i<3; i++){
		cout<<"start in "<<3-i<<"...\n";
		usleep(1000*500); // half second
		//sleep(1);
	}
	cout<<"\nGO!\n";
	mainLogic = std::thread(boost::bind(&Robot::mainLogic, &rob));//*/

	ros::spin();
	return 0;
}
