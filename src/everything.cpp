/* everything.cpp */

#include "ros/ros.h"
#include "Robot.h"
#include "Nav.h"
#include "lidar.h"
#include <ros/console.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/MarkerArray.h>
#include <thread>     // 3 for thread and sharedptr
#include <iostream> 

using std::cout;

// RUNNING EVERYTHING
int main(int argc, char **argv){
	ros::init(argc,argv,"robot"); // start the node with name "robot"

	ROS_INFO("running main launcher, going to create robot\n");
	Robot rob;

	// Create the Navigation object and set it up
	ros::NodeHandle n;
	ros::Publisher navPub = n.advertise<visualization_msgs::MarkerArray>("NavMarkers",1000);
	Nav nav(1, &navPub, &rob); // level 1	
	rob.setNav(&nav); // give the robot the nav object so they can chit chat
	nav.makeMapMarks("marker_ns"); // make initial sets for publishing
	nav.makeWayMarks("ways_ns");
//	nav.outputMap();
	nav.setSmallRoomUpper(false);
	nav.setBigRoomUpper(true);
	cout<<"made nav object and linked to rob\n";
	
	// Create and link lidar class
	Lidar lid(&rob, &nav);
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &Lidar::scanCallback, &lid);

	// Spin off threads
	std::thread mainLogic, driveLoop, publishNavLoop, pubTrans, protoThread;
	// loop for publishing markers in rviz and to transform them appropriately, only for visualization
	publishNavLoop = std::thread(boost::bind(&Nav::publishLoop, &nav));	
	//publishNavLoop = std::thread(boost::bind(&Nav::publishLoopContinual, &nav));	
	pubTrans = std::thread(boost::bind(&Robot::pubTransformContinual, &rob, 10)); // at 10 Hz

	// loop for controlling motors w/ PID and odometry math
	//driveLoop = std::thread(boost::bind(&Robot::driveLoop, &rob));	

	// loop for talking to odroid board
	//protoThread = std::thread(boost::bind(&Robot::pinThread, &rob));	
		
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

