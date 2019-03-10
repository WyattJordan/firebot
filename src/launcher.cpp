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

#include "ros/ros.h"
#include <ros/console.h>
#include "Robot.h"
#include <firebot/ReconConfig.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>     // 3 for thread and sharedptr
#include <iostream> 
using std::cout;

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

	
	// load the dynamic reconfigure server
	dynamic_reconfigure::Server<firebot::ReconConfig> server;
	dynamic_reconfigure::Server<firebot::ReconConfig>::CallbackType f;
	// callback recquires a function then an instance of the class (here Robot)
	f = boost::bind(&Robot::recon, &rob, _1, _2);
	server.setCallback(f);
	
	cout<<"setup server\n";

	// load the maps, output maps to console
	ros::NodeHandle n;
	ros::Publisher navPub = n.advertise<visualization_msgs::MarkerArray>("NavMarkers",1000);
	
	Nav nav(1, &navPub); // level 1	
	nav.outputMap();
	nav.outputWays();
	cout<<"outputting ways graph\n";
	rob.setNav(&nav); // give the robot the nav object for interacting with it

	cout<<"\now go traverse\n";//*/
	std::thread thread1, driveLoop, publishNavLoop;
//	rob.openI2C();
	rob.openSerial();
	rob.setSerialMotors();
	sleep(8);
	
	driveLoop = std::thread(boost::bind(&Robot::driveLoop, &rob));	

	nav.makeMapMarks("marker_ns");
	nav.makeWayMarks("ways_ns");
//	publishNavLoop = std::thread(boost::bind(&Nav::publishLoop, &nav));	
	
	nav.setBigRoomUpper(big);
	nav.setSmallRoomUpper(small);
	//cout<<"before findExpected\n";

	if(ways){ 
		cout<<"using waypoints!\n";}
	else{
		cout<<"finding expected\n";
	}

	ros::spin();
	return 0;
}
