
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "lidar.h"
#include "Robot.h"
#include "math.h"
#include "line.h"
#include "Endpoint.h"
#include <vector>
#include <ctime>
#include <thread>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rplidar_node_client");
	ros::NodeHandle n;

	Robot rob; // Nav class needs a pointer, not really using this
	ros::Publisher navPub = n.advertise<visualization_msgs::MarkerArray>("NavMarkers",1000);
	Nav nav(1, &navPub, &rob); // level 1	
	nav.makeMapMarks("marker_ns"); // make initial sets for publishing
	nav.makeWayMarks("ways_ns");
	std::thread publishNavLoop, input;

	// publish whatever markers are made every second (markers changed by lidar class sending furniture and lines to it)
	publishNavLoop = std::thread(boost::bind(&Nav::publishLoopContinual, &nav));	

	Lidar lid;
	lid.setNav(&nav);

	input = std::thread(boost::bind(&Lidar::input, &lid));	
	cout<<"started input\n";

	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &Lidar::scanCallback, &lid);

	ros::spin();

	return 0;
}

