
#include "ros/ros.h"
#include "lidar.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#include "line.h"
#include "endpoint.h"
#include <vector>
#include <ctime>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rplidar_node_client");
	ros::NodeHandle n;

	Lidar lid;

//	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

	ros::spin();

	return 0;
}

