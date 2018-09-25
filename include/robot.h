/* Robot.h
 * Defines member variables for the main robot class. This class stores Maps
 * of it's environment, odometry data, indicator variables, etc.
 *
 *
 *
 *
 */

#pragma once
#include "maps.h"
#include "ros/ros.h"

class Robot{

	private:
	Maps maps;	
	ros::Publisher pubArduino;

	public:
	void lidarCallback(); // runs everytime a new lidar scan comes in
	void sendArduino(int code); // sends indicator info to arduino1

};
