/* Robot.h
 * Defines member variables for the main robot class. This class stores Maps
 * of it's environment, odometry data, indicator variables, etc.
 *
 *
 *
 *
 */

#pragma once
#include "map.h"
#include "ros/ros.h"
#include "nav.h"
#include <string>
using std::string;

class Robot{

	private:
	Nav beSmart;
	Map map;	
	ros::Publisher pubArduino;

	public:
	void lidarCallback(); // runs everytime a new lidar scan comes in
	void sendArduino(int code); // sends indicator info to arduino1
	void loadMap(int lvl);
	Map* getMapPtr();
	Nav* getNavPtr();
};
