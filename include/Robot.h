/* Robot.h
 * Defines member variables for the main robot class. This class stores Maps
 * of it's environment, odometry data, indicator variables, etc.
 *
 *
 *
 *
 */

#pragma once
#include "ros/ros.h"
#include "Nav.h"
#include <string>
using std::string;
static void pabort(const char *s)
{
	perror(s);
	abort();
}

class Robot{

	private:
		Nav beSmart;

	public:
		void lidarCallback(); // runs everytime a new lidar scan comes in
		void sendArduino(int code); // sends indicator info to arduino1
		void loadMapAndWayPoints(int lvl);
		Nav* getNavPtr();
		void serial(char send[], int size);
		void spi();
		void i2c();
		string tmp;
};
