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
#include "pid.h"

#define addrDrive 17  // I2C slave addresses
#define addrArm   16
using std::string;
static void pabort(const char *s)
{
	perror(s);
	abort();
}

class Robot{

	public:
		Robot();
		Nav* getNavPtr();
		void lidarCallback(); // runs everytime a new lidar scan comes in
		void loadMapAndWayPoints(int lvl);
		void i2c();
		void openI2C();
		void contactDrive();
		string tmp;
		int failed_reads, contacts, left255, right255;

	private:
		Nav beSmart;
		PID posePID;
		int fd; // file descriptor for I2C port
		float lDrive, rDrive;     // drive power levels -1:1
		unsigned char lPWM, rPWM; // drive PWMs 0:255
		int16_t lEnc, rEnc;      // enc counts 0:65535
		bool usingi2c;            // avoid conflicting contacts	
		int maxleft, maxright;
		void power2pwm();
		void checki2c();
};
