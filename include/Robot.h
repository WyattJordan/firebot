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

#include <firebot/ReconConfig.h>

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
		void driveLoop();
		void recon(firebot::ReconConfig &config, uint32_t level);
		Nav* getNavPtr();
		void lidarCallback(); // runs everytime a new lidar scan comes in
		void loadMapAndWayPoints(int lvl);
		void i2c();
		void openI2C();
		bool contactDrive();
		bool contactArms();
		string tmp;
		int failed_reads, contacts, left255, right255;

	private:
		Nav beSmart;
		PID posePID;
		int fd; // file descriptor for I2C port
		float lDrive, rDrive;     // drive power levels -1:1
		unsigned char lPWM, rPWM; // drive PWMs 0:255
		unsigned char D3, D6, D9, D10, D11; // PWMs 0:255 for arms
		int16_t lEnc, rEnc;       // enc counts 0:65535
		bool usingi2c;            // avoid conflicting contacts	
		int maxleft, maxright;
		void power2pwm();
		void checki2c();
		void quei2c(int size, unsigned char *q);
};
