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
#define WheelLCM  13.75 
#define WheelRCM  12.3 
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
		void openI2C();
		bool getEncoders();
		bool setMotors();
		bool contactArms();
		string tmp;
		int failed_reads, failed_writes, contacts, left255, right255;

	private:
		Nav beSmart;
		PID posePID;
		float setPose, error;
		float odomPose, odomX, odomY;
		float locXinR, locYinR, locPinR;
	       	double kp_, ki_, kd_, min_, max_, dt_; 
		int fd; // file descriptor for I2C port
		float lDrive, rDrive;     // drive power levels -1:1
		
		unsigned char lPWM, rPWM; // drive PWMs 0:255
		unsigned char D3, D6, D9, D10, D11; // PWMs 0:255 for arms
		unsigned char lforward, rforward;
		int16_t lEnc, rEnc;       // enc counts 0:65535
		bool usingi2c;            // avoid conflicting contacts	
		bool runPID;
		int maxleft, maxright;

		void calculateOdom();
		void power2pwm();
		void checki2c();
		void quei2c(int size, unsigned char *q);
};
