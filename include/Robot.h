/* Robot.h
 * Manages all sensor inputs and outputs, tons of settings and thresholds.
 * Controls drive motors and servo motors (interfaced through arduino).
 * PID loop for driving and odometry calculations.
 */

#pragma once
#include "ros/ros.h"
#include "Nav.h"
#include "pid.h"
#include <firebot/ReconConfig.h>
#include <Eigen/Core>
#include <string>

#define addrDrive 17  // I2C slave addresses
#define addrArm   16
#define WheelLCM  13.75 
#define WheelRCM  12.3 
using std::string;
using namespace Eigen;
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
		void lidarCallback(); // runs everytime a new lidar scan comes in
		void openI2C();
		bool getEncoders();
		bool setMotors();
		bool contactArms();
		string tmp;
		int failed_reads, failed_writes, contacts, left255, right255;

	private:
		PID posePID;
		float setPose, error;
		double kp_, ki_, kd_, min_, max_, dt_; 
		int fd; // file descriptor for I2C port
		float lDrive, rDrive;     // drive power levels -1:1
		
		unsigned char lPWM, rPWM; // drive PWMs 0:255
		unsigned char D3, D6, D9, D10, D11; // PWMs 0:255 for arms
		unsigned char lForward, rForward;
		int16_t lEnc, rEnc;       // enc counts 
		bool usingi2c;            // avoid conflicting contacts	
		bool runPID;
		int maxleft, maxright;

		// odometry vars;
		Matrix3f rob2world;	// rotation matrix calculated given pose
		Vector3f robotstep, worldstep, odomloc; // distance changes in robot + world frames
		void calculateTransform(float theta);

		void debugLoop();
		void calculateOdom();
		void power2pwm();
		void checki2c();
		void quei2c(int size, unsigned char *q);
};
