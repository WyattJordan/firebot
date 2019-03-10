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

#define addrDrive 17  // I2C slave addresses also 0x11 and 0x10
#define addrArm   16
#define WheelLCM  13.75 // width is 29.2cm outer 24.8cm inner, L is half this value
#define WheelRCM  6.2  // diameter is 12.4, radius is 6.2cm
#define PI2	  6.28319
#define INTSIZE   4

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
		void openSerial();
		void setSerialMotors();
		void getSerialEncoders();
		void sendSerial(char send[], int size);
		void piI2C(int size, unsigned char *q);
		bool getEncoders();
		bool setMotors(int trynum);
		bool contactArms();
		void setNav(Nav* nv);
		string tmp;
		int failed_reads, failed_writes, contacts, left255, right255;

	private:
		Nav *nav_;
		PID posePID_;
		float setPose_, setSpeed_, error_;
		double kp_, ki_, kd_, min_, max_, dt_; 
		int fd_; // file descriptor for I2C port
		int Rfails_[4];
		float lDrive_, rDrive_;     // drive power levels -1:1
		
		unsigned char lPWM_, rPWM_; // drive PWMs 0:255
		unsigned char D3_, D6_, D9_, D10_, D11_; // PWMs 0:255 for arms
		unsigned char lForward_, rForward_;
		int16_t lEnc_, rEnc_;   // enc counts 
		bool usingi2c_;         // avoid conflicting contacts	
		bool i2c_; 		// is odroid connected to circuit (for testing w/o bot)
		bool debugDrive_, runPID_, eStop_;
		int maxleft_, maxright_;
		int ms_; // ms delay between odom updates, fastest thread
		int wayUpdateRate_, mapUpdateRate_, robUpdateRate_;

		// odometry vars;
		Matrix3f rob2world_;	// rotation matrix calculated given pose
		Vector3f robotstep_, worldstep_, odomWorldLoc_; // distance changes in robot + world frames
		void calculateTransform(float theta);

		void debugLoop();
		void calculateOdom();
		void power2pwm();
		void checki2c();
		void quei2c(int size, unsigned char *q);
		void quei2c_4b(int size, unsigned char *q);
};
