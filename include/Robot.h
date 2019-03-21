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
#include <chrono>

#define addrDrive 17  // I2C slave addresses also 0x11 and 0x10
#define addrArm   16
#define WheelDist 12.95 // width is 28.8cm outer, 24cm inner, L is half this center value
#define WheelRad  6.2  // diameter is 12.4, radius is 6.2cm
#define PI2	  6.28319
#define PI	  3.14159 
#define stc std::chrono
#define clk std::chrono::steady_clock

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
		void mainLogic();
		void driveLoop();
		void recon(firebot::ReconConfig &config, uint32_t level);
		void lidarCallback(); // runs everytime a new lidar scan comes in
		void setSerialArms();
		void openSerial();
		void setSerialMotors();
		void getSerialEncoders();
		void sendSerial(char send[], int size);
		void setNav(Nav* nv);

	private:
		Nav *nav_;
		PID posePID_;
		float fudge_, setPose_, error_;
		double kp_, ki_, kd_, min_, max_, dt_; 
		int mapCount_, wayCount_, robCount_, debugCount_; // periodic triggers
		int fd_; // file descriptor for I2C port
		float lDrive_, rDrive_, speed_;
		float  rampSpeed_, rampInc_, rampTime_; // drive power levels -1:1
		bool useSpeed_, ramp_, firstRamp_, speedChange_;
		
		unsigned char lPWM_, rPWM_; // drive PWMs 0:255
		unsigned char D3_, D6_, D9_, D10_, D11_; // PWMs 0:255 for arms
		unsigned char lForward_, rForward_;
		int16_t lEnc_, rEnc_;   // enc counts 
		bool debugDrive_, runPID_, eStop_;
		int ms_; // ms delay between odom updates, fastest thread
		int wayUpdateRate_, mapUpdateRate_, robUpdateRate_;

		// odometry vars;
		Matrix3f rob2world_;	// rotation matrix calculated given pose
		Vector3f robotstep_, worldstep_, odomWorldLoc_; // distance changes in robot + world frames
		void calculateTransform(float theta);

		void outputTime(clk::time_point t1, clk::time_point t2);
		void calculateOdom();
		void setRamp(float s, float t);
		void rampUpSpeed();
		float incrementRamp();
		void speed2power(float adj);
		void power2pwm();
		void periodicOutput();
		float toRad(float deg);
};
