/* Robot.h
 * Manages all sensor inputs and outputs, tons of settings and thresholds.
 * Controls drive motors and servo motors (interfaced through arduino).
 * PID loop for driving and odometry calculations.
 */
#pragma once
#include "ros/ros.h"
#include "Nav.h"
#include "lidar.h"
#include "pid.h"
#include "Endpoint.h"
#include "definitions.h"
#include <tf/transform_broadcaster.h>

#include <thread>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <Eigen/Core>
#include <Eigen/LU> // for inverse
#include <chrono>
#include <deque>

// for serial
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

using std::cout;
using std::string;
using std::deque;
using namespace Eigen;

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define clk std::chrono::steady_clock
#define stc std::chrono

#define sw1Pin  4
#define sw2Pin 21
#define sw3Pin 23
#define blueLEDPin 7
#define redLEDPin 22
#define greenLEDPin 3
#define IR1Pin 29
#define IR2Pin 25

class Nav; // forward declaration since both include eachother
class Lidar;

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
		void lidarCallback(); // runs everytime a new lidar scan comes in
		void pubTransformContinual(int rate); // rate in HZ
		void updatePosition(Vector3f newPos);
		void setNav(Nav* nv);
		void setLidar(Lidar* lid);
		void transformEndPoint(EndPoint &ep);
		Vector3f getOdomWorldLoc();
		Ref<Vector3f> getTravelDist();
		tf::StampedTransform getTransform();
		void setExperimental(Vector3f pose);
		void outputTime(clk::time_point t1, clk::time_point t2);

	private:
		Nav *nav_;
		Lidar *lid_;
		PID posePID_;
		
		float fudge_, setPose_, adj_;
		double kp_, ki_, kd_, min_, max_, dt_; 
		int mapCount_, wayCount_, robCount_, debugCount_; // periodic triggers
		int fd_; // file descriptor for I2C port
		float lDrive_, rDrive_, speed_, runSpeed_;
		float  rampSpeed_, rampInc_, rampTime_; // drive power levels -1:1
		bool useSpeed_, ramp_, firstRamp_, speedChange_;
		
		unsigned char lPWM_, rPWM_; // drive PWMs 0:255
		unsigned char D3_, D6_, D9_, D10_, D11_; // PWMs 0:255 for arms
		unsigned char lForward_, rForward_;
		int16_t lEnc_, rEnc_;   // enc counts 
		bool debugDrive_, runPID_, eStop_;
		int ms_; // ms delay between odom updates, fastest thread
		int wayUpdateRate_, mapUpdateRate_, robUpdateRate_;
		int delay_;
		deque<EndPoint> navStack;
		bool startNavStack_, firstNav_, facingFirst_, reversed_, pt2pt_;
		
		// For updating the position from the Nav class
		bool updateDriving_, updateSavedPos_; 
		Vector3f travelDist_, newPos_;
		tf::TransformBroadcaster br_;
		tf::StampedTransform tfTrans_;


		// odometry vars;
		Matrix3f rob2world_;	// rotation matrix calculated given pose
		Vector3f robotstep_, worldstep_, odomWorldLoc_, experimental_; // distance changes in robot + world frames
		void calculateTransform(float theta);
		void calculateOdom();

		void pinThread();
		void openSerial();
		void setSerialMotors();
		bool getSerialEncoders();
		void setSerialArms();
		void setRamp(float s, float t);
		void rampUpSpeed();
		void speed2power(float adj);
		void power2pwm();

		void periodicOutput();

		float prevdist_;
		bool buildNavStack(vector<int> ids,bool append=false);
		void executeNavStack();
		float getPoseToPoint(EndPoint pt, EndPoint* pt2 = NULL);
		float distToNextPoint();

		void testDistToStop();
		void moveInSquare();
		void msleep(int t);
		float toRad(float deg);
};
