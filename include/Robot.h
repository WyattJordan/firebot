/* Robot.h
 * Manages all sensor inputs and outputs.
 * Controls drive motors and servo motors (interfaced through arduino).
 * PID loop for driving and odometry calculations.
 */
#ifndef ROBOT_H
#define ROBOT_H

#include "ros/ros.h"
#include "Nav.h"
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

class Nav; // forward declaration since both include eachother
class Lidar;

static void pabort(const char *s){
	perror(s);
	abort();
}

class Robot{

	public:
		Robot();

		// Set pointers
		void setNav(Nav* nv);
		void setLidar(Lidar* lid);
		
		// handles high level control of Robot. Commands to complete the challenge.
		void mainLogic();
		
		// This is the main thread that runs and controls the 'bot the mainLogic
		// loop simply adjusts variables that this thread is referencing.
		// Flow:
		// 	1. Get Encoder counts via arduino 
		// 	2. Determine new world location from counts or if a LIDAR update occurred
		// 	3. Check if the speed is being ramped and if so increment speed_ (rampupspeed())
		// 	4. Check if a navStack is running and if so set pose accordingly
		// 	5. Run the PID and and calculate lDrive and rDrive
		// 	6. Set the motors via arduino
		// 	7. Output debug info (not every loop) includes rviz map, ways, debugDrive_
		// 	8. Wait time remaining such that this loop took 20ms (or ms_ ms)
		void driveLoop();

		// Continually update the transformation from ROBOTFRAME to GLOBALFRAME for the lidar data
		// Transformation is based on the odomWOrldloc
		void pubTransformContinual(int rate); // rate in HZ

		// Transform an Endpoint in the lidar/robot frame to the global frame
		EndPoint transformEndPoint(EndPoint ep);

		// Set flag to integrate position in calculateOdom() and save newPos
		void updatePosition(Vector3f newPos);

		// Return currently stored location
		Vector3f getOdomWorldLoc();

		// Travel distance since last update
		Ref<Vector3f> getTravelDist();

		// Get a copy of the transform from lidar/robot to global frame
		tf::StampedTransform getTransform();

		// For testing localization updates from LIDAR without actually applying them
		void setExperimental(Vector3f pose);

		// For testing various pins and sensors
		void pinThread();

		// Given two timepoints output the duration between theme to console
		void outputTime(clk::time_point t1, clk::time_point t2);

	private:
		Nav *nav_;
		Lidar *lid_;
		PID posePID_;
		
		float fudge_, setPose_, adj_;
		double kp_, ki_, kd_, min_, max_, dt_; 		   		// PID constants 
		int mapCount_, wayCount_, robCount_, debugCount_; 	// periodic triggers
		int fd_; 											// file descriptor for I2C port
		float lDrive_, rDrive_, speed_; 					// power levels for motors/speed (-1:1)
		float  rampSpeed_, rampInc_, rampTime_; 			// drive power levels -1:1
		bool useSpeed_, ramp_, firstRamp_, speedChange_;  	// flags for rampupspeed()
		
		unsigned char lPWM_, rPWM_; 				// drive PWMs 0:255
		unsigned char D3_, D6_, D9_, D10_, D11_; 	// PWMs 0:255 for arms
		unsigned char lForward_, rForward_; 		// either 'f' or 'b' for drive motors dir on arduino
		int16_t lEnc_, rEnc_;   // enc counts 
		bool debugDrive_, runPID_, eStop_;			// output info to console, self-explanatory flags
		int ms_; 									// ms delay between odom updates, usually 20ms (50Hz)
		int wayUpdateRate_, mapUpdateRate_, robUpdateRate_; // rates for RVIZ updates
		int delay_; 								// testing delay for between arduino reads/writes
		deque<EndPoint> navStack; 					// stack of waypoints the robot will navigate between
		bool startNavStack_, firstNav_, facingFirst_, reversed_, pt2pt_; // flags for executeNavStack()
		
		// For updating the position from the Nav class
		bool updateDriving_, updateSavedPos_; 
		Vector3f travelDist_, newPos_;
		tf::TransformBroadcaster br_;
		tf::StampedTransform tfTrans_;


		// odometry vars;
		Matrix3f rob2world_;	// rotation matrix calculated given pose
		// distance changes in robot + world frames
		Vector3f robotstep_, worldstep_, odomWorldLoc_, experimental_; 

		// makes rob2world (a z-axis rotation matrix) for a given theta
		void calculateTransform(float theta);

		// Runs in driveLoop at 50Hz.
		// 1. calculate robotstep_ which is change in robot location in robot frame
		// If no update:
		// 		Calculate rotation matrix rob2world using calculateTransform()
		//		Transform the step in the robot frame to the world frame and add to the world location
		//		increment the traveldis_ with the robot step
		// If update
		//		Change x,y,t that were saved in newPos_
		//		reset rraveldist
		void calculateOdom();

		// open the tty/USB0 serial port and configure with correct settings for the arduino
		void openSerial();

		// check for estop_ and send l/rForward and l/rPWM to the motors
		void setSerialMotors();

		// read the encoder values from the arduino
		bool getSerialEncoders();

		// send the arm servo PWM values to the arduino
		void setSerialArms();

		// set the ramp speed, time, and two flags to start ramping up the speed
		void setRamp(float s, float t);

		// calculate rampInc given PID loop rate (50Hz) and increment until 
		// desired speed is reached (gets called by driveLoop)
		void rampUpSpeed();

		// turn off the PID loop, wiggle left/right num times after turning on the 
		// water pump and solenoid valve
		void sprayNpray(int num);

		// set state of the pump and solenoid valve (both wired to the same pin)
		void setSprayer(bool on);

		// tell the lidar class to search for the candle, rotate 40deg, and search again
		// if found make a new navstack to the candle waypoint returned by Lidar::findCandle
		// execute that nav stack and run sprayNpray()
		bool searchNDestroy();

		// given speed level and error set l/rPWM
		void speed2power(float adj);
		// given l/rDrive set l/rPWM
		void power2pwm();

		// output info to console at slower rate than driveLoop
		void periodicOutput();

		float prevdist_; // save last distance to next waypoint to determine if passing it

		// given stack of Endpoint IDs make the stack of endpoints
		bool buildNavStack(vector<int> ids,bool append=false);

		// add an endpoint (given ID) to the front of the stack
		bool insertToNavStack(int id);

		// Main navigation loop. Called from mainLogic runs in parallel with driveLoop
		// Drives to next waypoint, once reached stops, adjusts to face next waypoint
		// once turned continues to drive, repeats for all points in navStack_
		// Also can do rounded turns at 50% speed but odom accuracy decreases quickly
		int executeNavStack();

		// Finds global pose to an endpoint (used by executeNavStack
		// Or find pose between two different endpoints if both arguments are given
		float getPoseToPoint(EndPoint pt, EndPoint* pt2 = NULL);
		// calculate distance between current robot location and top endpoint in navStack
		float distToNextPoint();

		// wait t milliseconds
		void msleep(int t);
		// return deg value in radians
		float toRad(float deg);
};

#endif
