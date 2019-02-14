/* Robot.cpp
 *
 */

#include "Robot.h"
#include "Nav.h"
#include "pid.h"
#include <Eigen/Core>
//#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <firebot/ReconConfig.h>

#include <string>
#include <iostream>
#include <fstream>
#include <wiringPi.h>

#include <boost/thread/thread.hpp>
#include <chrono>
using std::string;

// includes for i2c
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

using std::cout;
using namespace Eigen;
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

Robot::Robot() : posePID_(0,0,0,0,0,0){ // also calls pose constructor
	failed_reads = failed_writes = contacts = 0;
	maxleft_ = maxright_ = 0;
	left255 = right255 = 0;
	usingi2c_ = false;
	debugDrive_ = false;
	i2c_ = false;
	runPID_ = false;
	lDrive_ = rDrive_ = 0;
	lForward_ = 'a';
	rForward_ = 'c';
	lPWM_ = rPWM_ = 200;
	lEnc_ = rEnc_ = 0;
	ms_ = 20;

	D3_ = 0;
	D6_ = 40;
	D9_ = 127;
	D10_ = 180;
	D11_ = 255;
	power2pwm();
	srand(time(NULL));
	
	// initialize vectors, if not it won't compile!
	rob2world_ << 1, 0, 0,   0, 1, 0,   0, 0, 1; // as if theta = 0
	robotstep_ << 0,0,0;
	worldstep_ << 0,0,0;
	odomloc_   << 0,0,0;
	
} 

void Robot::setNav(Nav* nv){
	nav_ = nv;
}

void Robot::recon(firebot::ReconConfig &config, uint32_t level){ 
	mapUpdateRate_ = config.maprate;
	wayUpdateRate_ = config.wayrate;
	lDrive_ = config.left;
	rDrive_ = config.right;
	i2c_ = config.i2c;
	debugDrive_ = config.debugdrive;
	power2pwm();
	runPID_  = config.runpid;
	setPose_ = config.setpose;
	// do these need to be member variables? probs not
	kp_ = config.kp;
	ki_ = config.ki;
	kd_ = config.kd;
	max_ = config.max;
	min_ = config.min;

	posePID_ = PID(ms_, kp_, ki_, kd_, max_, min_);	
	cout<<"RECONFIGURED!\n\n";
		
}

// Increment locX, locY, locP with the new encoder vals
void Robot::calculateOdom(){
	float x = WheelRCM / 2.0 * (lEnc_ + rEnc_);
	float y = 0;
	float p = WheelRCM / (2.0 * WheelLCM) * (lEnc_ - rEnc_);
	robotstep_ <<  x, y, p;
	float theta = odomloc_(2) + robotstep_(2)/2.0; // this isn't matlab!
	calculateTransform(theta);	      // find transform using half the step	
	worldstep_ = rob2world_*robotstep_; 
	odomloc_ += worldstep_;
}

// Given the robot's pose relative to the world calculate rob2world_.
void Robot::calculateTransform(float theta){
	rob2world_.topLeftCorner(2,2) << cos(theta), -sin(theta),
			 sin(theta),  cos(theta);

}

void Robot::debugLoop(){


}

void Robot::driveLoop(){
	cout<<"setting first motors....\n";
	setMotors();
	usleep(5*ms_*1000); // sleep 5 loops before starting
	cout<<"entering loop \n";
	int mapCount, wayCount, robCount;
	mapCount = wayCount = robCount = 0;
	i2c_ = false;

	while(1){
		// get encoders, do PID math, set motors, delay dt
	boost::chrono::system_clock::time_point time_limit =
	   	boost::chrono::system_clock::now() + boost::chrono::milliseconds(ms_);

		if(debugDrive_) cout<<"running, i2c = "<<i2c_<<"\n";
		if(i2c_) {getEncoders();}
		else {
			lEnc_ = 100 * lDrive_ ;//+ rand()%10;
			rEnc_ = 100 * rDrive_ ;//+ rand()%10;		
		}
		if(debugDrive_) cout<<"got/made enc vals, doing odom...\n";
		calculateOdom();
		int measured = 9;
		setPose_ = 10;
		if(debugDrive_) cout<<"running pid = "<<runPID_ <<"\n";
		if(runPID_){
			float adj = posePID_.calculate(setPose_, measured);
			lDrive_ += adj;
			rDrive_ -= adj;	
		}
		power2pwm();
		if(i2c_) setMotors();
	       	if(debugDrive_) cout<<"ran\n\n";

		// set flags for publishing markers and robot
		mapCount++; wayCount++; robCount++;
		if(mapUpdateRate_ > 0 && mapCount >= 1000 / (ms_ * mapUpdateRate_)) {
			nav_->pubMap_  = true;
			mapCount = 0;
		}
		if(wayUpdateRate_ > 0 && wayCount >= 1000 / (ms_ * wayUpdateRate_)){
			nav_->pubWays_ = true;
			wayCount = 0;
		}
		if(robUpdateRate_ > 0 && robCount >= 1000 / (ms_ * robUpdateRate_)){
			nav_->setOdomLoc(odomloc_); // give the Nav class the odom loc
			nav_->pubRob_ = true;
			robCount = 0;
		}


		auto start = std::chrono::steady_clock::now(); // measure length of time remaining
		boost::this_thread::sleep_until(time_limit);
		auto end = std::chrono::steady_clock::now();
		if(debugDrive_)
		       	cout<<"driveLoop running in "<< ms_ <<" ms with "<<
		std::chrono::duration_cast <std::chrono::milliseconds>(end-start).count()<<"ms and "<<
		std::chrono::duration_cast <std::chrono::microseconds>(end-start).count()<<
		"us (hopefully) leftover\n";
	}
}

// scales -1:1 to 0:255
void Robot::power2pwm(){
	//cout<<"making pwms with lDrive_ = "<<lDrive_<<" rDrive_ = "<<rDrive_<<"\n";
	lPWM_ = lDrive_ > 0 ? lDrive_*255.0 : -1*lDrive_*255.0;
	rPWM_ = rDrive_ > 0 ? rDrive_*255.0 : -1*rDrive_*255.0;
	lForward_ = lDrive_ > 0 ? 'a' : 'b'; // directions set by char for each motor
	rForward_ = rDrive_ > 0 ? 'c' : 'd';
	//cout<<"made pwms lPWM_= "<<(int)lPWM_<<" rPWm= "<<(int)rPWM_<<"\n";
	//cout<<"lforward: "<<lForward_<<"  rforward: "<<rForward_<<"\n";
}

// tries to open the I2C port and set fd, repeates 10 times if failing
void Robot::openI2C(){
   const char *fileName = "/dev/i2c-1";         // Name of the port we will be using
   int count = 0;
    while ((fd = open(fileName, O_RDWR)) < 0 && count < 10) {   // Open port for reading and writing
	    sleep(1);
      printf("Failed to open i2c port, did you set sudo??\n trying again...");
	  if (count == 10) {
		ROS_ERROR(" Could not open I2C port, aborting mission\n");
	    //exit(1);
	  }
   }
	ROS_INFO("Successfully opened I2C port");
	cout<<"fd = "<<fd<<"\n";

}

// waits for i2c to be available
void Robot::checki2c(){
	while(usingi2c_){ /* put a delay here?*/ }
	usingi2c_ = true;
}

bool Robot::contactArms(){
	checki2c();
	if (ioctl(fd, I2C_SLAVE, addrDrive) < 0) {     
	   // Set the port options and set the address of the device we wish to speak to
	   ROS_ERROR("Unable to open port, someone else using it?");
	   return false;
	   }

	// sets all PWM values for pins D3, D6, D9, D10, and D11
	unsigned char que[6] = {'a', D3_, D6_, D9_, D10_, D11_ };
	quei2c(6, que);

	usingi2c_ = false;
	return true;
}

// sends a byte que to the already selected I2C device 
// bytes returned from device replace sent messages in the que
void Robot::quei2c(int size, unsigned char *q){
	for(int i=0; i<size; i++){
		unsigned char send[1] = {q[i]};
		//cout<<" sending que with fd = " << fd<<"\n";
		if ((write(fd, &send, 1)) != 1) {         // send a byte   
		      printf("error writing to i2c slave in Robot::quei2c\n");
		      failed_writes++;
		      //exit(1);
		   }
		contacts++;
		 if (read(fd, send, 1) != 1) {            // Read a byte 
		      printf("Unable to read from slave in Robot::quei2c\n");
		      failed_reads++;
		   }
		contacts++;

	 q[i] = send[0];
	}

}

// send PWM signals to Arduino
bool Robot::setMotors(){
	cout<<"running set motors\n";
	cout<<"lPWM_ = "<<(int) lPWM_<<" rPWM_ = "<<(int) rPWM_<<"\n";
	checki2c();
	if (ioctl(fd, I2C_SLAVE, addrDrive) < 0) {     
	   // Set the port options and set the address of the device we wish to speak to
	   ROS_ERROR("Unable to open port, someone else using it?");
	   return false;
	   }
	unsigned char que[4] = {lForward_, lPWM_, rForward_, rPWM_};
	quei2c(4,que);
	usingi2c_ = false;
	if(que[1] == lPWM_ && que[3] == rPWM_) return true;
	ROS_ERROR("Robot::setMotors miscommunication");
	return false;
}

bool Robot::getEncoders(){
	checki2c();
	if (ioctl(fd, I2C_SLAVE, addrDrive) < 0) {     
	   // Set the port options and set the address of the device we wish to speak to
	   ROS_ERROR("Unable to open port, someone else using it?");
	   return false;
	   }
	unsigned char que[4] = {'1','2','3','4'};
	quei2c(4,que);
	usingi2c_ = false;

	lEnc_ = (que[0] << 8) | que[1]; // left is first, high byte is first
	rEnc_ = (que[2] << 8) | que[3];
	
	if(lEnc_ == 255 || rEnc_ == 255) return false; // very rarely it fails

	// debug code
	/*
	if(abs(lEnc_)>maxleft_) maxleft_ = lEnc_;
	if(abs(rEnc_)>maxright_) maxright_ = rEnc_;
	if(lEnc_ == 255) left255++; // sometimes failed reads return 255
	if(rEnc_ == 255) right255++;
	cout<<"lEnc_ = "<<lEnc_<<"  maxleft_ = " <<maxleft_ <<" 255 count = "<<left255<<"\n";
	cout<<"rEnc_ = "<<rEnc_<<"  maxright_ = "<<maxright_<<" 255 count = "<<right255<<"\n";
	cout<<"Failed reads: "<<failed_reads<<" and Failed writes: "<<
		failed_writes<<" out of "<<contacts<<"\n";// */
	return true;
}

