/* Robot.cpp
 *
 */

#include "Robot.h"
#include "Nav.h"
#include "pid.h"
#include <Eigen/Core>
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

//using namespace std::literals ;
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

Robot::Robot() : posePID(0,0,0,0,0,0){ // also calls pose constructor
	failed_reads = failed_writes = contacts = 0;
	maxleft = maxright = 0;
	left255 = right255 = 0;
	usingi2c = false;
	runPID = false;
	lDrive = rDrive = 0;
	lForward = 'a';
	rForward = 'c';
	lPWM = rPWM = 200;
	D3 = 0;
	D6 = 40;
	D9 = 127;
	D10 = 180;
	D11 = 255;
	power2pwm();

	rob2world << 1, 0, 0,   0, 1, 0,   0, 0, 1; // as if theta = 0
} 

void Robot::recon(firebot::ReconConfig &config, uint32_t level){ 
	lDrive = config.left;
	rDrive = config.right;
	power2pwm();
	setPose = config.setpose;
	runPID  = config.runpid;
	// do these need to be member variables? probs not
	kp_ = config.kp;
	ki_ = config.ki;
	kd_ = config.kd;
	max_ = config.max;
	min_ = config.min;

	posePID = PID(10.0, kp_, ki_, kd_, max_, min_);	
	std::cout<<"RECONFIGURED!\n\n";
		
}

// Increment locX, locY, locP with the new encoder vals
void Robot::calculateOdom(){
	robotstep << WheelRCM / 2.0 * (lEnc + rEnc),              /* robot move in x */
			  	 0,                                           /* robot cant move in its own y */
				 WheelRCM / (2.0 * WheelLCM) * (lEnc - rEnc); /* robot rotation */
	calculateTransform(odomloc(3) + robotstep(3)/2.0);	      // find transform using half the step	
	worldstep = rob2world*robotstep; 
	odomloc += worldstep;
}

// Given the robot's pose relative to the world calculate rob2world.
void Robot::calculateTransform(float theta){
	rob2world.topLeftCorner(2,2) << cos(theta), -sin(theta),
									sin(theta),  cos(theta);
}
void Robot::debugLoop(){



}

void Robot::driveLoop(){
	std::cout<<"setting first motors....\n";
	setMotors();
	usleep(10*1000);
	std::cout<<"entering loop \n";
	while(1){
		// get encoders, do PID math, set motors, delay dt
	boost::chrono::system_clock::time_point time_limit =
	   	boost::chrono::system_clock::now() + boost::chrono::milliseconds(500);

		std::cout<<"running\n";
		getEncoders();
		calculateOdom();
		int measured = 9;
		setPose = 10;
		if(runPID){
			float adj = posePID.calculate(setPose, measured);
			lDrive += adj;
			rDrive -= adj;	
		}
		power2pwm();
		setMotors(); std::cout<<"ran\n\n";
		//usleep(100*1000); // ms * 1000 dummy wait
		boost::this_thread::sleep_until(time_limit);
	}
}

// scales -1:1 to 0:255
void Robot::power2pwm(){
	std::cout<<"making pwms with lDrive = "<<lDrive<<" rDrive = "<<rDrive<<"\n";
	lPWM = lDrive > 0 ? lDrive*255.0 : -1*lDrive*255.0;
	rPWM = rDrive > 0 ? rDrive*255.0 : -1*rDrive*255.0;
	lForward = lDrive > 0 ? 'a' : 'b'; // directions set by char for each motor
	rForward = rDrive > 0 ? 'c' : 'd';
	std::cout<<"made pwms lPWM= "<<(int)lPWM<<" rPWm= "<<(int)rPWM<<"\n";
	std::cout<<"lforward: "<<lForward<<"  rforward: "<<rForward<<"\n";
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
	std::cout<<"fd = "<<fd<<"\n";

}

// waits for i2c to be available
void Robot::checki2c(){
	while(usingi2c){ /* put a delay here?*/ }
	usingi2c = true;
}

bool Robot::contactArms(){
	checki2c();
	if (ioctl(fd, I2C_SLAVE, addrDrive) < 0) {     
	   // Set the port options and set the address of the device we wish to speak to
	   ROS_ERROR("Unable to open port, someone else using it?");
	   return false;
	   }

	// sets all PWM values for pins D3, D6, D9, D10, and D11
	unsigned char que[6] = {'a', D3, D6, D9, D10, D11 };
	quei2c(6, que);

	usingi2c = false;
	return true;
}

// sends a byte que to the already selected I2C device 
// bytes returned from device replace sent messages in the que
void Robot::quei2c(int size, unsigned char *q){
	for(int i=0; i<size; i++){
		unsigned char send[1] = {q[i]};
		//std::cout<<" sending que with fd = " << fd<<"\n";
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
	std::cout<<"running set motors\n";
	std::cout<<"lPWM = "<<(int) lPWM<<" rPWM = "<<(int) rPWM<<"\n";
	checki2c();
	if (ioctl(fd, I2C_SLAVE, addrDrive) < 0) {     
	   // Set the port options and set the address of the device we wish to speak to
	   ROS_ERROR("Unable to open port, someone else using it?");
	   return false;
	   }
	unsigned char que[4] = {lForward, lPWM, rForward, rPWM};
	quei2c(4,que);
	usingi2c = false;
	if(que[1] == lPWM && que[3] == rPWM) return true;
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
	usingi2c = false;

	lEnc = (que[0] << 8) | que[1]; // left is first, high byte is first
	rEnc = (que[2] << 8) | que[3];
	
	if(lEnc == 255 || rEnc == 255) return false; // very rarely it fails

	// debug code
	if(abs(lEnc)>maxleft) maxleft = lEnc;
	if(abs(rEnc)>maxright) maxright = rEnc;
	if(lEnc == 255) left255++;
	if(rEnc == 255) right255++;
	std::cout<<"lEnc = "<<lEnc<<"  maxleft = " <<maxleft <<" 255 count = "<<left255<<"\n";
	std::cout<<"rEnc = "<<rEnc<<"  maxright = "<<maxright<<" 255 count = "<<right255<<"\n";
	std::cout<<"Failed reads: "<<failed_reads<<" and Failed writes: "<<
		failed_writes<<" out of "<<contacts<<"\n";// */
	return true;
}

