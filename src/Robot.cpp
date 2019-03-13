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
#include <wiringPiI2C.h>

#include <boost/thread/thread.hpp>
#include <chrono>
using std::string;

// includes for i2c
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>

// for serial
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

using std::cout;
using namespace Eigen;
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

Robot::Robot() : posePID_(0,0,0,0,0,0){ // also calls pose constructor
	failed_reads = failed_writes = contacts = 0;
	Rfails_[0] = 0; 
	Rfails_[1] = 0; 
	Rfails_[2] = 0; 
	Rfails_[3] = 0; 
	maxleft_ = maxright_ = 0;
	left255 = right255 = 0;
	debugDrive_ = runPID_ = eStop_ = false;
	lDrive_ = rDrive_ = 0;
	lForward_ = 'a';
	rForward_ = 'c';
	lPWM_ = rPWM_ = 200;
	lEnc_ = rEnc_ = 0;
	ms_ = 10;

	D3_ = 0; D6_ = 40; D9_ = 127; D10_ = 180; D11_ = 255;
	power2pwm();
	srand(time(NULL));
	
	// initialize vectors, if not it won't compile!
	rob2world_ << 1, 0, 0,   0, 1, 0,   0, 0, 1; // as if theta = 0
	robotstep_ << 0,0,0;
	worldstep_ << 0,0,0;
	odomWorldLoc_   << 0,0,0; // starting pose/position

	wiringPiSetup();
	fd_ = wiringPiI2CSetup(0x11);
} 

void Robot::setNav(Nav* nv){
	nav_ = nv;
}

void Robot::recon(firebot::ReconConfig &config, uint32_t level){ 
	mapUpdateRate_ = config.maprate;
	wayUpdateRate_ = config.wayrate;
	robUpdateRate_ = config.robrate;
	ms_ = config.ms;
	lDrive_ = config.left;
	rDrive_ = config.right;
	//i2c_ = config.i2c;
	debugDrive_ = config.debugdrive;
	power2pwm();
	runPID_  = config.runpid;
	setPose_ = config.setpose;
	setSpeed_= config.setspeed;
	eStop_ 	 = config.estop;
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
	bool debug = false;
	float lRad = (PI2 * lEnc_ ) / 663.0; // 663.0 enc counts / rotation
	float rRad = (PI2 * rEnc_ ) / 663.0; 
	if(debug) cout<<"lRad = "<<lRad<<" rRad = "<<rRad<<"\n";

	float x = WheelRCM / 2.0 * (lRad + rRad);
	float y = 0;
	float p = WheelRCM / (2.0 * WheelLCM) * (rRad - lRad);
	robotstep_ <<  x, y, p;
	if(debug) cout<<"robot step:\n"<<robotstep_<<"\n";
	if(debug) cout<<"odomWorldLoc_ = \n"<<odomWorldLoc_<<"\n";	
	float theta = odomWorldLoc_(2) + robotstep_(2)/2.0; // this isn't matlab!

	if(debug) cout<<"using theta = "<<theta<<"\n";
	calculateTransform(theta);	      // find transform using half the step	
	if(debug) cout<<"rob2world_ =\n"<<rob2world_<<"\n";
	worldstep_ = rob2world_*robotstep_; 
	odomWorldLoc_ += worldstep_;

	if(debug) cout<<"odomWorldLoc_ = \n"<<odomWorldLoc_<<"\n";	
}

// Given the robot's pose relative to the world calculate rob2world_.
void Robot::calculateTransform(float theta){
	rob2world_.topLeftCorner(2,2) << cos(theta), -sin(theta),
					 sin(theta),  cos(theta);
}

void Robot::setSerialMotors(){
	if(eStop_){
		lPWM_ = 0;
		rPWM_ = 0;
	}
	//cout<<"trying to send code in Robot::setSerialMotors\n";
	int16_t mcode[1] = {-1}; // cannot be from 0-255 (use negatives)
	write(fd_, mcode, 2); // Identify what info is coming next 
	//cout<<"wrote code in setSerialMotors\n"; usleep(500); // time to send a byte at 115200 b/s is ~70us // lForward_ = 'f'; rForward_ = 'b'; lPWM_ = 221; rPWM_ = 135; unsigned char mdata[4] = {lForward_, lPWM_, rForward_, rPWM_};	
	write(fd_, mdata, 4);
	//cout<<"done setSerialMotors\n";
}

void Robot::setSerialArms(){
	int16_t mcode[1] = {-3}; // code is -3, then sends 5 bytes for PWMs
	write(fd_, mcode, 2); 
	usleep(200);
	unsigned char mdata[5] = {D3_, D6_, D9_, D10_, D11_ };
	write(fd_, mdata, 5);
}

void Robot::getSerialEncoders(){
	//cout<<"trying to send code in Robot::getSerialEncoders\n";
	int16_t mcode2[1] = {-2}; // cannot be from 0-255 (use negatives)
	write(fd_, mcode2, 2); // Identify what info is coming next 
	//cout<<"trying to send data in Robot::getSerialEncoders\n";
	usleep(500);
	unsigned char encs[4];
	read(fd_, encs, 4); // read 2 integers from the arduino

	lEnc_ = (encs[0] << 8) | encs[1]; 
	rEnc_ = (encs[2] << 8) | encs[3]; 
	if(lEnc_ != 0) cout<<"left enc is "<<lEnc_<<"\n";
	if(rEnc_ != 0) cout<<"right enc is "<<rEnc_<<"\n\n";
}

void Robot::driveLoop(){
	cout<<"talking to arduino... \n";
	getSerialEncoders();
	cout<<"got encoders\n";
	usleep(5*ms_); // sleep 5 loops before starting
	usleep(3000*1000); // sleep 3 seconds before starting

	int mapCount, wayCount, robCount, debugCount;
	mapCount = wayCount = robCount = debugCount = 0;

	nav_->pubWays_ = true;
	nav_->pubMap_ = true;
	nav_->pubRob_ = true;

	cout<<"starting driveLoop\n";
	while(1){
	mapUpdateRate_ = wayUpdateRate_ = robUpdateRate_ = 1;
		// get encoders, do PID math, set motors, delay dt
	boost::chrono::system_clock::time_point time_limit =
	   	boost::chrono::system_clock::now() + boost::chrono::milliseconds(ms_);

		getSerialEncoders(); 

		/*else { // simulate Enc vals based on drive lvls
			lEnc_ = 100 * lDrive_ ;//+ rand()%10;
			rEnc_ = 100 * rDrive_ ;//+ rand()%10;		
		}//*/
		if(debugDrive_) cout<<"lEnc_ = "<<lEnc_<<"  rEnc_ = "<<rEnc_<<"\n";
		calculateOdom();
		if(runPID_){
			if(debugDrive_) cout<<"running pid = "<<runPID_ <<"\n";
			float adj = posePID_.calculate(setPose_, odomWorldLoc_(2));
			lDrive_ += adj;
			rDrive_ -= adj;	
		}

		power2pwm();
		setSerialMotors();

		// set flags for debugging and publishing markers and robot
		mapCount++; wayCount++; robCount++; debugCount++;
		/*if(debugCount > 100){ // 100 = every 2 seconds
			debugDrive_ = ;
		}
		else if(debugDrive_){
			debugDrive_ = false;
		}*/

		if(mapUpdateRate_ > 0 && mapCount >= 1000 / (ms_ * mapUpdateRate_)) {
			nav_->pubMap_  = true;
			mapCount = 0;
		}
		if(wayUpdateRate_ > 0 && wayCount >= 1000 / (ms_ * wayUpdateRate_)){
			nav_->pubWays_ = true;
			wayCount = 0;
		}
		if(robUpdateRate_ > 0 && robCount >= 1000 / (ms_ * robUpdateRate_)){
			nav_->setOdomLoc(odomWorldLoc_); // give the Nav class the odom loc
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

void Robot::openSerial(){
	cout<<"opening USB connection \n";
	fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY); // read+write | not controlling term | ? 

	perror("open_port: /dev/ttyUSB0 - ");
	if(fd_ == -1){
		cout<<"could not open serial port\n";
		exit(1);
	}
	

	struct termios options;
	tcgetattr(fd_, &options);
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8; 	// 8 bit chars
	options.c_cflag &= ~PARENB;	// no parity
	options.c_cflag &= ~CSTOPB;
	tcsetattr(fd_, TCSANOW, &options);

	fcntl(fd_, F_SETFL, 0); // set the file status flag
	sleep(4);
	cout<<"done Robot::openSerial\n";
	
}


// scales -1:1 to 0:255
void Robot::power2pwm(){
	//cout<<"making pwms with lDrive_ = "<<lDrive_<<" rDrive_ = "<<rDrive_<<"\n";
	lPWM_ = lDrive_ > 0 ? lDrive_*255.0 : -1*lDrive_*255.0;
	rPWM_ = rDrive_ > 0 ? rDrive_*255.0 : -1*rDrive_*255.0;
	lForward_ = lDrive_ > 0 ? 'f' : 'b'; // directions set by char for each motor
	rForward_ = rDrive_ > 0 ? 'f' : 'b';
	//cout<<"made pwms lPWM_= "<<(int)lPWM_<<" rPWm= "<<(int)rPWM_<<"\n";
	//cout<<"lforward: "<<lForward_<<"  rforward: "<<rForward_<<"\n";
}

