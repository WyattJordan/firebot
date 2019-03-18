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
#define clk boost::chrono::system_clock
#define stc std::chrono

Robot::Robot() : posePID_(0,0,0,0,0,0){ // also calls pose constructor
	failed_reads = failed_writes = contacts = 0;
	maxleft_ = maxright_ = 0;
	left255 = right255 = 0;
	debugDrive_ = runPID_ = eStop_ = false;
	lDrive_ = rDrive_ = 0;
	lForward_ = 'a';
	rForward_ = 'c';
	lPWM_ = rPWM_ = 200;
	lEnc_ = rEnc_ = 0;
	ms_ = 20;

	D3_ = 0; D6_ = 40; D9_ = 127; D10_ = 180; D11_ = 255;
	power2pwm();
	srand(time(NULL));
	ramp_ = false;
	
	// initialize vectors, if not it won't compile!
	rob2world_ << 1, 0, 0,   0, 1, 0,   0, 0, 1; // as if theta = 0
	robotstep_ << 0,0,0;
	worldstep_ << 0,0,0;
	//odomWorldLoc_   << 0,0,0; // starting pose/position
	//odomWorldLoc_   << WheelDist,13.5,0; // start at back left w/ steel block
	odomWorldLoc_   << 123,WheelDist,PI2/4; // start at center of back wall
	// lower right corner of maze facing to the right

	wiringPiSetup();
	openSerial();
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
	useSpeed_ = config.usespeed;
	speed_ = config.speed;
	fudge_ = config.fudge;
	ramp_ = config.ramp;
	if(ramp_) {firstRamp_ = true;}
	rampSpeed_ = config.rampspeed;
	rampTime_  = config.ramptime;

	if(useSpeed_){
		lDrive_ = speed_;
		rDrive_ = speed_ * fudge_;
	}

	power2pwm();

	runPID_  = config.runpid;
	setPose_ = config.setpose;
	eStop_ 	 = config.estop;
	// do these need to be member variables? probs not
	kp_ = config.kp;
	ki_ = config.ki;
	kd_ = config.kd;
	max_ = config.max;
	min_ = config.min;

	mapCount_ = wayCount_ = robCount_ = debugCount_ = 0;

	posePID_ = PID(ms_, kp_, ki_, kd_, max_, min_);	
	cout<<"RECONFIGURED!\n\n";
}


// Increment locX, locY, locP with the new encoder vals
void Robot::calculateOdom(){
	bool debug = false;
	float lRad = (PI2 * (float) lEnc_ ) / (663.0 * 2.0); // 663.0 enc counts / rotation
	float rRad = (PI2 * (float) rEnc_ ) / (663.0 * 2.0); 
	if(debug) cout<<"lRad = "<<lRad<<" rRad = "<<rRad<<"\n";

	float x = WheelRad / 2.0 * (lRad + rRad);
	float y = 0;
	float p = WheelRad / (2.0 * WheelDist) * (rRad - lRad);
	robotstep_ <<  x, y, p;
	if(debug) cout<<"robot step:\n"<<robotstep_<<"\n";
	if(debug) cout<<"odomWorldLoc_ = \n"<<odomWorldLoc_<<"\n";	
	float theta = odomWorldLoc_(2) + robotstep_(2)/2.0; // radians!

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
	int16_t mcode[1] = {-1}; // cannot be from 0-255 (use negatives)
	unsigned char mdata[4] = {lForward_, lPWM_, rForward_, rPWM_};	
	write(fd_, mcode, 2); // Identify what info is coming next 
	write(fd_, mdata, 4);
}

void Robot::setSerialArms(){
	int16_t mcode[1] = {-3}; // code is -3, then sends 5 bytes for PWMs
	write(fd_, mcode, 2); 
	//usleep(500);
	unsigned char mdata[5] = {D3_, D6_, D9_, D10_, D11_ };
	write(fd_, mdata, 5);
}

void Robot::getSerialEncoders(){
	int16_t mcode2[1] = {-2}; // cannot be from 0-255 (use negatives)
	unsigned char encs[4];
	write(fd_, mcode2, 2); // Identify what info is coming next 
	usleep(500);
	read(fd_, encs, 4); // read 2 integers from the arduino

	lEnc_ = (encs[0] << 8) | encs[1]; 
	rEnc_ = (encs[2] << 8) | encs[3]; 
	//if(lEnc_ != 0) cout<<"left enc is "<<lEnc_<<"\n";
	//if(rEnc_ != 0) cout<<"right enc is "<<rEnc_<<"\n\n";
}


void Robot::rampUpSpeed(){
	if(firstRamp_){
		cout<<"first ramp%%%%%%%%%%%%%%%%%%%\n";
		rampInc_ = (rampSpeed_ - speed_) / (rampTime_ / (ms_ / 1000.0));
		firstRamp_ = false;
	}
	if(ramp_){
		cout<<"Ramping... rampInc_ = "<<rampInc_<<" speed = "<<speed_<<"\n";
		speed_ = speed_ + rampInc_;
		speed2power(0);
		cout<<"made speed drive levels, making pwms\n";
		power2pwm();
		cout<<"done PWM\n";
		// if ramp has overshot
		if((rampSpeed_ < 0 && speed_ < rampSpeed_) ||
			(rampSpeed_ > 0 && speed_ > rampSpeed_)){
			cout<<"exiting ramp...\n";
			speed_ = rampSpeed_;
			ramp_ = false;
		}	
	}

}
void Robot::speed2power(float adj){
	lDrive_ = speed_ - adj;
	rDrive_ = speed_*fudge_ + adj;	
}

void Robot::driveLoop(){
	cout<<"talking to arduino... \n";
	getSerialEncoders();
	cout<<"got encoders\n";

	nav_->pubWays_ = true;
	nav_->pubMap_ = true;
	nav_->pubRob_ = true;
	usleep(5*ms_); // sleep 5 loops before starting

	cout<<"starting driveLoop\n";
	// get encoders, do PID math, set motors, delay leftover time 
	while(1){
		clk::time_point time_limit = clk::now() + boost::chrono::milliseconds(ms_);

		getSerialEncoders(); 
		calculateOdom();
		//rampUpSpeed();
		
		if(runPID_){
			if(0||debugDrive_) cout<<"in pid,odomWorldLoc_ = \n"<<odomWorldLoc_<<"\n";
			if(0||debugDrive_) cout<<"theta = "<<180/PI2*odomWorldLoc_(2)<<"\n";

			// give the PID the desired and current rotations
			// note: 0 is robot front, +0 turns left, -0 turns right
			// loop around occurs at back of robot
			float adj = posePID_.calculate(setPose_ * PI2/180.0, odomWorldLoc_(2));
			speed2power(adj);
			cout<<"adj = "<<adj<<"\n";
		}
		power2pwm();
		setSerialMotors();
		if(debugDrive_) {
			cout<<"lDrive_= "<<lDrive_<<" ("<<lPWM_<<
				") rDrive_ = "<<rDrive_<<" ("<<rPWM_<<")\n";
		}

		periodicOutput(); // check what needs to be output

		auto start = stc::steady_clock::now(); // measure length of time remaining
		boost::this_thread::sleep_until(time_limit);
		auto end = stc::steady_clock::now();

		if(debugDrive_)
		       	cout<<"driveLoop takes "<< ms_ <<" ms with "<<
		stc::duration_cast <stc::milliseconds>(end-start).count()<<"ms and "<<
		stc::duration_cast <stc::microseconds>(end-start).count()<<
		"us (hopefully) leftover\n";
	}
}

void Robot::periodicOutput(){
	mapCount_++; wayCount_++; robCount_++; debugCount_++;
	if(debugDrive_){
		debugDrive_ = false;
	}
	if(0 && debugCount_ > 200){ // 100 = every 4 seconds (ms_ = 20);
		debugDrive_ = true;
		debugCount_ = 0;
	}
	
	if(mapUpdateRate_ > 0 && mapCount_ >= 1000 / (ms_ * mapUpdateRate_)) {
		nav_->pubMap_  = true;
		mapCount_ = 0;
	}
	if(wayUpdateRate_ > 0 && wayCount_ >= 1000 / (ms_ * wayUpdateRate_)){
		nav_->pubWays_ = true;
		wayCount_ = 0;
	}
	if(robUpdateRate_ > 0 && robCount_ >= 1000 / (ms_ * robUpdateRate_)){
		nav_->setOdomLoc(odomWorldLoc_); // give the Nav class the odom loc
		nav_->pubRob_ = true;
		robCount_ = 0;
	}

}

void Robot::openSerial(){
	cout<<"opening USB connection \n";
	//fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY); // read+write | not controlling term | ? 
	fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY); // read+write | not controlling term | ? 

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

	//options.c_oflag = 0;
	options.c_lflag = 0; // allow raw input

	// THESE TWO LINES ARE CRITICAL FOR TIMING WHEN READING!!
	// see: https://stackoverflow.com/questions/20154157/termios-vmin-vtime-and-blocking-non-blocking-read-operations 
	options.c_cc[VTIME] = 1; // wait 0.1s between bytes before timing out on reads
	options.c_cc[VMIN] = 4; // must read at least 4 bytes when reading (only reading encs)


	tcsetattr(fd_, TCSANOW, &options);

	fcntl(fd_, F_SETFL, 0); // set the file status flag
	sleep(3);
	cout<<"done Robot::openSerial\n";
	
}


// scales -1:1 to 0:255
void Robot::power2pwm(){
	if(debugDrive_) cout<<"making pwms with lDrive_ = "<<lDrive_<<" rDrive_ = "<<rDrive_<<"\n";
	lPWM_ = lDrive_ >= 0 ? lDrive_*255.0 : -1*lDrive_*255.0;
	rPWM_ = rDrive_ >= 0 ? rDrive_*255.0 : -1*rDrive_*255.0;
	lForward_ = lDrive_ >= 0 ? 'f' : 'b'; // directions set by char for each motor
	rForward_ = rDrive_ >= 0 ? 'f' : 'b';
	if(debugDrive_) cout<<"made pwms lPWM_= "<<(int)lPWM_<<" rPWm= "<<(int)rPWM_<<"\n";
	if(debugDrive_) cout<<"lforward: "<<lForward_<<"  rforward: "<<rForward_<<"\n";
}

