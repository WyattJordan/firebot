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
	usingi2c_ = false;
	debugDrive_ = i2c_ = runPID_ = eStop_ = false;
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
	odomWorldLoc_   << 0,0,0;

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
	i2c_ = config.i2c;
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

void Robot::piI2C(int size, unsigned char *q){
	for(int i=size; i<size; i++){
		wiringPiI2CWrite(fd_,q[i]);
		q[i] = wiringPiI2CRead(fd_);
	}
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
	if(debug) cout<<"done this step!\n";
}

// Given the robot's pose relative to the world calculate rob2world_.
void Robot::calculateTransform(float theta){
	rob2world_.topLeftCorner(2,2) << cos(theta), -sin(theta),
					 sin(theta),  cos(theta);
}

void Robot::debugLoop(){
}

void Robot::setSerialMotors(){
	if(eStop_){
		lPWM_ = 0;
		rPWM_ = 0;
	}
	int16_t mcode[1] = {-1}; // cannot be from 0-255 (use negatives)
	write(fd_, mcode, 2); // Identify what info is coming next 
	//unsigned char mdata[4] = {lForward_, lPWM_, rForward_, rPWM_};	
	//write(fd_, mdata, 4);
}

void Robot::getSerialEncoders(){

}

void Robot::driveLoop(){
	cout<<"talking to arduino, reset rviz now\n";
	setMotors(1); usleep(500); setMotors(2);
	cout<<"set motors\n";
	getEncoders();
	cout<<"got encoders\n";
	usleep(5*ms_*1000); // sleep 5 loops before starting
	//usleep(5000*1000); // sleep 5 seconds before starting
	cout<<"starting driveLoop\n";
	int mapCount, wayCount, robCount, debugCount;
	mapCount = wayCount = robCount = debugCount = 0;
	//i2c_ = false; // allows for actual use vs simulation

	nav_->pubWays_ = true;
	nav_->pubMap_ = true;
	nav_->pubRob_ = true;

	vector<int> fail_hist;
	while(1){
		// get encoders, do PID math, set motors, delay dt
	boost::chrono::system_clock::time_point time_limit =
	   	boost::chrono::system_clock::now() + boost::chrono::milliseconds(ms_);

		if(0 && i2c_) {getEncoders();} //////////////////////////////////////////////
		else { // simulate Enc vals based on drive lvls
			lEnc_ = 100 * lDrive_ ;//+ rand()%10;
			rEnc_ = 100 * rDrive_ ;//+ rand()%10;		
		}
		if(debugDrive_) cout<<"lEnc_ = "<<lEnc_<<"  rEnc_ = "<<rEnc_<<"\n";
		calculateOdom();
		if(runPID_){
			if(debugDrive_) cout<<"running pid = "<<runPID_ <<"\n";
			float adj = posePID_.calculate(setPose_, odomWorldLoc_(2));
			lDrive_ += adj;
			rDrive_ -= adj;	
		}

		power2pwm();
		if(i2c_) {
			setMotors(1); 
			usleep(300);
			bool t = true;
			t = setMotors(2); 
			if(!t) fail_hist.push_back(-1);
			else fail_hist.clear();
			if(fail_hist.size() >= 10){
				cout<<"too many consecutive failures\n";
				unsigned char fail[1] = {'s'};
				quei2c(1,fail);
				exit(1);
			}
		}

	       	if(debugDrive_) cout<<"ran setMotors\n\n";

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

void Robot::sendSerial(char send[], int size){

}

void Robot::openSerial(){
	cout<<"opening USB connection \n";
	fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY); // read+write | not controlling term | ? 
	if(fd_ == -1){
		cout<<"could not open serial port\n";
		perror("open_port: Unable to open /dev/ttyUSB0 - ");
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
	cout<<"connected to USB!\n";
}

void Robot::quei2c_4b(int size, unsigned char *q){
	//cout<<" sending que with fd = " << fd<<"\n";
	if ((write(fd_, &q, size)) != size) {         // send a byte   
	      //printf("error writing to i2c slave in Robot::quei2c\n");
	   }
	contacts++;
	 if (read(fd_, q, size) != size) {            // Read a byte 
	      //printf("Unable to read from slave in Robot::quei2c\n");
	      failed_reads++;
	   }
	contacts++;

	if(failed_writes > 0 || failed_reads > 0) {
		printf("in Robot::quei2c %d failed writes and %d failed reads\n",
			       	failed_writes, failed_reads);
		if(failed_writes>0){
			for(int i=0; i<4; i++){
				cout<<Rfails_[i]<<", "; 
			}
			cout<<"\n";
		}
	}
}

// sends a byte que to the already selected I2C device 
// bytes returned from device replace sent messages in the que
void Robot::quei2c(int size, unsigned char *q){
	failed_reads = 0;
	failed_writes = 0;
	for(int i=0; i<size; i++){
		unsigned char send[1] = {q[i]};
		//cout<<" sending que with fd = " << fd<<"\n";
		if ((write(fd_, &send, 1)) != 1) {         // send a byte   
		      //printf("error writing to i2c slave in Robot::quei2c\n");
		      Rfails_[i]++;
		      failed_writes++;
		   }
		contacts++;
		 if (read(fd_, send, 1) != 1) {            // Read a byte 
		      //printf("Unable to read from slave in Robot::quei2c\n");
		      failed_reads++;
		   }
		contacts++;
	 q[i] = send[0];
	}
	if(failed_writes > 0 || failed_reads > 0) {
		printf("in Robot::quei2c %d failed writes and %d failed reads\n",
			       	failed_writes, failed_reads);
		if(failed_writes>0){
			for(int i=0; i<4; i++){
				cout<<Rfails_[i]<<", "; 
			}
			cout<<"\n";
		}
	}
}

// send PWM signals to Arduino
bool Robot::setMotors(int trynum){
	if(debugDrive_) cout<<"running set motors\n";
	if(debugDrive_) cout<<"lPWM_ = "<<(int) lPWM_<<" rPWM_ = "<<(int) rPWM_<<"\n";
	checki2c();
	if (ioctl(fd_, I2C_SLAVE, addrDrive) < 0) {     
	   // Set the port options and set the address of the device we wish to speak to
	   ROS_ERROR("Unable to open port, make sure openI2C has run or someone else using it?");
	   return false;
   	}
	
	if(eStop_){
		lPWM_ = 0;
		rPWM_ = 0;
	}

	unsigned char que[4] = {lForward_, lPWM_, rForward_, rPWM_};
	if(trynum == 2){
			que[0] = lForward_ == 'a' ? 'w' : 'x';
			que[2] = rForward_ == 'c' ? 'y' : 'z';
		}
	unsigned char sent[4] = {que[0], lPWM_, que[2], rPWM_};

	//piI2C(4, que);
	quei2c(4,que);

	usingi2c_ = false;
	if(que[1] == sent[1] && que[3] == sent[3]
		&& que[2] == sent[2] && que[0] == sent[0])
       	{ return true; }
	if(1 || trynum == 2){
		ROS_ERROR("Robot::setMotors miscommunication");
		unsigned char fail[1] = {'s'};
		quei2c(1,fail);
				
		cout<<"Sent: ["; 
		for(int i=0; i<4; i++){
			if(i%2==0){
				cout<<sent[i]<<",";
			}
			else{
				cout<<(int) sent[i]<<",";
			}
		}
		cout<<" ]";
		cout<<" Got: ["; 
		for(int i=0; i<4; i++){
			if(i%2==0){
				cout<<que[i]<<",";
			}
			else{
				cout<<(int) que[i]<<",";
			}
		}
		cout<<" ]\n";
	}
	/*cout<<"sent lPWM_ = "<<(int) lPWM_<<" and rPWM_ = "<<(int) rPWM_<<
		" but got "<<(int) que[1]<<" and "<<(int) que[3]<<"\n"; //*/
	return false;
}

bool Robot::getEncoders(){
	checki2c();
	if (ioctl(fd_, I2C_SLAVE, addrDrive) < 0) {     
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

// tries to open the I2C port and set fd, repeates 10 times if failing
void Robot::openI2C(){
//	fd_ = wiringPiI2CSetup(0x11);
	
   const char *fileName = "/dev/i2c-1";         // Name of the port we will be using
   int count = 0;
    while ((fd_ = open(fileName, O_RDWR)) < 0 && count < 10) {   // Open port for reading and writing
	    sleep(1);
      printf("Failed to open i2c port, did you set sudo??\n trying again...");
	  if (count == 10) {
		ROS_ERROR(" Could not open I2C port, aborting mission\n");
	    //exit(1);
	  }
   }
	ROS_INFO("Successfully opened I2C port");
	cout<<"fd = "<<fd_<<"\n";
	

}

// waits for i2c to be available
void Robot::checki2c(){
	while(usingi2c_){ /* put a delay here?*/ }
	usingi2c_ = true;
}

bool Robot::contactArms(){
	checki2c();
	if (ioctl(fd_, I2C_SLAVE, addrDrive) < 0) {     
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


