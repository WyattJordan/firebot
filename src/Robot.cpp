/* Robot.cpp
 *
 */

#include "Robot.h"
#include "Nav.h"
#include "pid.h"
#include <string>
#include <iostream>
#include <fstream>
#include <wiringPi.h>

/* for serial
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>*/
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

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

Robot::Robot() : posePID(0,0,0,0,0,0){ // also calls pose constructor
	failed_reads = contacts = 0;
	maxleft = maxright = 0;
	left255 = right255 = 0;
	usingi2c = false;
	lDrive = rDrive = 0;
	D3 = 0;
	D6 = 40;
	D9 = 127;
	D10 = 180;
	D11 = 255;
	power2pwm();
} 
void Robot::loadMapAndWayPoints(int lvl){
	if(lvl == 3){
		Nav tmp("lvl3_map.txt", "dummy");  //root is catkin ws
		beSmart = tmp;
	}	
	else {
		Nav tmp("/home/wyatt/cat_ws/src/firebot/lvl1_map.txt", 
				"/home/wyatt/cat_ws/src/firebot/wayPoints.txt");	
		beSmart = tmp;
	}
}

// scales -1:1 to 0:255
void Robot::power2pwm(){
	lPWM = (unsigned char) lDrive*127+127;
	rPWM = (unsigned char) rDrive*127+127;
}

// tries to open the I2C port and set fd, repeates 10 times if failing
void Robot::openI2C(){
   const char *fileName = "/dev/i2c-1";         // Name of the port we will be using
   int count = 0;
    while ((fd = open(fileName, O_RDWR)) < 0 && count < 10) {   // Open port for reading and writing
      printf("Failed to open i2c port, did you set sudo??\n trying again...");
	  if (count == 10) {
		ROS_ERROR(" Could not open I2C port, aborting mission\n");
	    exit(1);
	  }
   }
	ROS_INFO("Successfully opened I2C port");
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
		if ((write(fd, &send, 1)) != 1) {         // send a byte   
		      printf("error writing to i2c slave in Robot::contactDrive\n");
		      exit(1);
		   }

		 if (read(fd, send, 1) != 1) {            // Read a byte 
		      printf("Unable to read from slave in Robot::contactDrive\n");
		      failed_reads++;
	   }

	 q[i] = send[0];
	}

}
// sends the drive pwm levels and gets the encoder counts
bool Robot::contactDrive(){
	contacts++;
	checki2c();
	if (ioctl(fd, I2C_SLAVE, addrDrive) < 0) {     
	   // Set the port options and set the address of the device we wish to speak to
	   ROS_ERROR("Unable to open port, someone else using it?");
	   return false;
	   }

	// do 2 interactions one for each motor, each interacion sends and receives 2 bytes
	// 		odroid sends l/r and lPWM/rPWM
	// 		arduino sends upper and lower bytes of int count
	unsigned char que[4] = {'l',lPWM,'r',rPWM};
	quei2c(4,que);
	usingi2c = false;

	lEnc = (que[0] << 8) | que[1]; // left is first, high byte is first
	rEnc = (que[2] << 8) | que[3];
	
	if(lEnc == 255 || rEnc == 255) return false; // very rarely it fails

	// debug code
	/*if(abs(lEnc)>maxleft) maxleft = lEnc;
	if(abs(rEnc)>maxright) maxright = rEnc;
	if(lEnc == 255) left255++;
	if(rEnc == 255) right255++;
	std::cout<<"lEnc = "<<lEnc<<"  maxleft = "<<maxleft<<" 255 count = "<<left255<<"\n";
	std::cout<<"rEnc = "<<rEnc<<"  maxright = "<<maxright<<" 255 count = "<<right255<<"\n";
	std::cout<<"Failed reads: "<<failed_reads<<" out of "<<contacts<<"\n";// */
	return true;
}

// i2c example code for testing
void Robot::i2c(){
   int fd;                     // File descrition
   const char *fileName = "/dev/i2c-1";         // Name of the port we will be using
   int  address = 0x11;               // Address of CMPS03 shifted right one bit
    if ((fd = open(fileName, O_RDWR)) < 0) {   // Open port for reading and writing
      printf("Failed to open i2c port, did you set sudo??\n");
      exit(1);
   }
   
   if (ioctl(fd, I2C_SLAVE, address) < 0) {     
	   // Set the port options and set the address of the device we wish to speak to
      printf("Unable to get bus access to talk to slave\n");
      exit(1);
   }
   std::cout<<"sending data to sensors\n";
	unsigned char send[2];
	send[0] = 'w';
	send[1] = 'e';
         if ((write(fd, &send, 2)) != 2) {            // send register we want to read from   
	      printf("error writing to i2c slave\n");
	      exit(1);
	   }

   std::cout<<"reading from sensors \n";
  int readSize = 2;
  unsigned char get[readSize]; 
  get[0] = 'x';
  get[1] = 'y';
	   if (read(fd, get, readSize) != readSize) {            // Read back data into buf[]
	      printf("Unable to read from slave\n");
	      exit(1);
	   }
	   else{
		   for(int i=0; i<readSize; i++){
			   std::cout<<"byte "<<i<<" :" << get[i]<<"\n"; // data byte 1
		   }
	   }
	   std::cout<<"opening motor arduino\n";

 if (ioctl(fd, I2C_SLAVE, 0x10) < 0) {     
	   // Set the port options and set the address of the device we wish to speak to
      printf("Unable to get bus access to talk to slave\n");
      exit(1);
   }

 std::cout<<"reading from motor arduino\n ";
  get[0] = 'x';
  get[1] = 'y';
	   if (read(fd, get, readSize) != readSize) {            // Read back data into buf[]
	      printf("Unable to read from slave\n");
	      exit(1);
	   }
	   else{
		   for(int i=0; i<readSize; i++){
			   std::cout<<"byte "<<i<<" :" << get[i]<<"\n"; // data byte 1
		   }
	   }
	   std::cout<< "Done and can use same port opening different slaves\n";
 

}

/*
void Robot::i2c(){
	std::cout<<"started I2C connection \n";
   int fd;                     // File descrition
   const char *fileName = "/dev/i2c-1";         // Name of the port we will be using
   int  address = 0x11;               // Address of CMPS03 shifted right one bit
   unsigned char buf[1000];            // Buffer for data being read/ written on the i2c bus

   if ((fd = open(fileName, O_RDWR)) < 0) {   // Open port for reading and writing
      printf("Failed to open i2c port, did you set sudo??\n");
      exit(1);
   }
   
   if (ioctl(fd, I2C_SLAVE, address) < 0) {     
	   // Set the port options and set the address of the device we wish to speak to
      printf("Unable to get bus access to talk to slave\n");
      exit(1);
   }
  
	auto start = std::chrono::steady_clock::now();		
	// should the tmp have the & ?
	unsigned char send[2];
	send[0] = 'w';
	send[1] = 'e';
         if ((write(fd, &send, 2)) != 2) {            // Send register we want to read from   
	      printf("Error writing to i2c slave\n");
	      exit(1);
	   }
  int readSize = 2;
  unsigned char get[readSize]; 
  get[0] = 'x';
  get[1] = 'y';
	   if (read(fd, get, readSize) != readSize) {            // Read back data into buf[]
	      printf("Unable to read from slave\n");
	      exit(1);
	   }
	   else{
		   for(int i=0; i<readSize; i++){
			   std::cout<<"byte "<<i<<" :" << get[i]<<"\n"; // data byte 1
		   }
	   }
   
	auto end = std::chrono::steady_clock::now();		
		std::cout << "Elapsed time in microseconds : "
		<< std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
		<< " us" << std::endl;
}
*/
Nav* Robot::getNavPtr(){ return &beSmart;}
