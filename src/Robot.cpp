/* iobot.cpp
 *
 */

#include "Robot.h"
#include "Nav.h"
#include <string>
#include <iostream>
#include <fstream>

// for serial
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <chrono>
using std::string;

// for spi
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
//*/
#include <wiringPi.h>
 
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

void Robot::sendArduino(int code){
	std::cout<<"the given code was: "<<code<<" \n";
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
//void openSensorI2C(){
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
         if ((write(fd, &send, 2)) != 2) {            // Send register we want to read from   
	      printf("Error writing to i2c slave\n");
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
int Robot::getEncoder(bool left){

	return 0;
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
/*void Robot::serial(char send[], int size){
	std::cout<<"in thread. waiting for USB connection...\n";

	int fd;
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd == -1){
		std::cout<<"could not open serial port...\n";
		perror("open_port: Unable to open /dev/ttyUSB0 - ");
	}
	else{
		struct termios options;
		tcgetattr(fd, &options);
		cfsetispeed(&options, B9600); // SET SPEED
		cfsetospeed(&options, B9600);
		options.c_cflag |= (CLOCAL | CREAD);
		
		// 8N1 options
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;     // 8 bit chars
		options.c_cflag &= ~PARENB; // no parity
		options.c_cflag &= ~CSTOPB;
		
		tcsetattr(fd, TCSANOW, &options);

		std::cout<<" Connected!\n";
		fcntl(fd, F_SETFL, 0);

		//write(fd, send, size); // send data over serial
		std::cout<<"sent bytes\n";
		if(send[0] == 'l' || send[0] == 'r'){
			char in[4]; // one float is 4 bytes
		//	int bytes = read(fd, &in, sizeof(in)); // receive data over serial
			float f;
			memcpy(&f, &in, sizeof(f));
			std::cout<<"read a float response = "<<f<<"\n";
		}
	}
}
*/
Nav* Robot::getNavPtr(){ return &beSmart;}
