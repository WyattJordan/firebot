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
/*
void Robot::i2c(){
	int fd;                     // File descrition
   const char *fileName = "/dev/i2c-1";         // Name of the port we will be using
   int  address = 0x11;               // Address of CMPS03 shifted right one bit
   unsigned char buf[1000];            // Buffer for data being read/ written on the i2c bus

	std::ifstream fin;
	fin.open("/home/wyatt/i2cin.txt");
	std::cout<<"opened i2cin.txt\n";
	char ch;
	int pos = 0;
	while(fin >> std::noskipws >> ch){
		buf[pos] = ch;
		std::cout<<pos<<" ||| "<<ch<<"\n";
		pos++;
	}

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
     for(int i=0; i<200; i+=2){
	  unsigned char tmp[2]; 
	  tmp[0] = buf[i];
	  tmp[1] = buf[i+1];
    if ((write(fd, tmp, 2)) != 2) {            // Send register we want to read from   
      printf("Error writing to i2c slave\n");
      exit(1);
   }
     }

  for(int i=0; i<200; i+=2){
  unsigned char tmp[2]; 
	   if (read(fd, tmp, 2) != 2) {            // Read back data into buf[]
	      printf("Unable to read from slave\n");
	      exit(1);
	   }
	   else{
		   buf[i]   = tmp[0];
		   buf[i+1] = tmp[1];
	   }
   }
   
	auto end = std::chrono::steady_clock::now();		
		std::cout << "Elapsed time in microseconds : "
		<< std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
		<< " us" << std::endl;

   /*for(int i=0; i<200; i++){
		   std::cout<<"buf["<<i<<"] = "<<buf[i]<<"\n";
	   }
	 
	std::ofstream f;
	f.open("/home/wyatt/i2cout.txt");
	int count = 0;
	std::cout<<"writing to file\n";
	while(count < 200){
		f<<buf[count++];
	}

	f<<std::endl;
	f.close();
	
	
}*/
/*
void Robot::serial(char send[], int size){
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

Nav* Robot::getNavPtr(){ return &beSmart;}*/
