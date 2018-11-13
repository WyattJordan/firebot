/* Robot.cpp
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
	   }*/
	 
	std::ofstream f;
	f.open("/home/wyatt/i2cout.txt");
	int count = 0;
	std::cout<<"writing to file\n";
	while(count < 200){
		f<<buf[count++];
	}

	f<<std::endl;
	f.close();
	
}

void Robot::spi(){
	wiringPiSetup();
	while(1){

		usleep(800000); // wait 100ms
		static const char *device = "/dev/spidev1.0";
		static uint8_t mode = 0;
		static uint8_t bits = 8;
		static uint32_t speed = 100000;
		static uint16_t delay;
		int ret = 0;
		int fd;
	 
		fd = open(device, O_RDWR);
		if (fd < 0)
			pabort("can't open device");
	 
		/*
		 * spi mode
		 */
		ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
		if (ret == -1)
			pabort("can't set spi mode");
	 
		ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
		if (ret == -1)
			pabort("can't get spi mode");
	 
	/*
		 * bits per word
		 */
		ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
		if (ret == -1)
			pabort("can't set bits per word");
	 
		ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
		if (ret == -1)
			pabort("can't get bits per word");
	 
		/*
		 * max speed hz
		 */
		ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
		if (ret == -1)
			pabort("can't set max speed hz");
	 
		ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
		if (ret == -1)
			pabort("can't get max speed hz");
	 
		printf("spi mode: %d\n", mode);
		printf("bits per word: %d\n", bits);
		printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);


		uint8_t tx[] = {
			0xCC, 0x15, 0xF2, 0x0A,
		//	0x48, 0x49, 0x20, 0x54, 0x48, 0x45, 0x52, 0x45, // HI THERE
		};
		printf("size of tx is: %d\n", ARRAY_SIZE(tx));
		uint8_t rx[ARRAY_SIZE(tx)] = {0, };
		struct spi_ioc_transfer tr;
		
		memset(&tr, 0, sizeof(tr));
		tr.tx_buf = (unsigned long)tx;
		tr.rx_buf = (unsigned long)rx;
		tr.len = ARRAY_SIZE(tx);
		tr.delay_usecs = delay;
		tr.speed_hz = speed;
		tr.bits_per_word = bits;
		tr.cs_change = 0;

		ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
		if (ret < 1)
			pabort("can't send spi message");
	 
		for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
			if (!(ret % 8))
				puts("");
			printf("%.2X ", rx[ret]);
		}
		puts("");
	}
 
}
void Robot::serial(){
	sleep(1);
	std::cout<<"in thread. waiting for USB connection...\n";
	
	bool way1 = 1;
	if(way1){

		int fd;
		fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
		if(fd == -1){
			std::cout<<"could not open serial port...\n";
			perror("open_port: Unable to open /dev/ttyUSB0 - ");
			
		}
		else{
			struct termios options;
			tcgetattr(fd, &options);
			cfsetispeed(&options, B230400); // SET SPEED
			cfsetospeed(&options, B230400);
			options.c_cflag |= (CLOCAL | CREAD);
			
			// 8N1 options
			options.c_cflag &= ~CSIZE;
			options.c_cflag |= CS8;     // 8 bit chars
			options.c_cflag &= ~PARENB; // no parity
			options.c_cflag &= ~CSTOPB;
			
			tcsetattr(fd, TCSANOW, &options);

			std::cout<<" Connected!\n";
			fcntl(fd, F_SETFL, 0);
			int size = 1000;
			char send[size];

			char in[1];
			std::ifstream fin;
			fin.open("/home/wyatt/usbin.txt");
			std::cout<<"opened usbin.txt\n";
			char ch;
			int pos = 0;
			while(fin >> std::noskipws >> ch){
				send[pos] = ch;
				pos++;
			}
			std::cout<<"made send buffer\n";
			// TAKE A TIMESTAMP HERE 
			auto start = std::chrono::steady_clock::now();		
			write(fd, send, size);
			std::ofstream f;
			f.open("/home/wyatt/usbout.txt");
			int count = 0;
			std::cout<<"sent bytes, going to read\n";
			while(count < 200){
				int bytes = read(fd, &in, sizeof(in));

				if(bytes>0){
					f<<in[0];
					count++;
				}
				//write(fd, "q",1);
				//usleep(800000); // wait 100ms
			}
			f<<std::endl;
			f.close();

			auto end = std::chrono::steady_clock::now();
			std::cout << "Elapsed time in nanoseconds : "
		<< std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()
		<< " ns" << std::endl;

	std::cout << "Elapsed time in microseconds : "
		<< std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
		<< " us" << std::endl;

	std::cout << "Elapsed time in milliseconds : "
		<< std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
		<< " ms" << std::endl;

	std::cout << "Elapsed time in seconds : "
		<< std::chrono::duration_cast<std::chrono::seconds>(end - start).count()
		<< " sec";

		} 
		/*std::std::cout<<"starting way1\n";
		FILE *file;
		while(1){
			file = fopen("/dev/ttyUSB0 ", "w");
			std::cout<<"firing q\n";
			fprintf(file,"q\n");
			fclose(file);
		}//*/
	}
	
	else{
		
		std::ifstream file("/dev/ttyUSB0");
		
		string line;
		std::cout<<"starting way2\n";
		if(file.is_open()){

			while(getline(file,line)){
				std::cout<<line<<"\n";
				std::cout<<"testing "<<tmp;
			}
		}
		else{
			std::cout<<"can't open file...\n";
		}
	}
}

Nav* Robot::getNavPtr(){ return &beSmart;}
