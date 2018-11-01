/* Robot.cpp
 *
 */

#include "Robot.h"
#include "Nav.h"
#include <string>
#include <iostream>
#include <fstream>


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
using std::string;

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
			cfsetispeed(&options, B9600);
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
			char in[1];
			while(1){
				int bytes = read(fd, &in, sizeof(in));

				if(bytes>0){
					std::cout<<"read a byte which said: ";
				       	printf("%c\n", in[0]);
				}
				//write(fd, "q",1);
				//usleep(800000); // wait 100ms
			}

		}



		/*std::cout<<"starting way1\n";
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
