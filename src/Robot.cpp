/* Robot.cpp
 *
 */

#include "Robot.h"

// check while navStack.size()>0 to wait while navigating
void Robot::mainLogic(){
	//odomWorldLoc_   << 0,0,0; // starting pose/position
	//odomWorldLoc_   << WheelDist, 23.5,0; // start at back left w/ steel block
	//odomWorldLoc_   << 123,WheelDist,PI2/4; // start at center of back wall
	eStop_ = false;
	speed_ = 0;
	
	// TODO -- only works w/ 0.3 speed, something else must also be at 0.3 to make it work... 
	cout<<"started PID while stopped speed = "<<speed_<<"\n";

	//testDistToStop();
	odomWorldLoc_   << 0,0,90.0*PI/180.0; // starting pose/position
	runPID_ = true;
	vector<int> tmp;
	EndPoint go(80+WheelDist, 89.5, -1, tmp); // to go from line crosses on stem basement
	navStack.push_back(go);
}

Robot::Robot() : posePID_(0,0,0,0,0,0, &debugDrive_){ // also calls pose constructor
	// defaults values
	fudge_ = 0.949; // accounts for difference between right and left wheel
	speed_ = 0;      // start stopped
	runSpeed_ = 0.5; // set speed to run at for this competition
	eStop_ = reversed_ = positionUpdated_ = false;
	ramp_ = firstRamp_ = false;

	debugDrive_ = runPID_ = false;
	lDrive_ = rDrive_ = 0;
	lForward_ = rForward_ = 'f';
	lPWM_ = rPWM_ = lEnc_ = rEnc_ = 0;
	ms_ = 20;

	D3_ = 0; D6_ = 40; D9_ = 127; D10_ = 180; D11_ = 255; // Arm PWM 
	power2pwm();
	srand(time(NULL));
	
	max_ = 0.4; min_ = 0.001; kp_ = 1; kd_ = 0; ki_ = 0; // PID gains
	posePID_.setVals(ms_, max_, min_, kp_, kd_, ki_) ;
	wayCount_ = mapCount_ = robCount_ = debugCount_ = 0;

	// initialize vectors, if not it won't compile!
	rob2world_ << 1, 0, 0,   0, 1, 0,   0, 0, 1; // as if theta = 0
	robotstep_ << 0,0,0; 
	worldstep_ << 0,0,0;

	odomWorldLoc_   << 0,0,0; // starting pose/position
	//odomWorldLoc_   << WheelDist,13.5,0; // start at back left w/ steel block
	//odomWorldLoc_   << 123,WheelDist,PI2/4; // start at center of back wall

	wiringPiSetup();
	openSerial();
} 

void Robot::outputTime(clk::time_point t1, clk::time_point t2){
	cout<<"takes "<<
	stc::duration_cast <stc::milliseconds>(t2-t1).count()<<"ms and "<<
	stc::duration_cast <stc::microseconds>(t2-t1).count()<< "us\n";
}

// stopping distance from 0.5 speed in 0.5s is 12.5cm
// stopping distance from 0.5 speed in 1s is 22.5cm
// acceleration distance from 0 to 0.5 in 0.5s is 6.44cm
// therefore if the point being navigated to is less than 12.5 + 6.44 use 0.2 speed
void Robot::executeNavStack(bool pt2pt){
	// TODO - update pose to next point when LIDAR updates position

	if(navStack.size()>0){
		float dist = distToNextPoint();
		if(startNavStack_){ // if starting to navigate!
			startNavStack_ = false;
			speed_ = 0; // make sure we are starting from a stationary position

			// if the robot is already at the first point (within threshold)
			if(dist < WayPointStartThreshold){ // 2cm, everthing within this accuracy
				navStack.pop_front();
			}
			/*else{ // if too far away from first point start turning to face it
			}*/

			setPose_ = getPoseToPoint(navStack.front()); // 
			facingFirst_ = true; // wait for first turn
		}
		else if(facingFirst_){ // turning to face first point
			if(ab(adj_) < 0.1){ // PID has finished turning, no more adj needed
				facingFirst_ = false;
				float s = dist < MinDistFor50 ? 0.2 : 0.5; // if it's very close use 20%
				setRamp(s, 0.5); // start driving to next point
			}

		}

		else if(navStack.size() == 1 || pt2pt){ // slowing down as approaching final point
			if( (ab(ab(speed_) - 0.5) < 0.05 && dist < StopDist50) ||
			    (ab(ab(speed_) - 0.2) < 0.05 && dist < StopDist20)){
				setRamp(0, 0.5);
				cout<<"Popping marker "<<navStack.front().getID()<<" because arrived at loc\n";
				navStack.pop_front();
				if(pt2pt) startNavStack_ = true;
				cout<<"reached final point in navStack!\n";
			}
		}
		else if(navStack.size()>1){ // fluid 90deg turns assumes running at 0.5 speed (!pt2pt)
			float poseToNext = getPoseToPoint(navStack.front()); 
			if(ab(setPose_ - poseToNext) < SamePoseThreshDeg){
				cout<<"Popping marker "<<navStack.front().getID()<<" because same angle\n";
				navStack.pop_front();
			}
			else if(positionUpdated_){
				// TODO - recalculate Pose to be used based on new position and wayPoint
				setPose_ = poseToNext; 

			}
			else if(dist < StartTurnDist50){
				// TODO - calculate next pose (usually factor of 90) and set it
				
				cout<<"Popping marker "<<navStack.front().getID()<<" because turning\n";
			}
		}
	}
	else if(navStack.size() == 0){ // done last navigation waiting to start again
	   	startNavStack_ = true;
	}
}

// determines distance between robot pos (odomWorldLoc_) and next point in navstack
float Robot::distToNextPoint(){
	if(navStack.size()>0){
		float xdiff = navStack.front().getx() - odomWorldLoc_(0);
		float ydiff = navStack.front().gety() - odomWorldLoc_(1);
		return pow( pow(xdiff,2) + pow(ydiff,2), 0.5);
	}

	cout<<"Robot::distToNextPoint called on empty stack!!!\n";
	exit(1);
	return -1.0;

}


// returns in Degrees!
float Robot::getPoseToPoint(EndPoint pt, EndPoint* pt2){
	float x = pt2 == NULL ? odomWorldLoc_(0) : pt2->getx();
	float y = pt2 == NULL ? odomWorldLoc_(1) : pt2->gety();
	pt.calcPolar(x,y); // find polar with robot as origin
	float t = pt.getTheta();
	
	// TODO after getting stack nav working uncomment this to reverse bot when needed
/*	if(ab(setPose_ - t) > 90){ // if the turn is > 90 reverse the bot
		// right wheel becomes left and vice versa in all instances
		// by swapping lDrive and rDrive in the following locations
		// 1. speed2power
		// 2. odometry (getting enc vals)
		// 3. before PID somewhere?
		// 4. 

		reversed_ = !reversed_; 
	}*/

	return pt.getTheta();
}

// This is the main thread that runs and controls the 'bot the mainLogic
// loop simply adjusts variables that this thread is referencing.
// Flow:
// 	1. Get Encoder counts via arduino
// 	2. Determine new world location from counts
// 	3. Check if the speed is being ramped and if so increment speed_
// 	4. Check if a navStack is running and if so set pose accordingly
// 	5. Run the PID and and calculate lDrive and rDrive
// 	6. Set the motors via arduino
// 	7. Output debug info (not every loop) includes rviz map, ways, debugDrive_
// 	8. Wait time remaining such that this loop took 20ms (or ms_ ms)
void Robot::driveLoop(){
	cout<<"talking to arduino... \n";
	delay_ = 20; // also the default in Recon.cfg
	getSerialEncoders(); // just make sure arduino is connected
	cout<<"got encoders\n";

	nav_->pubWays_ = true;
	nav_->pubMap_ = true;
	nav_->pubRob_ = true;
	usleep(5*ms_); // sleep 5 loops before starting
	robUpdateRate_ = mapUpdateRate_ = wayUpdateRate_ = 1;

	cout<<"starting driveLoop\n";
	while(1){

		clk::time_point time_limit = clk::now() + stc::milliseconds(ms_);
		auto t1 = stc::steady_clock::now(); // measure length of time remaining
		getSerialEncoders(); 
		auto t2 = stc::steady_clock::now(); // measure length of time remaining
		calculateOdom(); 
		rampUpSpeed();
		executeNavStack();
		
		if(runPID_){ // this should almost always be the one running
			if(debugDrive_) cout<<"in pid, odomWorldLoc_ = "<<odomWorldLoc_(0)
			<<" x "<<odomWorldLoc_(1)<<" y "<<360/PI2*odomWorldLoc_(2)<< " thet\n";

			// give the PID the desired and current rotations
			// note: 0 is robot front, +0 turns left, -0 turns right
			adj_ = posePID_.calculate(setPose_ * PI2/360.0, odomWorldLoc_(2));
			speed2power(adj_);
			if(debugDrive_) cout<<"set = "<<setPose_<<" P - "<<kp_<<" I - "<<ki_
				<<" D - "<<kd_<<"\n"<<"speed = "<<speed_<<"adj = "<<adj_<<"\n\n";
		}
		else{ // if PID gets turned off for some odd reason...
			speed2power(0);
		}
		/*
		else{ // honestly this shouldn't even be running...
			if(reversed_) {ROS_ERROR("calling power2pwm when reversed which is bad...");}
			cout<<"running just power2pwm!!!\n";
			power2pwm(); // already run by speed2power in PID
		}*/

		auto t5 = stc::steady_clock::now(); // measure length of time remaining
		setSerialMotors();
		auto t6 = stc::steady_clock::now(); // measure length of time remaining
		if(0 && debugDrive_) { // usually unneeded but just in case enable
			cout<<"speed = "<<speed_<<"Drive_= "<<lDrive_<<" ("<<(int)lPWM_<<
				") rDrive_ = "<<rDrive_<<" ("<<(int)rPWM_<<")\n";
		}

		periodicOutput(); // check what needs to be output
		auto t7 = stc::steady_clock::now(); // measure length of time remaining


		/////////////////////////////////////////////////////////////////////
		//////// Rest in this loop is just timing and some outputs //////////
		/////////////////////////////////////////////////////////////////////
		int PID_time = stc::duration_cast <stc::milliseconds>(t7-t1).count();
		if(PID_time > 14){
			cout<<"PID took over 14ms\n";
			/*cout<<"PID takes "<<
			stc::duration_cast <stc::milliseconds>(t7-t1).count()<<"ms and "<<
			stc::duration_cast <stc::microseconds>(t7-t1).count()<< "us\n";
			cout<<"t1 to next: "; outputTime(t1,t2);
			cout<<"t5 to next: "; outputTime(t5,t6);
			cout<<"t6 to next: "; outputTime(t6,t7);
			cout<<"speed = "<<speed_<<"adj = "<<adj_<<"\n\n";//*/
		}

		auto start = stc::steady_clock::now(); // measure length of time remaining
		std::this_thread::sleep_until(time_limit); // wait such that loop took ms_ long
		auto end = stc::steady_clock::now();

		int ms = stc::duration_cast <stc::milliseconds>(end-start).count();
		if(debugDrive_ || ms<0){ // if less than 4 ms left over
			if(ms<4){ 
				 cout<<"in pid, odomWorldLoc_ = "<<odomWorldLoc_(0)
			<<" x "<<odomWorldLoc_(1)<<" y "<<360/PI2*odomWorldLoc_(2)<< " thet\n";

				cout<<"speed = "<<speed_<<"adj = "<<adj_<<"\n\n";
				ROS_INFO("Problem with timing!! (less than 4ms leftover)");
			}
			else{ cout<<odomWorldLoc_<<"\n";}

			cout<<"driveLoop takes "<< ms_ <<" ms with "<<
			stc::duration_cast <stc::milliseconds>(end-start).count()<<"ms and "<<
			stc::duration_cast <stc::microseconds>(end-start).count()<<
			"us (hopefully) leftover\n";
			cout<<"speed = "<<speed_<<"\n";
		}
	}
}

void Robot::periodicOutput(){
	mapCount_++; wayCount_++; robCount_++; debugCount_++;
	/*if(debugDrive_){
		debugDrive_ = false;
		cout<<"\n";
	}
	if(debugCount_ > 150){ // 50 counts = 1s (ms_ = 20);
		debugDrive_ = true;
		debugCount_ = 0;
	}//*/
	
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

// Increment locX, locY, locP with the new encoder vals
void Robot::calculateOdom(){
	bool debug = 0;//debugDrive_;
	float lRad = (PI2 * (float) lEnc_ ) / (1470.0); // 1470.0 enc counts / rotation??
	float rRad = (PI2 * (float) rEnc_ ) / (1470.0); 
	if(debug) cout<<"lRad = "<<lRad<<" rRad = "<<rRad<<"\n";

	float x = WheelRad / 2.0 * (lRad + rRad);
	float y = 0;
	float p = WheelRad / (2.0 * WheelDist) * (rRad - lRad);
	robotstep_ <<  x, y, p;
	//if(debug) cout<<"robot step:\n"<<robotstep_<<"\n";
	if(debug) cout<<"odomWorldLoc_ = \n"<<odomWorldLoc_<<"\n";	
	float theta = odomWorldLoc_(2) + robotstep_(2)/2.0; // radians!

	if(debug) cout<<"using theta = "<<theta<<" for rob2world_\n";
	calculateTransform(theta);	      // find transform using half the step	
	//if(debug) cout<<"rob2world_ =\n"<<rob2world_<<"\n";
	worldstep_ = rob2world_*robotstep_; 
	odomWorldLoc_ += worldstep_;

	//if(debug) cout<<"odomWorldLoc_ = \n"<<odomWorldLoc_<<"\n";	
}

// Given the robot's pose relative to the world calculate rob2world_.
void Robot::calculateTransform(float theta){
	rob2world_.topLeftCorner(2,2) << cos(theta), -sin(theta),
					 sin(theta),  cos(theta);
}

// ^^^^^^^^^^^^^^^^^^^^^^^ START SERIAL FUNCTIONS ^^^^^^^^^^^^^^^^^^^^^^^
void Robot::openSerial(){
	cout<<"opening USB connection \n";
	//fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY); // read+write | not controlling term | ? 
	fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY); // read+write | not controlling term 

	perror("open_port: /dev/ttyUSB0 - ");
	if(fd_ == -1){
		cout<<"could not open serial port\n";
		exit(1);
	}

	struct termios options;
	tcgetattr(fd_, &options);
	//cfsetispeed(&options, B9600); // 115200 b/s input and output
	//cfsetospeed(&options, B9600);
	cfsetispeed(&options, B115200); // 115200 b/s input and output
	cfsetospeed(&options, B115200);
	options.c_cflag &= ~CSIZE;      // no bit mask for data bits
	options.c_cflag |= CS8; 	// 8 bit chars
	options.c_cflag &= ~PARENB;	// no parity
	options.c_cflag &= ~CSTOPB;     // only 1 stop bit

	//options.c_oflag = 0;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // allow raw input

	// THESE TWO LINES ARE CRITICAL FOR TIMING WHEN READING!!
	// see: https://stackoverflow.com/questions/20154157/termios-vmin-vtime-and-blocking-non-blocking-read-operations 
	options.c_cc[VTIME] = 1; // wait 0.1s between bytes before timing out on reads
	options.c_cc[VMIN] = 0; // must read at least 0 bytes (don't wait if it's not there!)


	tcsetattr(fd_, TCSANOW, &options);

	fcntl(fd_, F_SETFL, 0); // set the file status flag
	cout<<"done Robot::openSerial, letting arduino reset\n";
	sleep(3);
	
}

void Robot::setSerialMotors(){
	if(eStop_){
		lPWM_ = 0;
		rPWM_ = 0;
	}
	int16_t mcode[1] = {-1}; // cannot be from 0-255 (use negatives)
	unsigned char mdata[4] = {lForward_, lPWM_, rForward_, rPWM_};	
	write(fd_, mcode, 2); // Identify what info is coming next 
	usleep(50);
	write(fd_, mdata, 4);
}

void Robot::setSerialArms(){
	int16_t mcode[1] = {-3}; // code is -3, then sends 5 bytes for PWMs
	write(fd_, mcode, 2); 
	//usleep(500);
	unsigned char mdata[5] = {D3_, D6_, D9_, D10_, D11_ };
	write(fd_, mdata, 5);
}

bool Robot::getSerialEncoders(){
	int16_t mcode2[1] = {-2}; // cannot be from 0-255 (use negatives)
	unsigned char encs[4];
	write(fd_, mcode2, 2); // Identify what info is coming next 
	usleep(delay_);
	int code = read(fd_, encs, 4); // read 2 integers from the arduino
	if(code == 4){
		if(!reversed_){
			lEnc_ = (encs[0] << 8) | encs[1]; 
			rEnc_ = (encs[2] << 8) | encs[3]; 
		}
		else{ // when reversed swap which wheel is which and the direction read
			rEnc_ = (encs[0] << 8) | encs[1]; 
			lEnc_ = (encs[2] << 8) | encs[3]; 
			rEnc_ *= -1;
			lEnc_ *= -1;
		}
		return true;
	}
	cout<<"Could not read encs, code = "<<code<<"\n";
	return false;
	//if(lEnc_ != 0) cout<<"left enc is "<<lEnc_<<"\n";
	//if(rEnc_ != 0) cout<<"right enc is "<<rEnc_<<"\n\n";
}
// ^^^^^^^^^^^^^^^^^^^^^^^ END SERIAL FUNCTIONS ^^^^^^^^^^^^^^^^^^^^^^^

void Robot::speed2power(float adj){ // note uses local adj not member
	if(!reversed_){
		lDrive_ = speed_ - adj;
		rDrive_ = speed_*fudge_ + adj;	
	}
	else {
		lDrive_ = (-1.0 * speed_) + adj;
		rDrive_ = (-1.0 * speed_)*fudge_ - adj;	
	}
	power2pwm();
}

// scales -1:1 to 0:255
void Robot::power2pwm(){
	//if(debugDrive_) cout<<"making pwms with lDrive_ = "<<lDrive_<<" rDrive_ = "<<rDrive_<<"\n";
	lPWM_ = lDrive_ >= 0 ? lDrive_*255.0 : -1*lDrive_*255.0;
	rPWM_ = rDrive_ >= 0 ? rDrive_*255.0 : -1*rDrive_*255.0;
	lForward_ = lDrive_ >= 0 ? 'f' : 'b'; // directions set by char for each motor
	rForward_ = rDrive_ >= 0 ? 'f' : 'b';
	//if(debugDrive_) cout<<"made pwms lPWM_= "<<(int)lPWM_<<" rPWm= "<<(int)rPWM_<<"\n";
	//if(debugDrive_) cout<<"lforward: "<<lForward_<<"  rforward: "<<rForward_<<"\n";
}

void Robot::setRamp(float s, float t){
	rampTime_ = t;
	rampSpeed_ = s;
	firstRamp_ = true;
	ramp_ = true; 
}

void Robot::rampUpSpeed(){
	if(ramp_){
		if(firstRamp_){
			rampInc_ = (rampSpeed_ - speed_) / (rampTime_ / (ms_ / 1000.0));
			firstRamp_ = false;
			cout<<"Starting ramp... rampInc_ = "<<rampInc_<<" speed = "<<speed_<<"\n";
		}
		speed_ = speed_ + rampInc_;
		speed2power(0);
		// if ramp has overshot or is very close to value
		//cout<<"Ramping... rampInc_ = "<<rampInc_<<" speed = "<<speed_<<"\n";
		if((rampSpeed_ == 0 && ab(speed_) < ab(rampInc_)) ||
			(rampSpeed_ < 0 && speed_ < rampSpeed_) ||
			(rampSpeed_ > 0 && speed_ > rampSpeed_)){

			speed_ = rampSpeed_;
			ramp_ = false;
			cout<<"exiting ramp... speed = "<<speed_<<"\n";
		}	
	}
}


float Robot::toRad(float deg){ return deg*PI2/360.0; }
void msleep(int t){ usleep(1000*t); }
void Robot::setNav(Nav* nv){ nav_ = nv; }

void Robot::testDistToStop(){
	// Testing 90deg turn distance overshoot for setting StartTurnDist50 and StartTurnDist20
	/*
	odomWorldLoc_   << 0,0,0; // starting pose/position
	eStop_ = false;
	speed_ = 0;

	cout<<"set = "<<setPose_<<" P - "<<kp_<<" I - "<<ki_
	<<" D - "<<kd_<<"\n"<<"speed = "<<speed_<<"adj = "<<adj_<<"\n\n";
	reversed_ = false;
	setRamp(0.5, 0.5); // slowly accelerate
	runPID_ = true;

	sleep(2);
	Vector3f tmp = odomWorldLoc_;
	setPose_ = -90;
	msleep(800);
	cout<<"diff is: "<<odomWorldLoc_ - tmp<<"\n";
	speed_ = 0; */

	// Testing to find MinDistFor50, StopDist50, and StopDist20
	// Note: MinDist includes acceleration and decelleration distances
/*	
	reversed_ = false;
	setRamp(0.2, 0.5); // slowly accelerate
	runPID_ = true;
	cout<<"waiting to accelerate!!!!!!!!!!!!!!!!\n";
	while(speed_ !=0.2) {
		cout<<"speed = "<<speed_<<"\n";
		if(speed_ == 0.2) break;
		if(ab(speed_ - 0.2) < 0.001) break;
	} //wait to stop
	cout<<"done waiting to accelerate!!!!!!!!!!!!!!!!\n";
	usleep(10);
	cout<<"0.2 acceleration distance = \n"<<odomWorldLoc_<<"\n";
	while(odomWorldLoc_(0) < 30) {;} //wait to stop
	Vector3f after_acc = odomWorldLoc_;
	setRamp(0, 0.5);
	while(speed_ !=0) {;}
	cout<<"stopping distance = \n"<<odomWorldLoc_-after_acc<<"\n";
	//cout<<"total distance = \n"<<odomWorldLoc_<<"\n";*/
}

void Robot::moveInSquare(){
	int count = 0;
	setPose_ = 0;
	runPID_ = true;
	Vector3f temp;
	while(1){
		int idx = count%4;

		if(idx==0 && odomWorldLoc_(0) > 150){
			temp = odomWorldLoc_;
			setPose_ = 90 + 360;
			sleep(2);
			cout<<"set Pose to "<<setPose_<<" idx = "<<idx<<"\n"
			<<" difference in x from turn: "<<odomWorldLoc_(0) - temp(0)<<"\n";
			count++;

		}
		else if(idx==1 && odomWorldLoc_(1) > 150){
			temp = odomWorldLoc_;
			setPose_ = 180;
			sleep(2);
			cout<<"set Pose to "<<setPose_<<" idx = "<<idx<<"\n"
				<<" diff in y from turn: "<<odomWorldLoc_(1) - temp(1)<<"\n";
			count++;

		}	
		else if(idx==2 && odomWorldLoc_(0) < 50){
			setPose_ = 270;
			cout<<"set Pose to "<<setPose_<<" idx = "<<idx<<"\n";
			sleep(2);
			count++;

		}	
		else if(idx==3 && odomWorldLoc_(1) < 50){
			setPose_ = 0;
			cout<<"set Pose to "<<setPose_<<" idx = "<<idx<<"\n";
			sleep(2);
			count++;
			//sleep(2);
			//speed_=0;
		}	
	}
}

// if you use this expect things to break also it runs automatically
// on startup so default values will be whatever is in cfg/Recon.cfg
/*void Robot::recon(firebot::ReconConfig &config, uint32_t level){ 
	eStop_ 	 = config.estop;
	//mapUpdateRate_ = config.maprate;
	//wayUpdateRate_ = config.wayrate;
	//robUpdateRate_ = config.robrate;
	delay_ = config.delay; 

	//lDrive_ = config.left;
	//rDrive_ = config.right;
	debugDrive_ = config.debug;
	//useSpeed_ = config.usespeed;
	//if(speed_ != config.speed) speedChange_ = true;
	speed_ = config.speed;
	//fudge_ = config.fudge;

	/*
	if(rampSpeed_ != config.rampspeed){
		cout<<"%\n%\n%\n%\n%\n%\n%\n%\n%\n%\n%\n%\n%\n%\n%\n%\n%\n%\n%\n%\n";
		rampSpeed_ = config.rampspeed;
		rampTime_  = config.ramptime;
		firstRamp_ = true;
		ramp_ = true;
	}//*/
	/*
	if(useSpeed_){
		lDrive_ = speed_;
		rDrive_ = speed_ * fudge_;
	}//*/

	//power2pwm();

	//runPID_  = config.runpid;
	//setPose_ = config.setpose;
	// do these need to be member variables? probs not
/*
	kp_ = config.kp;
	ki_ = config.ki;
	kd_ = config.kd;
	max_ = config.max;
	min_ = config.min;

	posePID_.setVals(ms_, max_, min_, kp_, kd_, ki_) ;
	cout<<"RECONFIGURED!\n\n";
}//*/


