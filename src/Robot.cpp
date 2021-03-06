/* Robot.cpp
 *
 */

#include "Robot.h"
#include "Nav.h"
#include "lidar.h"

// check while navStack.size()>0 to wait while navigating
void Robot::mainLogic(){
	//odomWorldLoc_   << 0,0,0; // starting pose/position
	//odomWorldLoc_   << WheelDist, 23.5,0; // start at back left w/ steel block
	//odomWorldLoc_   << 123,WheelDist,PI2/4; // start at center of back wall
	//testDistToStop();
	//odomWorldLoc_   << 0,0,90.0*PI/180.0; // starting at corner of STEM lines on floor 
	//EndPoint go1(80+WheelDist, 89.5, -1, tmp); // to go from line crosses on stem basement
	
	eStop_ = false;
	speed_ = 0;
	cout<<"started PID while stopped speed = "<<speed_<<"\n";
	runPID_ = true;
	
	odomWorldLoc_   << 15+2, 34.5, 0; // start at back left w/ steel block
	int start = 2; // nearest waypt at beginning
	int initial = start;
	//vector<int> rooms{19,2,8,12}; // R1 - R4 waypts (for predetermined map)
	//vector<int> rooms{2,8,12,19}; // R1 - R4 waypts (for predetermined map)
	vector<int> rooms{2,8,12,17}; // R1 - R4 waypts (for predetermined map)
	int Center = 18;
	pt2pt_ = true;

	for(int r=0; r<4; r++){ // don't check starting room
		if(rooms[r] != initial){
			vector<int> path = nav_->findWay(start, rooms[r]);

			cout<<"making path from "<<start<<" to "<<rooms[r]<<"\n";
			for(int i=0; i<path.size(); i++){
				cout<<path[i]<<"  ";
			}
			cout<<"\n";

			buildNavStack(path);
			nav_->highlightWays(navStack);
			start = executeNavStack(); // go to room (waits on this line till arrived)
			if(searchNDestroy()){
			       cout<<"mission completed exiting room nav\n";
		       	       break;
			}
		}
	}

	cout<<" DONE CHECKING ALL ROOMS, DONE MAIN LOGIC\n";
}

bool Robot::searchNDestroy(){
	int waypt = lid_->findCandle();
	if(waypt == -1){
	        setPose_+=40;
		sleep(2); // wait to stop
		waypt = lid_->findCandle(); // run again at offset in case hidden by struts
	}
	if(waypt != -1){
		vector<int> toCandle{waypt};
		buildNavStack(toCandle);
		pt2pt_ = true;
		executeNavStack();
		sprayNpray(3);
		return true;
	}
	return false;
}
void Robot::sprayNpray(int num){
	float degreeSwing = 20;
	float centerPose = odomWorldLoc_(2)*180./PI;
	speed_ = 0;
	speed2power(0); // make sure it's stopped
	float power = 0.15;
	runPID_ = false; // going manual
	ramp_ = false; 
	
	cout<<"starting sprayNpray...\n";
	setSprayer(true);
	for(int i=0; i<num; i++){
		for(int k=0; k<2; k++){
			lDrive_ =  k%2==0 ? power : -1*power; // starts turning to the right
			rDrive_ =  k%2==0 ? -1*power : power;
			power2pwm();
			float target = k%2==0 ? centerPose - degreeSwing : centerPose + degreeSwing;
			float error = ab(odomWorldLoc_(2)*180./PI - target);
			while( error > SamePoseThreshDeg){
				error = ab(odomWorldLoc_(2)*180./PI - target);
			}
		}
	}
	setSprayer(false);

	runPID_ = true;
	usleep(1000*500); // give it a half second to reorient
}

void Robot::pinThread(){
	bool sw1, sw2, sw3;
	bool sw = false;
	cout<<"starting pin thread\n";
	while(1){
		/*sw1 = digitalRead(sw1Pin);
		sw2 = digitalRead(sw2Pin);
		sw3 = digitalRead(sw3Pin);
		digitalWrite(blueLEDPin, sw1 ? HIGH : LOW);
		digitalWrite(redLEDPin,  sw2 ? HIGH : LOW);
		digitalWrite(greenLEDPin,sw3 ? HIGH : LOW);*/
		digitalWrite(blueLEDPin, sw ? HIGH : LOW);
		digitalWrite(redLEDPin,  sw ? HIGH : LOW);
		digitalWrite(greenLEDPin,sw ? HIGH : LOW);
		digitalWrite(sprayPin, sw ? LOW : HIGH);
		//digitalWrite(sprayPin,HIGH);
		cout<<"pins are: "<<sw<<"\n";

		if(sw){
			usleep(1000*1000);
		}
		else{
			usleep(1000*3000);
		}
		sw = !sw;

		//cout<<"sw1 = "<<sw1<<" sw2 = "<<sw2<<" sw3 = "<<sw3<<"\n";
		//int ir1 = analogRead(IR1Pin);
		//int ir2 = analogRead(IR2Pin);
		//cout<<"ir1 = "<<ir1<<" ir2 = "<<ir2<<" sw = "<<sw<<"\n";
	}
}


Robot::Robot() : posePID_(0,0,0,0,0,0, &debugDrive_){ // also calls pose constructor
	// defaults values
	fudge_ = 0.949; // accounts for difference between right and left wheel
	speed_ = 0;      // start stopped
	//runSpeed_ = 0.5; // set speed to run at for this competition
	eStop_ = reversed_ = pt2pt_ = false;
	updateSavedPos_ = updateDriving_ = false;
	travelDist_ << 0,0,0;
	firstNav_ = facingFirst_ = true;
	ramp_ = firstRamp_ = false;
	robUpdateRate_ = mapUpdateRate_ = wayUpdateRate_ = 1;
	prevdist_ = 9999; // used for checking if missed a waypt

	debugDrive_ = runPID_ = false;
	lDrive_ = rDrive_ = 0;
	lForward_ = rForward_ = 'f';
	lPWM_ = rPWM_ = lEnc_ = rEnc_ = 0;
	ms_ = 20;

	D3_ = 0; D6_ = 40; D9_ = 127; D10_ = 180; D11_ = 255; // Arm PWM 
	power2pwm();
	srand(time(NULL));
	
	max_ = 0.4; min_ = 0.001; kp_ = 2; kd_ = 0; ki_ = 0; // PID gains
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
	pinMode(blueLEDPin, OUTPUT);
	pinMode(redLEDPin, OUTPUT);
	pinMode(greenLEDPin, OUTPUT);
	pinMode(sprayPin, OUTPUT);
	setSprayer(false);
	cout<<"turned sprayer off\n";

	pinMode(sw1Pin, INPUT);
	pinMode(sw2Pin, INPUT);
	pinMode(sw3Pin, INPUT);
	pinMode(IR1Pin, INPUT);
	pinMode(IR2Pin, INPUT);
	
	openSerial();
} 

void Robot::setSprayer(bool on){
	digitalWrite(sprayPin, on ? LOW : HIGH); // make sure it's off
}
void Robot::outputTime(clk::time_point t1, clk::time_point t2){
	cout<<"takes "<<
	stc::duration_cast <stc::milliseconds>(t2-t1).count()<<"ms and "<<
	stc::duration_cast <stc::microseconds>(t2-t1).count()<< "us\n";
}

bool Robot::buildNavStack(vector<int> ids, bool append){
	if(!append) navStack.resize(0);
	for(int id : ids){
		EndPoint ep = nav_->getWayPoint(id);
		if(ep.getX() == -1 && ep.getY() == -1) {
			navStack.resize(0);
			cout<<"TRIED TO ADD NONEXISTENT ENDPOINT TO NAVSTACK!\n";
			return false;
		}
		navStack.push_back(nav_->getWayPoint(id));
	}
	return true;
}

bool Robot::insertToNavStack(int id){
	EndPoint ep = nav_->getWayPoint(id);
	if(ep.getX() == -1 && ep.getY() == -1) {
		navStack.resize(0);
		cout<<"TRIED TO ADD NONEXISTENT ENDPOINT TO NAVSTACK!\n";
		return false;
	}
	navStack.push_front(nav_->getWayPoint(id));
	return true;
}

// stopping distance from 0.5 speed in 0.5s is 12.5cm
// stopping distance from 0.5 speed in 1s is 22.5cm
// acceleration distance from 0 to 0.5 in 0.5s is 6.44cm
// therefore if the point being navigated to is less than 12.5 + 6.44 use 0.2 speed
// OPERATION: pt2pt mode runs at 0.2 speed and stops at every waypoint in navStack
// default mode faces first point then runs at 0.5 speed with rounding of corners
int Robot::executeNavStack(){ // runs in parallel to driveLoop, called from mainLogic
	// TODO - update pose to next point when LIDAR updates position
	unsigned int count = 0; // used for periodic output
	while(navStack.size()>0){
		float dist = distToNextPoint(); // distance from next waypoint
		int pts = navStack.size();
		
		if(firstNav_){ // if starting to navigate!
			firstNav_ = false;
			speed_ = 0; // make sure we are starting from a stationary position
			//ramp_ = false; // MIGHT NEED THIS

			// if the robot is already at the first point (within threshold)
			if(dist < WayPointStartThreshold){ // 2cm, everthing within this accuracy
				cout<<"removed starting point, too close\n";
				navStack.pop_front();
			}

			// start turning to face first (or next closest) point
			setPose_ = getPoseToPoint(navStack.front()); // 
			cout<<"starting to face first with pose = "<<setPose_<<"\n";
			facingFirst_ = true; // wait for first turn
		}

		else if(facingFirst_){ // turning to face first point
			speed_ = 0; // for some reason this wasn't 0... only other place it gets set is ramp
			ramp_ = false; // ramp is running for some reason here....
			float error = fmod(ab(odomWorldLoc_(2)*180.0/PI - setPose_), 360.0);
			error = error > 180 ? 360 - error : error; 
			if(error < SamePoseThreshDeg /*|| ab(adj_) < 0.1*/){ // PID has finished turning, no more adj needed
				//cout<<"done facing first\n";
				facingFirst_ = false;
				float s = dist < MinDistFor50 || pt2pt_ ? 0.2 : 0.5; // if it's very close use 20%
				setRamp(s, 0.5); // start driving to next point, take half second to accelerate
			}
			else if(count++%10000000 == 0){ // has gotten stuck here in the past, this output indicates if it's hanging here
				cout<<"still in face first with error: "<<error<<"\n";
			}
		}

		else if(navStack.size() == 1 || pt2pt_){ // slowing down as approaching final point
			if( (ab(ab(speed_) - 0.5) < 0.05 && dist < StopDist50) ||
			    (ab(ab(speed_) - 0.2) < 0.05 && dist < StopDist20) ||
			    (dist>prevdist_+1 && dist<10.0)){ // stop if reached stopping distance or now overshooting the way pt by 1 cm
				setRamp(0, 0.5);
				msleep(500);
				cout<<"Popping marker "<<navStack.front().getID()<<" because arrived at loc\n";
				if(dist>prevdist_+1)cout<<"dist: "<<dist<<" prevdist_: "<<prevdist_<<"\n";

				int lastpt = navStack.front().getID();

				navStack.pop_front();
				firstNav_ = true; // this is either the last point or a pt2pt nav so reset 
				if(navStack.size() == 0) return lastpt;
			}
			else if(updateDriving_){
				setPose_ = getPoseToPoint(navStack.front()); 
				updateDriving_ = false;
			}
		}

		else if(navStack.size()>1){ // fluid 90deg turns assumes running at 0.5 speed (!pt2pt_)
			float poseToNextPoint = getPoseToPoint(navStack.front());  // basically the current pose
			float poseAfterTurn = getPoseToPoint(navStack.front(), &navStack.at(1)); // find pose between next two pts
			float turnDiff = poseAfterTurn - poseToNextPoint; // amount it will need to turn
			if(turnDiff < -180) turnDiff += 360;
			if(turnDiff > 180) turnDiff += 360;
			turnDiff = ab(turnDiff);

			if(turnDiff < SamePoseThreshDeg){ 
				cout<<"Popping marker "<<navStack.front().getID()<<" because same angle\n";
				cout<<"poseToNextPoint = "<<poseToNextPoint<<" poseAfterTurn = "<<poseAfterTurn<<"\n";
				cout<<"setPose = "<<setPose_<<" and poseToNextPoint = "<<poseToNextPoint<<" and turnDiff = "<<turnDiff<<"\n";
				navStack.pop_front();
				// if approaching waypoint at an angle and arriving close to it turnDiff could be low but pose needs to change
				setPose_ = getPoseToPoint(navStack.front());
			}
			else if(dist < StartBigTurnDist50 && turnDiff > 70){ // if large turn start early
				setPose_ = poseAfterTurn;	
				cout<<"Popping marker "<<navStack.front().getID()<<" and turning to "<<setPose_<<" because BIG turning\n";
				navStack.pop_front();
			}
			else if(dist < StartSmallTurnDist50){
				setPose_ = poseAfterTurn;	
				cout<<"Popping marker "<<navStack.front().getID()<<" and turning to "<<setPose_<<" because SMALL turning\n";
				navStack.pop_front();
			}
			/*else if(dist>prevdist_+2){
				setPose_ = poseAfterTurn;	
				cout<<"Popping marker "<<navStack.front().getID()<<" and turning to "<<setPose_<<" because MISSED it\n";
				navStack.pop_front();
			}*/
			else if(updateDriving_){
				// recalculate in case updates between poseToNextPoint being made and this line
				setPose_ = getPoseToPoint(navStack.front()); 
				updateDriving_ = false;
			}
		}
		
		if(pts > navStack.size()){ // if a waypoint was popped
			cout<<"popped marker updating rviz\n";
			prevdist_ = 99999; // a marker was popped so reset dist
			travelDist_(2) -= 5;
			nav_->highlightWays(navStack);
		} 
		else {prevdist_ = dist;}
	}// while loop
}

// determines distance between robot pos (odomWorldLoc_) and next point in navstack
float Robot::distToNextPoint(){
	if(navStack.size()>0){
		float xdiff = navStack.front().getX() - odomWorldLoc_(0);
		float ydiff = navStack.front().getY() - odomWorldLoc_(1);
		return pow( pow(xdiff,2) + pow(ydiff,2), 0.5);
	}

	cout<<"Robot::distToNextPoint called on empty stack!!!\n";
	exit(1);
	return -1.0;

}


// returns in Degrees from 0-360!
float Robot::getPoseToPoint(EndPoint pt, EndPoint* pt2){
	float t;
	if(pt2 == NULL){ // defualt if ommitted
		pt.calcPolar(odomWorldLoc_(0),odomWorldLoc_(1)); // find polar with robot as origin
		t = pt.getCalculatedTheta();
	}
	else{ // get angle to pt2 with pt as the origin
		pt2->calcPolar(pt.getX(),pt.getY()); // find polar with robot as origin
		t = pt2->getCalculatedTheta();

	}
	
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

	return t;
}

void Robot::driveLoop(){
	cout<<"talking to arduino... \n";
	delay_ = 20; // also the default in Recon.cfg, this is the us delay for waiting for enc values after pinging arduino
	getSerialEncoders(); // just make sure arduino is connected
	cout<<"got encoders\n";

	nav_->pubWays_ = true;
	nav_->pubMap_ = true;
	nav_->pubRob_ = true;
	usleep(5*ms_); // sleep 5 loops before starting

	cout<<"starting driveLoop\n";
	while(1){

		clk::time_point time_limit = clk::now() + stc::milliseconds(ms_);
		auto t1 = stc::steady_clock::now(); // measure length of time remaining
		getSerialEncoders(); 
		auto t2 = stc::steady_clock::now(); // measure length of time remaining
		calculateOdom(); 
		rampUpSpeed();
		
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
			// sprayNpray function will cause this line to run
		}

		auto t5 = stc::steady_clock::now(); // measure length of time remaining
		setSerialMotors(); // sends lPWM_ and rPWM_ to motors
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
		if(PID_time > 80){
			cout<<"Arduino reset inside PID!!!\n\n";
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
			cout<<"debug says facingFirst_ = "<<facingFirst_<<"\n";
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
		nav_->setOdomLoc(odomWorldLoc_); // give Nav the location to publish
		nav_->pubRob_ = true;
		robCount_ = 0;
	}

}

EndPoint Robot::transformEndPoint(EndPoint ep){
	// multiply by inverse of rotation matrix
	Vector2f pt;
	pt << ep.getX(), ep.getY();
	pt = rob2world_.topLeftCorner(2,2)*pt;

	// add translation of bot
	pt(0) = pt(0) + odomWorldLoc_(0);
	pt(1) = pt(1) + odomWorldLoc_(1);
	ep.setCart(pt(0),pt(1));
	return ep;
}

void Robot::updatePosition(Vector3f newPos){
	updateSavedPos_ = true;
	newPos_ = newPos; // saves new var so it can be integrated at in calculateOdom (don't just hard change it)
}

// Increment locX, locY, locP with the new encoder vals
void Robot::calculateOdom(){
	bool debug = 0;//debugDrive_;
	float lRad = (PI2 * (float) lEnc_ ) / (1470.0); // 1470.0 enc counts / rotation??
	float rRad = (PI2 * (float) rEnc_ ) / (1470.0); 

	float x = WheelRad / 2.0 * (lRad + rRad);
	float y = 0;
	float p = WheelRad / (2.0 * WheelDist) * (rRad - lRad);
	robotstep_ <<  x, y, p; // this is in the robot (lidar) frame

	if(!updateSavedPos_){ 
		float theta = odomWorldLoc_(2) + robotstep_(2)/2.0; // this is in radians!
		calculateTransform(theta);	      // find transform using half the step	
		worldstep_ = rob2world_*robotstep_; 
		odomWorldLoc_ += worldstep_;
		Vector3f v(x,x,x);
		travelDist_ += v; // keeps track of dist traveled between updates in x,y,theta (Nav resets w/ pass by ref)
	}
	else{ // set variables for updating the position, only gets sent when there is new data from Nav
		for(int i=0; i<3; i++){ 
			if(newPos_(i) != 0){ // some positions may not get updated (default to 0)
			       	odomWorldLoc_(i) = newPos_(i);
			}
		}
		if(newPos_(2)!=0) calculateTransform(odomWorldLoc_(2));
		if(navStack.size()>0)  updateDriving_ = true; // adjust heading if driving 
		travelDist_(2) = 0;
		updateSavedPos_ = false;
	}
	//if(debug) cout<<"odomWorldLoc_ = \n"<<odomWorldLoc_<<"\n";	
}

void Robot::pubTransformContinual(int rate){
	while(1){
		tf::Transform trans;
		//trans.setOrigin(tf::Vector3(experimental_(0), experimental_(1), 0)); // x,y,0 cm shift, could be problematic...
		trans.setOrigin(tf::Vector3(odomWorldLoc_(0), odomWorldLoc_(1), 0)); // x,y,0 cm shift, could be problematic...
		tf::Quaternion q;
		//q.setRPY(0,0,experimental_(2)); // radian shift
		q.setRPY(0,0,odomWorldLoc_(2)); // radian shift
		trans.setRotation(q);
		tfTrans_ = tf::StampedTransform(trans, ros::Time::now(), GLOBALFRAME, ROBOTFRAME);
		//br_.sendTransform(tf::StampedTransform(trans, ros::Time::now(), GLOBALFRAME, ROBOTFRAME));
		br_.sendTransform(tfTrans_);
		msleep(1/(float) rate * 1000.0);
	}
}

// Given the robot's pose relative to the world calculate rob2world_.
void Robot::calculateTransform(float theta){
	rob2world_.topLeftCorner(2,2) << cos(theta), -sin(theta),
					 sin(theta),  cos(theta);
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
			//cout<<"Starting ramp... rampInc_ = "<<rampInc_<<" speed = "<<speed_<<"\n";
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
			//cout<<"exiting ramp... speed = "<<speed_<<"\n";
		}	
	}
}

//////////////////// Power and Speed Handling for serial //////////////////
void Robot::speed2power(float adj){ // note uses local adj not member
	lDrive_ = speed_ - adj;
	rDrive_ = speed_*fudge_ + adj;	
	/*if(!reversed_){
	  lDrive_ = speed_ - adj;
	  rDrive_ = speed_*fudge_ + adj;	
	  }
	  else {
	  lDrive_ = (-1.0 * speed_) + adj;
	  rDrive_ = (-1.0 * speed_)*fudge_ - adj;	
	  }*/
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


// ^^^^^^^^^^^^^^^^^^^^^^^ START SERIAL FUNCTIONS ^^^^^^^^^^^^^^^^^^^^^^^
void Robot::openSerial(){
	cout<<"opening USB connection \n";
	//fd_ = open("/dev/??", O_RDWR | O_NOCTTY | O_NDELAY); // read+write | not controlling term | ? 
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

bool Robot::getSerialEncoders(){
	int16_t mcode2[1] = {-2}; // cannot be from 0-255 (use negatives)
	write(fd_, mcode2, 2); // Identify what info is coming next 

	// normal method
	usleep(delay_);
	//usleep(5000);
	unsigned char encs[4];
	int code = read(fd_, encs, 4); // read 2 integers from the arduino

	if(code == 4){
		if(!reversed_){
			//lEnc_ = (enc1[0]<<8) | enc2[0]; // single chare method
			//rEnc_ = (enc3[0]<<8) | enc4[0];
			lEnc_ = (encs[0] << 8) | encs[1]; 
			rEnc_ = (encs[2] << 8) | encs[3]; 
		}
		/*
		else{ // when reversed swap which wheel is which and the direction read
			cout<<"reversed problem in getSerialEncs\n";
			rEnc_ = (encs[0] << 8) | encs[1]; 
			lEnc_ = (encs[2] << 8) | encs[3]; 
			rEnc_ *= -1;
			lEnc_ *= -1;
			}*/
		return true;
	}
	cout<<"Could not read encs, code = "<<code<<"\n";
	return false;
	//if(lEnc_ != 0) cout<<"left enc is "<<lEnc_<<"\n";
	//if(rEnc_ != 0) cout<<"right enc is "<<rEnc_<<"\n\n";
}

void Robot::setSerialMotors(){
	if(eStop_){
		lPWM_ = 0;
		rPWM_ = 0;
	}
	int16_t mcode[1] = {-1}; // cannot be from 0-255 (use negatives)
	unsigned char mdata[4] = {lForward_, lPWM_, rForward_, rPWM_};	
	write(fd_, mcode, 2); // Identify what info is being sent
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

// ^^^^^^^^^^^^^^^^^^^^^^^ END SERIAL FUNCTIONS ^^^^^^^^^^^^^^^^^^^^^^^

////////////////// SIMPLE FUNCTIONS ////////////////////////////////
void Robot::setNav(Nav* nv){ nav_ = nv; }
void Robot::setLidar(Lidar* lid){ lid_ = lid; }
Vector3f Robot::getOdomWorldLoc(){return odomWorldLoc_;};
Ref<Vector3f> Robot::getTravelDist(){ return travelDist_;}
tf::StampedTransform Robot::getTransform(){ return tfTrans_;}

void Robot::setExperimental(Vector3f pose){experimental_ = pose;}

float Robot::toRad(float deg){ return deg*PI2/360.0; }
void Robot::msleep(int t){ usleep(1000*t); }
