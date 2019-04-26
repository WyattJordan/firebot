#include "lidar.h"
#include "Robot.h" // these two classes were forward declared
#include "Nav.h"
void Lidar::processData(const sensor_msgs::LaserScan::ConstPtr& scan){
	int num = scan->scan_time / scan->time_increment;
	degrees_.resize(0);
	rad_.resize(0);
	xVal_.resize(0); 
	yVal_.resize(0);

	// formats the data into the 4 vectors rad_, degrees_, xVal_, and yVal_
	for(int i = 0; i < num; i++) { 
		float radius = scan->ranges[i];
		if ((isinf(radius) == 0)&&(radius > MINDIST)){ // check if acceptable range measurement
			if(radius > MAXDIST) radius = MAXDIST; // makes the jump detection easier
			float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
			float ang = degree>0 ? degree : degree + 360;
			rad_.push_back(radius);
			degrees_.push_back(ang);
			xVal_.push_back(POLAR2XCART(radius, ang));
			yVal_.push_back(POLAR2YCART(radius, ang));
		}
		else{
			/*if(isinf(radius) == 0){
				cout << "radius distance: " << radius << endl << "Angle: " << RAD2DEG(scan->angle_min + scan->angle_increment*i) << endl;
			}*/
		}
	}
	//removes the first and last two points because they are usually bad datapoints
	removePt(0);
	removePt(0);
	removePt(rad_.size()-1);
	removePt(rad_.size()-1);

	for(int i=0; i < rad_.size(); i++){ // remove points from the studs or near their angles
		if((degrees_[i] > 37 && degrees_[i] < 41)   ||
		   (degrees_[i] > 137 && degrees_[i] < 141) ||
		   (degrees_[i] > 319 && degrees_[i] < 323) ||
		   (degrees_[i] > 219 && degrees_[i] < 223)){
			removePt(i);
			i--;
		}
	}
}

void Lidar::input(){
	while(1){
		cin.get();
		keypress_ = true;
		findCandle();
		cout<<"key pressed...\n";
	}
}

void Lidar::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
	if(executing_) return; // if the last laserscan is still being processed
	executing_ = true;
	bool linearMove = false;
	//tf::StampedTransform oldTrans; // save values immediately if going to be used (computing all this will take time
	ros::Time scanStamp = scan->header.stamp;
	Vector3f currentPos = rob_->getOdomWorldLoc(); // this is an undefined ref for some reason...
	auto t1 = stc::steady_clock::now(); // measure length of time remaining
	if(/*tickCount_++%LidarUpdateRate==0  &&*/ ! (prevOdom_(0) == -100 && prevOdom_(1) == -100)){ // if not the first run (default odom loc) 
		// if the angle has changed less than 5deg between the two positions
		if(abs(prevOdom_(2) - currentPos(2))*180/PI < 2.0) linearMove = true;
		//if(linearMove) oldTrans = rob_->getTransform(); // save current copy
	}//*/

	processData(scan); // populates rad_, degrees_, xVal_, yVal_ with pt data (all in Lidar frame)

	if(keypress_ || checkCandle_){
		//cout<<"gathering candle data...\n";
		findJumps(false);   // populates jumps_ and smallJumps_, bool determines if looking for big jumps
		findFurniture();   // determines what is furniture from smallJump_ 
		nav_->makeFurnMarks(furns_); // publishfurniture in rviz
		findLines(true); // pub segments on
		nav_->makeLineMarks(lines_, true, true);
		//locateCandle();
		//cleanBigJumps();   // removes furniture jumps and any jumps counted twice 
		//startRooms_.push_back(classifyRoomFromJumps()); // used to determine room for starting location, will run findLines
		keypress_ = false;

	}
	else if(0 && startCount_++ < 10){ // classify the room multiple times before determining which room the 'bot is in 
		findJumps(true);   // populates jumps_ and smallJumps_, bool determines if looking for big jumps
		findFurniture();   // determines what is furniture from smallJump_ 
		nav_->makeFurnMarks(furns_); // publishfurniture in rviz

		cleanBigJumps();   // removes furniture jumps and any jumps counted twice 
		startRooms_.push_back(classifyRoomFromJumps()); // used to determine room for starting location, will run findLines

		if(startCount_ == 10){
			localRoom_ = modeRoom(); // just in case some bad classifications occured
			cout<<"found mode to be "<<localRoom_<<"\n";
		}
	}
	else if(0 && !started_ && localRoom_ != -1){ 
		cout<<"localizing to room "<<localRoom_<<"\n";
		started_ = checkLocalize();
		cout<<"done initial localize sequence\n";
	}
	/*else if(started_){ // testing picking room code by reseting on keypress
		nav_->makeLineMarks(lines_, true, true);
		if(startCount_++ > 50){
			startCount_ = 0;
			started_ = false;
			localRoom_ = -1;
			startRooms_.resize(0);
			cout<<"\n\n\n\n\n";
		}
	}*/
	// this is what runs most of the time while driving for updates
	else if(linearMove && !pauseUpdates_){ // normal scan update
		//cout<<"\nlinear movement! Going to process data!\n";
		findJumps(true);  // don't look for big jumps, just furniture
		findFurniture();   // determines what is furniture from smallJump_ 
		nav_->makeFurnMarks(furns_); // publish furniture in rviz
		findLines(true); // pub segments off, might want leave this off (too much processing)
		nav_->makeLineMarks(lines_, true, true);

		bool updated = nav_->updatePosition(lines_, currentPos, rob_->getTravelDist());
		if(updated) {
			auto end = stc::steady_clock::now();
			rob_->outputTime(t1, end);
		}
	}//*/

	prevOdom_ = currentPos;//*/
	executing_ = false;
}

bool Lidar::checkLocalize(){ // checks some conditions for a good scan to initially localize on
	findJumps(true);  // need big jumps to find the location of the local pt and for close jump verification
	findFurniture();   // determines what is furniture from smallJump_ 
	nav_->makeFurnMarks(furns_); // publish furniture in rviz
	findLines(true); // what really matters
	nav_->makeLineMarks(lines_, true, true); // publish lines in rviz
		
	if(localRoom_ == 1){
		room1Localization();
		return true;
	}
	else if(localRoom_ == 2){
		room2Localization();
		return true;
	}
	else if(localRoom_ == 3){
		room3Localization();
		return true;
	}
	else if(localRoom_ == 4 && cJumps_.size()==2){ 
		int r4long = 0;  // determine which side the door is on from longer range measurements
		for(float d : rad_) {if(d>160) r4long++;}
		bool down = r4long > 5;
		outsideWalls_.resize(0);
		for(int i=0; i<lines_.size(); i++){
			if(lines_[i].getClosestRadius() > 60) outsideWalls_.push_back(i);
		}
		if( outsideWalls_.size()>0 && ((down && outsideWalls_.size()<3) || (!down && outsideWalls_.size() == 1) )){
			room4Localization(down);
		}
		else{
			cout<<"room 4 not started_ based on outsideWalls num: "<<outsideWalls_.size()<<" with down = "<<down<<"\n";
			return false;
		}
		return true;
	}
	cout<<"room "<<localRoom_<<" localization conditions not met...\n";
	cout<<"cJumps_ size: "<<cJumps_.size()<<" lines_ size: "<<lines_.size()<<"\n";
	return false;
}

void Lidar::room1Localization(){ }
void Lidar::room2Localization(){ }
void Lidar::room3Localization(){ }

void Lidar::room4Localization(bool down){
	EndPoint ep1, ep2;
	if(down){	// adjust map model based on door config and set global localization pt as entrance to door
		ep1 = nav_->getMapPoint(11);
		ep2 = nav_->getMapPoint(19);
		nav_->setSmallRoomUpper(false);
		cout<<"room4 door faces (0,0)\n";
	}
	else{
		ep1 = nav_->getMapPoint(13);
		ep2 = nav_->getMapPoint(18);
		nav_->setSmallRoomUpper(true);
		cout<<"room4 door faces top of maze\n";
	}

	// point in the center of the doorway in the global frame determined by the map and in local frame from close jumps
	EndPoint gDoorPt((ep1.getX() + ep2.getX()) / 2.0, (ep1.getY() + ep2.getY()) / 2.0);
	int closest = getCloserJumpPt(cJumps_[0]);
	int nClosest = getCloserJumpPt(cJumps_[1]);
	EndPoint lDoorPt((xVal_[closest] + xVal_[nClosest]) / 2.0, (yVal_[closest] + yVal_[nClosest]) / 2.0);

	int wall = -1;
	if(outsideWalls_.size()!=1){ // in this case size = 2
		int wall1 = outsideWalls_[0];
		int wall2 = outsideWalls_[1];
		float dist1 = lines_[wall1].getClosestRadius();
		float dist2 = lines_[wall2].getClosestRadius();
		if(std::max(dist1, dist2) > 80){ // it's picking up the far wall of room2
			cout<<"picked wall based on eliminating far (over 80cm nearest)\n ";
			wall = dist1 < dist2 ? wall1 : wall2;
		}	
		else{ // use the wall with more points in it, the other one is noise (hopefully)
			cout<<"picked wall based on most points of outside walls\n";
			wall = lines_[wall1].numPts() > lines_[wall2].numPts() ? wall1 : wall2;
		}
	}
	else{
		cout<<"picked wall based on it being the only one\n";
		wall = outsideWalls_[0];
	}
	cout<<"using wall w/ idx: "<<wall<<" which has center "<<lines_[wall].getCenterRadius()<<" cm away at angle "<<lines_[wall].getCenterTheta()<<"\n";
	//	localizeFromPt(lDoorPt, gDoorPt);
}

// to detect furniture look at dist between endpoint of first jump and start of second jump
int Lidar::classifyRoomFromJumps(){
	cout<<"finding room from jumps... ";
	// Check room4 first, if there are two close big jumps (the doorway) it's room 4
		
	if(jump_.size()==0){
		cout<<"no big jumps found!!!\n";
		return 0;
	}
	else if(jump_.size()==1){// either room2 or room3, pick which based on size of max jump
		float max = 0; 
		for(int j : jump_){
			if(getFurtherJumpRadius(j) > max )
				max = getFurtherJumpRadius(j);
		}
		
		if(max > 50){
			cout<<"classified asroom 2\n";
			return 2;
		}
		else{
			cout<<"classified as room 3\n";
			return 3;
		}
	}
	else{ // pick either room 1 or 4 based on if the two closest big jumps are seperated by a DoorWidth
		// sort the jumps by closest
		std::sort(jump_.begin(),jump_.end(),[this](int a, int b) -> bool {return getCloserJumpRadius(a) < getCloserJumpRadius(b);});
		float dist = pt2PtDist(xVal_[getCloserJumpPt(jump_[0])],yVal_[getCloserJumpPt(jump_[0])],
					xVal_[getCloserJumpPt(jump_[1])],yVal_[getCloserJumpPt(jump_[1])]);
		if(abs(dist - DoorWidth) < DoorWidthTol){
			cout<<"classified as room4\n";
			return 4;
		}
		else{
			cout<<"classified as room1\n";
			return 1;
		}
	}
}

void Lidar::locateCandle(){
	// loop thru radii and search for a positive jump, then look for a negative jump 
	// if a new positive comes along replace the old
	// if a neg jump is found, find the dist between the two points, if within candle thresh, avg them and push back to candle
	int posJumpIdx;
	bool foundPos = false;
	float minJump = 3.5;
	for(int p=0; p<rad_.size()-1; p++){
		if(rad_[p+1] - rad_[p] > minJump){
			posJumpIdx = p;
			foundPos = true;
		}
		if(foundPos && rad_[p+1] - rad_[p] < minJump){
			int pt1 = getCloserJumpPt(posJumpIdx);
			int pt2 = getCloserJumpPt(p);
			float width = pt2PtDist(xVal_[pt1], yVal_[pt1], xVal_[pt2], yVal_[pt2]); 
		       if(abs(CandleWidth - width) < CandleWidthTolerance){
				float x = (xVal_[pt1] + xVal_[pt2])/2.0;
				float y = (yVal_[pt1] + yVal_[pt2])/2.0;
				cout<<"found a candle at angles: "<<degrees_[pt1]<<" and "<<degrees_[pt2]<<" with dists: "<<
					rad_[pt1]<<" and "<<rad_[pt2];	
				cout<<"  with coords: ("<<x<<","<<y<<")\n";
				EndPoint candle = EndPoint(x,y);
				candleLocs_.push_back(candle);
		       }
		}
	}
}

int Lidar::findCandle(){
	candleLocs_.resize(0);
	checkCandle_ = true; // start appending to que
	pauseUpdates_ = true;
	int numGood = 0;
	float avgX = 0;
	float avgY = 0;
	while(candleLocs_.size()<50){
		for(int i=0; i<candleLocs_.size(); i++){
			if(candleLocs_[i].getX() != -1 && candleLocs_[i].getY() != -1){
				avgX += candleLocs_[i].getX();
				avgY += candleLocs_[i].getY();
				numGood++;
			}
		}
		usleep(1000*800);
		cout<<"num good is: "<<numGood<<" out of total: "<<candleLocs_.size()<<"\n";
		numGood = 0;
       	}	
	checkCandle_ = false;
	numGood = 0;
	avgX = 0;
	avgY = 0;
	for(int i=0; i<candleLocs_.size(); i++){
		if(candleLocs_[i].getX() != -1 && candleLocs_[i].getY() != -1){
			avgX += candleLocs_[i].getX();
			avgY += candleLocs_[i].getY();
			numGood++;
		}
	}
	pauseUpdates_ = false;
	if(numGood < 20){ // at least 10 candle detections
	   cout<<"less than 20 detections, no candle...\n";
   	   return -1; // candle not found
	}

	avgX /= (float) numGood;
	avgY /= (float) numGood;

	// let Nav make the candle in rviz and the ID to return back to Robot
	return nav_->foundCandle(avgX,avgY);
}

// Furniture will have ~13cm difference between endpoints (within FurnWidthTolerance)
// And will have a point with a lower polar Radius between the two points (by at least FurnDepthTolerance)
void Lidar::findFurniture(){
	furns_.resize(0);
	furnIdxs_.resize(0);
	//cout<<"starting to find furn with "<<smallJump_.size()<<" furnjumps\n";
	/*cout<<"starting furn jumps at angles: ";
	for(int q=0; q<smallJump_.size(); q++) {cout<<"j:"<<smallJump_[q]<<" at "<<degrees_[smallJump_[q]]<<"--"<<getCloserJumpRadius(smallJump_[q])<<"  ";}
	cout<<"\n";//*/

	bool gotCandle = false;
	furnJumpsConfirmed_.resize(0);
	for(int j=0; j<smallJump_.size(); j++){

		// get the two x,y points that are the closer to the 'bot of the two jump points  
		int pt1 = getCloserJumpPt(smallJump_[j]);
		int pt2 = getCloserJumpPt(smallJump_[(j+1)%smallJump_.size()]);
		int step = abs(pt1-pt2);  // this is the number of points hitting the furniture besides the endpoints 
		bool atLoopAround = false; // behind the 'bot which is where the rad_ vector loops 
		int middle = (pt1 + step/2) % rad_.size();
		// check the width tolerance of the two points
		float width = pt2PtDist(xVal_[pt1], yVal_[pt1], xVal_[pt2], yVal_[pt2]); 
		//cout<<"width for jumps at angles: "<<degrees_[pt1]<<" and "<<degrees_[pt2]<<" is "<<width<<"\n";
		if(!(abs(FurnWidth - width) < FurnWidthTolerance)){ // if the distance between the points is larger than furniture size
			// check if it could be a candle
			if(checkCandle_){
			       if(abs(CandleWidth - width) < CandleWidthTolerance){
					float x = (xVal_[pt1] + xVal_[pt2])/2.0;
					float y = (yVal_[pt1] + yVal_[pt2])/2.0;
					cout<<"found a candle at angles: "<<degrees_[pt1]<<" and "<<degrees_[pt2]<<" with dists: "<<
						rad_[pt1]<<" and "<<rad_[pt2];	
					cout<<"  with coords: ("<<x<<","<<y<<")\n";
					EndPoint candle = EndPoint(x,y);
					candleLocs_.push_back(candle);
					gotCandle = true;
				}
			}//*/
			//cout<<"deleted jump "<<smallJump_[j]<<" due to width constraint width = "<<width<<"\n";
			continue; // skips the rest of the code in this iteration
		}	

		if(step > 200 ){ // if there is a massive number of points hitting the "furniture" it's looping around the 180 spot so swap order
			atLoopAround = true;
			step = rad_.size() - step - 2; // instead of counting the long way around the circle
			middle = (pt2 - step/2);
			middle = middle<0 ? middle+rad_.size() : middle;
		}

		bool endsAreBack = rad_[middle] < rad_[pt1] && rad_[middle] < rad_[pt2];
		if(!endsAreBack){ // both ends of the furniture should be further from the bot than the center
			//cout<<"deleted jump "<<smallJump_[j]<<" because both ends aren't less than middle at "<<middle<<" with radius "<<rad_[middle]<<"\n";
			continue;
		}
		if(step > 80){ // if this statement runs something is wrong, probably with the loop around
		cout<<"large number of pts on furniture! ("<<step<<") should be impossible to have over 70 pts "
				<<"w/ R = 15cm and theta step = 0.0126 (500pts/scan)\n";
			continue;
		}
		if(step < 5){ // should let you detect a 13cm object 147cm away (tstep = 0.126) or a 5cm object 56cm away
			cout<<"too few of pts on the furniture for jump "<<smallJump_[j]<<"\n";
			continue;
		}

		// check that the midpoint is closer to the robot to confirm it's furniture, this may be unnecessary 
		float avgOutsideRad = (rad_[pt1] + rad_[pt2]) / 2.0;
		float curveHeight = abs(avgOutsideRad - rad_[middle]);

		if(curveHeight > FurnDistTolerance && curveHeight < 10){ // inner should be at least certain amount closer
			// Endpoint x and y determined by midpoint of furniture (should be lowest radius)
			float x = POLAR2XCART(rad_[middle]+FurnWidth/2.0, degrees_[middle]); 
			float y = POLAR2YCART(rad_[middle]+FurnWidth/2.0, degrees_[middle]);
			EndPoint f(x,y);
			furns_.push_back(f);
			// add the points between pt1 and p2 inclusive to the furnIdxs_
			if(!atLoopAround){ for(int rem=pt1; rem<=pt2; rem++) furnIdxs_.push_back(rem); }
			else{ for(int rem=pt1; rem%rad_.size()!=pt2; rem++) furnIdxs_.push_back(rem%rad_.size()); }

			furnJumpsConfirmed_.push_back(smallJump_[j]); // add the two jump idxs to the confirmed list
			furnJumpsConfirmed_.push_back(smallJump_[(j+1)%smallJump_.size()]);
	//		j++; // next jump will be the end of this piece of furniture so skip that // not necesarily...
		}
		else{
			//cout<<"jump "<<smallJump_[j]<<" not counted as furn due to curveHeight\n";
		}
	}

	if(!gotCandle){
	       candleLocs_.push_back(nav_->getBadPoint());
       }

	// save_here = furnJumpsConfirmed_; // if you want to save the furn jumps use this line, don't overwrite smallJump_ tho
	/*cout<<jump_.size()<<" Big jumps at angles: ";
	for(int r=0; r<jump_.size(); r++) {cout<<degrees_[jump_[r]]<<"--"<<getCloserJumpRadius(jump_[r])<<"   ";}
	cout<<"\n";*/
	
	/*cout<<"furn jumps after filtering: ";
	for(int t=0; t<furnJumpsConfirmed_.size(); t++) {cout<<degrees_[furnJumpsConfirmed_[t]]<<"--"<<getCloserJumpRadius(furnJumpsConfirmed_[t])<<"   ";}
	cout<<"\n";//*/
}

void Lidar::cleanBigJumps(){
	// remove furniture from big jumps
	for(int fj : furnJumpsConfirmed_){
		if(jump_.size()!=0 && std::find(jump_.begin(),jump_.end(),fj) != jump_.end()){ // if the fj is in jump_
			jump_.erase(std::remove(jump_.begin(), jump_.end(), fj), jump_.end()); // erase it
		}
	}

	// remove doubles
	for(int j=0; j<jump_.size(); j++){
		int nextIdx = (j+1)%jump_.size();
		// if within 4deg and 4cm delete the latter one
		if(abs(getCloserJumpRadius(jump_[j]) - getCloserJumpRadius(jump_[nextIdx])) < 4  &&
				abs(degrees_[jump_[j]] - degrees_[jump_[j]]) < 4){
			jump_.erase(jump_.begin() + nextIdx);
			j--;
		}

	}

}

void Lidar::findJumps(bool findBig){
	jump_.resize(0);
	smallJump_.resize(0);
	outliers_.resize(0);
	int bad = 0;
	for(int i=0; i<rad_.size(); i++){
		float diff = abs(rad_[i] - rad_[(i+1)%rad_.size()]);
		if(diff > DoorJumpDist && findBig){ 
			float avgPre, avgPost; // new method using function
			getAveragePrePost(avgPre,avgPost,i+1,5); // this omits idx+1 because that could be the nasty pt

			if(abs(avgPre - avgPost) > DoorJumpDist*0.75){
				jump_.push_back(i);
				smallJump_.push_back(i);
			}
			else{
				outliers_.push_back(i);
			}
		}
		else if(diff > SmallJumpDist){ // furn jump has lower tolerance for detecting furniture
			// use a smaller averaging scheme of only 3 pts before and after the jump
			// With the LIDAR getting 500pts/scan it's angle between pts is 0.0127 rad
			// which means the dist. between pts is R*0.0127, which equals 3cm at R = 238cm
			float avgPre, avgPost;
			getAveragePrePost(avgPre, avgPost, i+1, 2); // do 3pt averages before and after
			if(abs(avgPre - avgPost) > SmallJumpDist*0.75){ // filter out random bad pts
				smallJump_.push_back(i);
			}
			else{
				outliers_.push_back(i);
			}//*/
			smallJump_.push_back(i);
		}
	}

	//cout<<"bad points that weren't jumps: "<<bad<<"\n";
	
	/*cout<<"Big jumps at angles: ";
	for(int i=0; i<jump_.size(); i++) {cout<<degrees_[jump_[i]]<<"--"<<getCloserJumpRadius(jump_[i])<<"   ";}
	cout<<"\n";//*/
	
	/*
	cout<<"furn jumps at angles: ";
	for(int i=0; i<smallJump_.size(); i++) {cout<<degrees_[smallJump_[i]]<<" "<<getCloserJumpRadius(smallJump_[i])<<"   ";}
	cout<<"\n";*/
	
	/*
	for(int j=0; j<jump.size(); j++)
		cout<<"at i="<<jump_[j]<<" rad is "<<rad_[jump[j]]<<" xval is: "<<xVal_[jump[j]] <<" yVal_ is: "<<yVal_[jump[j]]<<"  ";
	cout<<"total i="<<rad_.size()<<"\n";
	}*/	
	/*
	for(int j : jump){
		cout<<"prev Pt i="<<j-1<<" angle="<<degrees_[j-1]<<" rad ="<<rad_[j-1]<<"  ";
		cout<<"Pt i="<<j<<" angle="<<degrees_[j]<<" rad ="<<rad_[j]<<"  ";
		cout<<"next Pt i="<<j+1<<" angle="<<degrees_[j+1]<<" rad ="<<rad_[j+1]<<"  ";
		cout<<"next Pt i="<<j+2<<" angle="<<degrees_[j+2]<<" rad ="<<rad_[j+2]<<"  ";
		cout<<"\n";
	}*/
}

void Lidar::findLines(bool pubSegmets){
	lines_.resize(0);
	line tmp;
	vector<float> checkX, checkY;
	
	tmp.addPointEnd(xVal_[0],yVal_[0]);
	tmp.addPointEnd(xVal_[1],yVal_[1]);
	tmp.buildLine();
	for(int p=2; p<rad_.size(); p++){
		if(rad_[p] ==  MAXDIST){
			continue; // skip because it's out of range 
		}
		else if((furnIdxs_.size()!=0 && std::find(furnIdxs_.begin(), furnIdxs_.end(), p) != furnIdxs_.end()) || 
				(outliers_.size()!=0 && std::find(outliers_.begin(), outliers_.end(), p) != outliers_.end())){
			//cout<<"pt "<<p<<" of "<<rad_.size()<<" was in furnIdxs_ \n";
			continue; // skip because it's a piece of furniture or a bad point
		}
		float err = tmp.findDist(xVal_[p], yVal_[p]); // perpendicular dist from pt to line
		float ptDist = pt2PtDist(xVal_[p], yVal_[p], xVal_[p-1], yVal_[p-1]); // dist between this point and the prev point
		// if dist from line model is within PerpTHRESH and distance from pre point < NextPtThresh add to line
		if( err < PerpThresh && ptDist < PrevPointDistThresh){
			tmp.addPointEnd(xVal_[p], yVal_[p]);
			tmp.buildLine();// update line model with new point added 
		}
		else{ // line broken, determine if keeping and start making new line model w/ 2 new pts

			// if line contains > minSegmentPts add to vector of lines and reset tmp line
			if(tmp.numPts() > MinPtsForLine){ // note: one bad pt will break up a line so these segments could be small
				lines_.push_back(tmp);
			}
			else{ // line is too short to keep
				for(int pt=0; pt<tmp.numPts(); pt++){ // save all pts in line to check against models later
					checkX.push_back(tmp.getXPoint(pt));
					checkY.push_back(tmp.getYPoint(pt));
				}
			}

			tmp.clearLine(); // reset the line

			if(p < rad_.size()-1) {
				tmp.addPointEnd(xVal_[p],yVal_[p]); // add the current pt which wasn't added to to tmp
				p++;
				tmp.addPointEnd(xVal_[p],yVal_[p]); // add the next pt
				tmp.buildLine();
			}
			else{
				checkX.push_back(xVal_[p]);
				checkY.push_back(yVal_[p]);
				break; // there weren't 2 pts remaining to make next line
			}
		}
	}
	if(tmp.numPts() > MinPtsForLine) lines_.push_back(tmp); // add the last line being made
	// Adding Back Points:
	int addedback = 0;
	for(int bPts = 0; bPts < checkX.size(); bPts++){
		bool added = false;
		for(int nLine = 0; nLine < lines_.size(); nLine++){
			float bErr = lines_[nLine].findDist(checkX[bPts], checkY[bPts]); // perpendicular dist from pt to line
			// get distances from checkPt to each end of the line segment and if within thresh add to appropriate end of line
			float bPtDist1 = pt2PtDist(checkX[bPts],checkY[bPts],lines_[nLine].getEndPtX1(),lines_[nLine].getEndPtY1()); 
			float bPtDist2 = pt2PtDist(checkX[bPts],checkY[bPts],lines_[nLine].getEndPtX2(),lines_[nLine].getEndPtY2());
			if(bPtDist1 > bPtDist2){ // closer to endpt2
				if( bErr < PerpThresh && bPtDist2 < PrevPointDistThresh){
					lines_[nLine].addPointEnd(checkX[bPts], checkY[bPts]);
					added = true;
					lines_[nLine].buildLine();// update line model with new point added 
					addedback++;
				}
			}
			else{
				if( bErr < PerpThresh && bPtDist1 < PrevPointDistThresh){
					lines_[nLine].addPointStart(checkX[bPts], checkY[bPts]);
					added = true;
					lines_[nLine].buildLine();// update line model with new point added 
					addedback++;
				}
			}
			if(added){break;} // don't check against all other line models if it gets added to one
		} // loop thru line models
	} // loop thru checkpts

	//cout<<lines_.size()<<" raw lines R^2 are: \n";
	for(int lm=0; lm<lines_.size(); lm++){
		if(lines_[lm].makeRSquared() > MinRSquaredSegment){
	//		cout<<lines_[lm].makeRSquared()<<", ";
		}
		else{
			lines_.erase(lines_.begin() + lm);
			lm--;
		}
	}

	// Merging line models
	//if(pubSegmets) nav_->makeLineMarks(lines_, false, true); // publish unmerged lines in rviz

	for(int lm=0; lm<lines_.size(); lm++){
		int nextIdx = (lm+1)%lines_.size();
		if(lines_[lm].canMerge(lines_[nextIdx])){
			//cout<<"can merge lines "<<lm<<" and "<<nextIdx<<"\n";
				lines_[lm].mergeLines( lines_[nextIdx] );
				lines_.erase(lines_.begin() + nextIdx);
				lm--;
		}
	}
	// Make RSquared values and delete any bad lines that managed to stay in there
	//cout<<lines_.size()<<" merged lines R^2 vals are: \n";
	for(int lm=0; lm<lines_.size(); lm++){
		if(lines_[lm].makeRSquared() > MinRSquaredFinal){
			//cout<<lines_[lm].getRSquared()<<", ";
		}
		else{
			lines_.erase(lines_.begin() + lm);
			lm --;
		}
	}
	// sort lines by closest to robot
	std::sort(lines_.begin(),lines_.end(),[](line a, line b) -> bool {return a.getClosestRadius() < b.getClosestRadius();});
	/*cout<<"num after merging lines: "<<lines_.size()<<" with sizes: ";
	for(int i=0; i<lines_.size(); i++){
		cout<<lines_[i].numPts()<<", ";
	}*/
}

void Lidar::findStartLocation(EndPoint endR1, EndPoint endR2, EndPoint endG1, EndPoint endG2){
	EndPoint endG; //lidar location
	float length0 = pt2PtDist(endR1.getX(), endR1.getY(), endR2.getX(), endR2.getY());
	float length1 = endR1.findRad();
	float length2 = endR2.findRad();
	float thetaR1 = endR1.findAngle();
	float thetaR2 = endR2.findAngle();
	float thetaD;
	if(thetaR1 < thetaR2){
		thetaD = thetaR2-thetaR1;
	}
	else{
		thetaD = thetaR2 - thetaR1 + 360;
	}
	float theta3 = asin((length2/length0)*sin(thetaD*3.14159/180));
	float x = endG1.getX() - length1*sin(theta3);
	float y = endG1.getY() + length1*cos(theta3);
	endG.setCart(x, y);
	cout << "x: " << x << endl << "y: " << y << endl;

	float xAng = endG1.getX() - x;
	float yAng = endG1.getY() - y;
	float thetaG = atan2(yAng, xAng)*180/3.14159 - endR1.findAngle();


	cout << xAng << endl << yAng << endl;
	cout << "Robot orientation: " << thetaG << endl;
}

void Lidar::getAveragePrePost(float &pre, float &post, int center, int offset, bool debug){
	// determine averages excluding the jump pt (center + 1)
	if(offset<1){ cout<<"bad call to getAveragePrePost\n"; return;}
	float aPre = 0;
	float aPost = 0;

	if(debug) cout<<"Prev: ";
	for(int a=-offset; a<0; a++){ 
		aPre += rad_[(center+a)%rad_.size()];
		if(debug) cout<<rad_[(center+a)%rad_.size()]<<"  ";
       
	}
	if(debug) cout<<"Center: "<<rad_[center%rad_.size()]<<" ";
	if(debug) cout<<"Post: ";
	for(int a=1; a<(offset+1); a++){ 
		aPost += rad_[(center+a)%rad_.size()];
		if(debug) cout<<rad_[(center+a)%rad_.size()]<<"  ";
       	}
	if(debug) cout<<"\n";
	pre = aPre / (float) offset;
	post = aPost / (float) offset;
}

// determines if the next point is actually meant to loop around
int Lidar::getEndIdx(int s){ 
	if(s > rad_.size()) {
		cout<<"Called getEndIdx with larger than rad.size() idx\n";
		exit(1);
	}
	return s+1 < rad_.size() ? s+1 : 0;
}
void Lidar::removePt(int i){
	rad_.erase(rad_.begin() + i );
	degrees_.erase(degrees_.begin() + i );
	xVal_.erase(xVal_.begin() + i );
	yVal_.erase(yVal_.begin() + i );
}

//float Lidar::pt2PtDist(float x1, float y1, float x2, float y2){ return pow(pow(x2-x1,2) + pow(y2-y1,2) ,0.5); }
// given the start of a jump index return the index of the closer pt in the jump
float Lidar::pt2PtDist(float x1, float y1, float x2, float y2){ return pow(pow(x1-x2,2) + pow(y1-y2,2),0.5);}
int Lidar::getCloserJumpPt(int i){  return rad_[i] < rad_[getEndIdx(i+1)] ? i : getEndIdx(i+1); }
int Lidar::getFurtherJumpPt(int i){ return rad_[i] > rad_[getEndIdx(i+1)] ? i : getEndIdx(i+1); }
float Lidar::getCloserJumpRadius(int i){  return rad_[getCloserJumpPt(i)]; }
float Lidar::getFurtherJumpRadius(int i){ return rad_[getFurtherJumpPt(i)]; }
bool Lidar::jumpAway(int i){ return rad_[i] < rad_[getEndIdx(i)]; }
void Lidar::setNav(Nav *nav){ nav_ = nav; }

int Lidar::modeRoom(){
	std::sort(startRooms_.begin(), startRooms_.end());

	int number = startRooms_[0];
	int mode = number;
	int count = 1;
	int countMode = 1;

	for(int i=1; i<startRooms_.size(); i++) {
	      	if(startRooms_[i] == number){ // count occurrences of the current number
		       ++count;
	      	}
	    	else{ // now this is a different number
		if (count > countMode){
			countMode = count; // mode is the biggest ocurrences
			mode = number;
		}
		count = 1; // reset count for the new number
		number = startRooms_[i];
		}
	}
	return mode;
}

float Lidar::getMaxRSquare(){
	float max = 0;
	for(line ln : lines_){
		if(ln.getRSquared() > max){
			max = ln.getRSquared();
		}
	}
	return max;
}

Lidar::Lidar(){
	defaults();
} // do not use this

void Lidar::defaults(){
	prevOdom_ << -100, -100, 0.4*PI; // start at random pos
	startCount_ = 0;
	localRoom_ = -1;
	started_ = false;
	executing_ = false;
	checkCandle_ = false;
	pauseUpdates_ = false;
	startRooms_.resize(0);
	tickCount_ = 0;
}

Lidar::Lidar(Robot *robRef, Nav* navRef){
	rob_ = robRef;
	nav_ = navRef;
	defaults();
}

Lidar::Lidar(Nav* navRef){
	nav_ = navRef;
	defaults();
}
