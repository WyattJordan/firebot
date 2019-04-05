#include "lidar.h"
Lidar::Lidar(){;} // do not use this

Lidar::Lidar(Robot *robRef, Nav* navRef){

	prevOdom_ << -100, -100, 0;
	rob_ = robRef;
	nav_ = navRef;
}

Lidar::Lidar(Nav* navRef){
	prevOdom_ << -100, -100, 0;
	nav_ = navRef;
}

void Lidar::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
	/* bool updatePosition = false;
	Vector3f currentPos = rob_->getOdomWorldLoc(); // this is an undefined ref for some reason...
	//Vector3f currentPos;
	if(! (prevOdom_(0) == -100 && prevOdom_(1) == -100)) // if not the first run (default odom loc) 
	{
		// if the angle has changed less than 5deg between the two positions
		if(abs(prevOdom_(2) - currentPos(2))*180/PI < 5) updatePosition = true;
	
	}*/ //stuff for position updates

	cout<<"\nentered callback...\n";
	int num = scan->scan_time / scan->time_increment;
	time_t start, finish;
	degrees_.resize(0);
	rad_.resize(0);
	xVal_.resize(0); 
	yVal_.resize(0);
	time(&start);

	// formats the data so that angle increases from 0 to 360
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
	}

	findJumps(true);
	cout<<"num jumps = "<<jump_.size()<<"\n";
	findFurniture(); // determines what is furniture from smallJump_
	nav_->makeFurnMarks(furns_);
	
//	classifyRoomFromJumps();
	findLines();
	nav_->makeLineMarks(lines_);

	// publish transformation from global to laser_frame
/*	static tf::TransformBroadcaster br;	
	tf::Transform trans;
	trans.setOrigin(tf::Vector3(currentPos(0), currentPos(1), 0));
	tf::Quaternion q;
	q.setRPY(0,0,currentPos(2));
	trans.setRotation(q);
	// determine the frame laser_frame in the global frame
	br.sendTransform(tf::StampedTransform(trans, ros::Time::now(),LASERFRAME, GLOBALFRAME));
	cout<<"sent tf frame via broacaster\n";
	prevOdom_ = currentPos;*/
}

void Lidar::room4Localization(vector<int> cJumps){
	if(cJumps.size()!=2) cout<<"room4 localization with other than 2 door jumps\n";
	// determine which side the door is on from longer range measurements
	int r4long = 0;
	for(float d : rad_) {if(d>160) r4long++;}
	EndPoint ep1, ep2;
	if(r4long > 5){	
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
	int closest = getCloserJumpPt(cJumps[0]);
	int nClosest = getCloserJumpPt(cJumps[1]);
	EndPoint lDoorPt((xVal_[closest] + xVal_[nClosest]) / 2.0, (yVal_[closest] + yVal_[nClosest]) / 2.0);

	int wall1 = -1;
	int wall2 = -1;
	float angleToDoor = lDoorPt.findAngle();
	// find two walls with furthest polar angle to doorway opening
	for(int w=0; w<2; w++){
		int max = 0;
		for(int i=0; i<lines_.size(); i++){
			float diff = abs(lines_[i].getCenterTheta() - angleToDoor); // all in lidar frame
			if(diff > max){
				if(w==0){ // if finding the first wall
					max = diff;
					wall1 = i;
				}
				else if(i !=wall1){ // if finding the 2nd wall and this one isn't the first
					max = diff;
					wall2 = i;
				}
			}
		}
	}

	// now eliminate wall with closest endpoint to doorway (should be smaller wall)
	float xDoor = lDoorPt.getX();
	float yDoor = lDoorPt.getY();
	
	float wall1Dist = std::min(pt2PtDist(xDoor,yDoor, lines_[wall1].getEndPtX1(),lines_[wall1].getEndPtY1())  , 
				pt2PtDist(xDoor,yDoor, lines_[wall1].getEndPtX2(),lines_[wall1].getEndPtY2()));
	float wall2Dist = std::min(pt2PtDist(xDoor,yDoor, lines_[wall2].getEndPtX1(),lines_[wall2].getEndPtY1())  , 
				pt2PtDist(xDoor,yDoor, lines_[wall2].getEndPtX2(),lines_[wall2].getEndPtY2()));

	int wall = wall1Dist > wall2Dist ? wall1 : wall2; // use the wall with further of closer endpoints to doorway

	cout<<"using wall :"<<wall<<" which is has center "<<lines_[wall].getCenterRadius()<<" cm away at angle "<<lines_[wall].getCenterTheta()<<"\n";
	
	//	localizeFromPt(lDoorPt, gDoorPt);
}

// to detect furniture look at dist between endpoint of first jump and start of second jump
void Lidar::classifyRoomFromJumps(){
	cout<<"finding room from jumps... ";

	// Check room4 first, if there are two close big jumps (the doorway) it's room 4
	float room4NearLimit = 70; 
	vector<int> closeJumps;
	for(int j : jump_){
		if(rad_[j] < room4NearLimit || rad_[getEndIdx(j+1)] < room4NearLimit){
			closeJumps.push_back(j);
		}
	}
	cout<<"closeJumpCount = "<<closeJumps.size()<<"\n";

	if(closeJumps.size() == 2){
		cout<<"localizing for room4\n";
		//room4Localization(closeJumps);
	}
	else if(jump_.size() > 1){ // rooms 2 and 3 only have one jump but room 1 has at least 2
		cout<<"localizing for room1\n";
		//room1Localization();
	}
	else{
		// TODO determine if it's room2 or room3 somehow...
		cout<<"localizing for room 2 or 3\n";
	}

}


void Lidar::room1Localization(){
}

// Furniture will have ~13cm difference between endpoints (within FurnWidthTolerance)
// And will have a point with a lower polar Radius between the two points (by at least FurnDepthTolerance)
void Lidar::findFurniture(){
	furns_.resize(0);
	furnIdxs_.resize(0);
	cout<<"starting to find furn with "<<smallJump_.size()<<" furnjumps\n";
	/*cout<<"starting furn jumps at angles: ";
	for(int q=0; q<smallJump_.size(); q++) {cout<<"j:"<<smallJump_[q]<<" at "<<degrees_[smallJump_[q]]<<"--"<<getCloserJumpRadius(smallJump_[q])<<"  ";}
	cout<<"\n";//*/

	vector<int> furnJumpsConfirmed;
	for(int j=0; j<smallJump_.size(); j++){

		// get the two x,y points that are the closer to the 'bot of the two jump points  
		int pt1 = getCloserJumpPt(smallJump_[j]);
		int pt2 = getCloserJumpPt(smallJump_[(j+1)%smallJump_.size()]);
		int step = abs(pt1-pt2);  // this is the number of points hitting the furniture besides the endpoints 
		bool atLoopAround = false;
		int middle = (pt1 + step/2) % rad_.size();
		// check the width tolerance of the two points
		float width = pt2PtDist(xVal_[pt1], yVal_[pt1], xVal_[pt2], yVal_[pt2]); 
		if(!(abs(FurnWidth - width) < FurnWidthTolerance)){ // if the distance between the points is larger than furniture size
			//cout<<"deleted due to width constraint width = "<<width<<"\n";
			continue; // skips the rest of the code in this iteration
		}	

		if(step > 200 ){ // if there is a massive number of points hitting the "furniture" it's looping around the 180 spot so swap order
			atLoopAround = true;
			step = rad_.size() - step - 2; // instead of counting the long way around the circle
			int tmp = pt1; // swap pts because we want pt1 to start off the furniture jumping onto it
			pt1 = pt2;
			pt2 = tmp;
			middle = (pt1 - step/2);
			middle = middle<0 ? middle+rad_.size() : middle;
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

		// check that the midpoint is closer to the robot to confirm it's furniture
		float avgOutsideRad = (rad_[pt1] + rad_[pt2]) / 2.0;
		float curveHeight = abs(avgOutsideRad - rad_[middle]);

		if( curveHeight > FurnDistTolerance && curveHeight < 10){ // inner should be at least certain amount closer
			// Endpoint x and y determined by midpoint of furniture (should be lowest radius)
			float x = POLAR2XCART(rad_[middle]+FurnWidth/2.0, degrees_[middle]); 
			float y = POLAR2YCART(rad_[middle]+FurnWidth/2.0, degrees_[middle]);
			EndPoint f(x,y);
			furns_.push_back(f);
			// add the points between pt1 and p2 inclusive to the furnIdxs_
			if(!atLoopAround){ for(int rem=pt1; rem<=pt2; rem++) furnIdxs_.push_back(rem); }
			else{ for(int rem=pt1; rem%rad_.size()>=pt2; rem--) furnIdxs_.push_back(rem); }

			furnJumpsConfirmed.push_back(smallJump_[j]); // add the two jump idxs to the confirmed list
			furnJumpsConfirmed.push_back(smallJump_[(j+1)%smallJump_.size()]);
			j++; // next jump will be the end of this piece of furniture so skip that
		}
		else{
			cout<<"jump "<<smallJump_[j]<<" not counted as furn due to curveHeight\n";
		}
	}

	for(int fj : furnJumpsConfirmed){
		if(std::find(jump_.begin(),jump_.end(),fj) != jump_.end()){ // if the fj is in jump_
			jump_.erase(std::remove(jump_.begin(), jump_.end(), fj), jump_.end()); // erase it
		}
	}

	// save_here = furnJumpsConfirmed; // if you want to save the furn jumps use this line, don't overwrite smallJump_ tho
	cout<<jump_.size()<<" Big jumps at angles: ";
	for(int r=0; r<jump_.size(); r++) {cout<<degrees_[jump_[r]]<<"--"<<getCloserJumpRadius(jump_[r])<<"   ";}
	cout<<"\n";
	
	cout<<"furn jumps after filtering: ";
	for(int t=0; t<furnJumpsConfirmed.size(); t++) {cout<<degrees_[furnJumpsConfirmed[t]]<<"--"<<getCloserJumpRadius(furnJumpsConfirmed[t])<<"   ";}
	cout<<"\n";//*/
}

void Lidar::findJumps(bool findBig){
	jump_.resize(0);
	smallJump_.resize(0);
	for(int i=0; i<rad_.size(); i++){
		float diff = abs(rad_[i] - rad_[(i+1)%rad_.size()]);
		if(diff > DoorJumpDist && findBig){ 
			float avgPre, avgPost; // new method using function
			getAveragePrePost(avgPre,avgPost,i+1,5); // this omits idx+1 because that could be the nasty pt

			if(abs(avgPre - avgPost) > DoorJumpDist*0.75){
				/* // debugging for checking what becomes a line
				cout<<"Pre: ";
				for(int a=-4; a<1; a++){ cout<<rad_[(i+a)%rad_.size()]<<"  "; }
				cout<<" Center: "<<rad_[i+1]<<"   Post: ";
				for(int a=2; a<7; a++){ cout<<rad_[(i+a)%rad_.size()]<<"  "; }
				cout<<"AvgPre = "<<avgPre<<" AvgPost = "<<avgPost<<"\n";
				*/
				jump_.push_back(i);
				smallJump_.push_back(i);
				// if there are two jumps right next to eachother delete both (caused by bad pt)
				// nearness can behave oddly since the LIDAR filters out points above 180cm
				if(jump_.size()>1 && abs(jump_[jump_.size()-2] - jump_[jump_.size()-1]) == 1){
					/*cout<<"Two jumps next to eachother at angles: "<<degrees_[i]<<" and "<<degrees_[i-1]<<"\n";
					float d1, d2;
					cout<<"Previous jump: \n";
					getAveragePrePost(d1,d2, i, 5, 1); 
					cout<<"Current jump: \n";
					getAveragePrePost(d1,d2, i+1, 5,1); */
					//jump_.erase(jump_.begin() + jump_.size()-1);
					//jump_.erase(jump_.begin() + jump_.size()-1);
					//also erase from smallJump_ if this is reinstated
				}
			}
		}
		else if(diff > SmallJumpDist){ // furn jump has lower tolerance for detecting furniture
			// use a smaller averaging scheme of only 3 pts before and after the jump
			// With the LIDAR getting 500pts/scan it's angle between pts is 0.0127 rad
			// which means the dist. between pts is R*0.0127, which equals 3cm at R = 238cm
			float avgPre, avgPost;
			getAveragePrePost(avgPre, avgPost, i+1, 3); // do 3pt averages before and after
			if(abs(avgPre - avgPost) > SmallJumpDist*0.75){ // filter out random bad pts
				smallJump_.push_back(i);
			}
		}
	}

	/*
	cout<<"Big jumps at angles: ";
	for(int i=0; i<jump_.size(); i++) {cout<<degrees_[jump_[i]]<<" "<<getCloserJumpRadius(jump_[i])<<"   ";}
	cout<<"\n";*/
	
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

void Lidar::findLines(){
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
		else if(furnIdxs_.size()!=0 && std::find(furnIdxs_.begin(), furnIdxs_.end(), p) != furnIdxs_.end()){
			continue; // skip because it's a piece of furniture
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
				tmp.addPointEnd(xVal_[++p],yVal_[++p]); // add the next pt
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

	cout<<"num raw lines found = "<<lines_.size()<<"\n";
	nav_->makeLineMarks(lines_);

	for(int j = 0; j < lines_.size(); j++){
	cout<<"X,Y to endpoint 1: " << lines_[j].getEndPtX1() << ", " << lines_[j].getEndPtY1()<<endl;
	cout<<"X,Y to endpoint 2: " << lines_[j].getEndPtX2() << ", " << lines_[j].getEndPtY2()<<endl;
	cout<<"Points in line: " << lines_[j].numPts() << endl;
	}
	cout<<"checkx = "<<checkX.size()<<" checkY = "<<checkY.size()<<"\n";

	// Adding Back Points:
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
				}
			}
			else{
				if( bErr < PerpThresh && bPtDist1 < PrevPointDistThresh){
					lines_[nLine].addPointStart(checkX[bPts], checkY[bPts]);
					added = true;
					lines_[nLine].buildLine();// update line model with new point added 
				}
			}
			if(added){break;} // don't check against all other line models if it gets added to one
		} // loop thru line models
	} // loop thru checkpts

	// Merging line models
	for(int lm=0; lm<lines_.size(); lm++){
		if(lines_[lm].canMerge(lines[(lm+1)%lines_.size()]){
				lines_[lm].mergeLines((lm+1)%lines_.size());
				lines_.erase(lines_.begin() + (lm+1)%lines_.size());
				lm--;
		}
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

float Lidar::pt2PtDist(float x1, float y1, float x2, float y2){ return pow(pow(x2-x1,2) + pow(y2-y1,2) ,0.5); }
// given the start of a jump index return the index of the closer pt in the jump
int Lidar::getCloserJumpPt(int i){  return rad_[i] < rad_[getEndIdx(i+1)] ? i : getEndIdx(i+1); }
int Lidar::getFurtherJumpPt(int i){ return rad_[i] > rad_[getEndIdx(i+1)] ? i : getEndIdx(i+1); }
float Lidar::getCloserJumpRadius(int i){  return rad_[getCloserJumpPt(i)]; }
float Lidar::getFurtherJumpRadius(int i){ return rad_[getFurtherJumpPt(i)]; }
bool Lidar::jumpAway(int i){ return rad_[i] < rad_[getEndIdx(i)]; }
void Lidar::setNav(Nav *nav){ nav_ = nav; }
