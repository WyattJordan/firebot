#include "lidar.h"
Lidar::Lidar(){;} // do not use this
/*
Lidar::Lidar(Robot *robRef){

	prevOdom_ << -100, -100, 0;
	rob_ = robRef;
}*/
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
	vector<float> tdeg, trad, tX, tY; // add to temp until cross is hit
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
	findFurniture(); // determines what is furniture from furnJump_
	nav_->makeFurnMarks(furns_);
	
	classifyRoomFromJumps();
	
/*	ROS_INFO("Starting findLine...");
	findLine(xVal_, yVal_);
	ROS_INFO("Finished findLine...");
	time(&finish);
	cout << "Time of program is " << difftime(finish, start) << " seconds" << endl;*/
//	for(int i = 0; i < xVal_.size(); i++){
//		ROS_INFO(" Testing: X = %f, Y = %f", xVal_[i], yVal_[i]);
//	}

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
	// get closest two big jumps (edges of doorway leaving room 4)
	int closest = -1;
	int nClosest = -2;
	float min = 99999999.9;
	for(int i=0; i<2; i++){
		for(int j : cJumps){
			float thismin = getCloserJumpRadius(j);
			if(thismin < min)
					min = thismin;
					if(i==0) {closest = j;}
					else{nClosest = j;}
		}
		// remove closest and reset min to find next min
		cJumps.erase(std::remove(cJumps.begin(), cJumps.end(), closest), cJumps.end());
		min = 99999999.9;
	}

	// determine which side the door is on from longer range measurements
	int r4long = 0;
	for(float d : rad_) {if(d>160) r4long++;}
	EndPoint ep1, ep2;
	if(r4long > 5){	
		ep1 = nav_->getMapPoint(11);
		ep2 = nav_->getMapPoint(19);
		cout<<"room4 door faces (0,0)\n";
	}
	else{
		ep1 = nav_->getMapPoint(13);
		ep2 = nav_->getMapPoint(18);
		cout<<"room4 door faces top of maze\n";
	}

	// point in the center of the doorway in the global frame determined by the map
	EndPoint gDoorPt((ep1.getX() + ep2.getX()) / 2.0, (ep1.getY() + ep2.getY()) / 2.0);

	// don't know if the first point of the jump or the second has the nearer radius, use the closer points to make the doorway waypoint
	closest = getCloserJumpPt(closest);
	nClosest = getCloserJumpPt(nClosest);
	EndPoint lDoorPt((xVal_[closest] + xVal_[nClosest]) / 2.0, (yVal_[closest] + yVal_[nClosest]) / 2.0);
	
//	localizeFromPt(lDoorPt, gDoorPt);
}

/*void Lidar::localizeFromPt(EndPoint l, EndPoint g){

}*/

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

	if(closeJumps.size() >= 2){
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
	// loop thru furn jumps
	furns_.resize(0);
	furnIdxs_.resize(0);

	cout<<"starting to find furn with "<<furnJump_.size()<<" furnjumps\n";
	cout<<"starting furn jumps at angles: ";
	for(int q=0; q<furnJump_.size(); q++) {cout<<"j:"<<furnJump_[q]<<" at "<<degrees_[furnJump_[q]]<<"--"<<getCloserJumpRadius(furnJump_[q])<<"  ";}
	cout<<"\n";//*/

	vector<int> furnJumpsConfirmed;
	for(int j=0; j<furnJump_.size(); j++){

		// get the two x,y points that are the closer to the 'bot of the two jump points  
		int pt1 = getCloserJumpPt(furnJump_[j]);
		int pt2 = getCloserJumpPt(furnJump_[(j+1)%furnJump_.size()]);
		int step = abs(pt1-pt2);  // this is the number of points hitting the furniture besides the endpoints 
		bool atLoopAround = false;
		int middle = (pt1 + step/2) % rad_.size();
		// check the width tolerance of the two points
		float width = pt2PtDist(xVal_[pt1], yVal_[pt1], xVal_[pt2], yVal_[pt2]); 
		if(!(abs(FurnWidth - width) < FurnWidthTolerance)){ // if the distance between the points is absurd skip this iteration
			//cout<<"deleted due to width constraint width = "<<width<<"\n";
			continue; // skips the rest of the code in this iteration
		}	

		if(step > 200 ){ // if there is a massive number of points hitting the "furniture" it's looping around the 180 spot so swap order
			cout<<"jump "<<furnJump_[j]<<" puts furniture at looping point!! step = "<<step<<" ";
			atLoopAround = true;
			step = rad_.size() - step - 2; // instead of counting the long way around the circle
			cout<<" now step = "<<step<<"\n";
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
		if(step < 7){ // should let you detect a 13cm object 147cm away (tstep = 0.126) or a 5cm object 56cm away
			cout<<"too few of pts on the furniture for jump "<<furnJump_[j]<<"\n";
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
			if(!atLoopAround){
				for(int rem=pt1; rem<=pt2; rem++) furnIdxs_.push_back(rem);
			}
			else{
				for(int rem=pt1; rem%rad_.size()>=pt2; rem--) furnIdxs_.push_back(rem);
			}

			furnJumpsConfirmed.push_back(furnJump_[j]); // add the two jump idxs to the confirmed list
			furnJumpsConfirmed.push_back(furnJump_[(j+1)%furnJump_.size()]);
			j++; // next jump will be the end of this piece of furniture so skip that
		}
		else{
			cout<<"jump "<<furnJump_[j]<<" not counted as furn due to curveHeight\n";
		}
	}

	for(int fj : furnJumpsConfirmed){
		if(std::find(jump_.begin(),jump_.end(),fj) != jump_.end()){ // if the fj is in jump_
			jump_.erase(std::remove(jump_.begin(), jump_.end(), fj), jump_.end()); // erase it
		}
	}

	furnJump_ = furnJumpsConfirmed; // only keep confirmed jumps
	cout<<jump_.size()<<" Big jumps at angles: ";
	for(int r=0; r<jump_.size(); r++) {cout<<degrees_[jump_[r]]<<"--"<<getCloserJumpRadius(jump_[r])<<"   ";}
	cout<<"\n";
	
	cout<<"furn jumps after filtering: ";
	for(int t=0; t<furnJump_.size(); t++) {cout<<degrees_[furnJump_[t]]<<"--"<<getCloserJumpRadius(furnJump_[t])<<"   ";}
	cout<<"\n";//*/
}

void Lidar::findJumps(bool findBig){
	jump_.resize(0);
	furnJump_.resize(0);
	cout<<"finding jumps\n";
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
				furnJump_.push_back(i);
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
					//also erase from furnJump_ if this is reinstated
				}
			}
		}
		else if(diff > FurnJumpDist){ // furn jump has lower tolerance for detecting furniture
			// use a smaller averaging scheme of only 3 pts before and after the jump
			// With the LIDAR getting 500pts/scan it's angle between pts is 0.0127 rad
			// which means the dist. between pts is R*0.0127, which equals 3cm at R = 238cm
			float avgPre, avgPost;
			getAveragePrePost(avgPre, avgPost, i+1, 3); // do 3pt averages before and after
			if(abs(avgPre - avgPost) > FurnJumpDist*0.75){ // filter out random bad pts
				furnJump_.push_back(i);
			}
		}
	}

	/*
	cout<<"Big jumps at angles: ";
	for(int i=0; i<jump_.size(); i++) {cout<<degrees_[jump_[i]]<<" "<<getCloserJumpRadius(jump_[i])<<"   ";}
	cout<<"\n";*/
	
	/*
	cout<<"furn jumps at angles: ";
	for(int i=0; i<furnJump_.size(); i++) {cout<<degrees_[furnJump_[i]]<<" "<<getCloserJumpRadius(furnJump_[i])<<"   ";}
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

void Lidar::removePt(int i){
	rad_.erase(rad_.begin() + i );
	degrees_.erase(degrees_.begin() + i );
	xVal_.erase(xVal_.begin() + i );
	yVal_.erase(yVal_.begin() + i );
}


vector<line> Lidar::findLine(){
	vector <line> myLines;
	line tempLine;
	tempLine.setGood(false);
	int scopeSize = 0;		// keeps track of where the line breaks
	int i = 0;			// iterator
	float distToLine = 0;		// distance from a point to a line
	bool twoPointLine = false; 	// checks to see if the line has only 2 points
	float distToPoint = 0;		// distance from point to point (If to big, create new line)

	//adjustable variables
	//Best Vals
	//pointThreshold:	 .02
	float pointThreshold = 2.5;		//distance between the point and line threshold
	float pointDistThresh = 5.75;		//distance between point to point in a line (if it exceeds this threshold it will make a new line)

	while (i < xVal_.size()) {
		for (i; i < xVal_.size(); i++) {
			if(rad_[i] > MAXDIST){
				break;
			}
			if (i == scopeSize) {	//adding first point to a line
				tempLine.addPointEnd(xVal_[i], yVal_[i]);
			}

			else if (i == scopeSize +1){							//checking the second point
				distToPoint = pt2PtDist(xVal_[i], yVal_[i], xVal_[i-1], yVal_[i-1]);	//checks the distance between the first and second point
				if(distToPoint < pointDistThresh){					//if they are close enough, add second point to line
					tempLine.addPointEnd(xVal_[i], yVal_[i]);
					tempLine.setFloats();
				}
				else{									//otherwise send to 'not line' array
					if(myLines.size() == 0){ // if no lines have been made yet
						tempLine.setFloats();
						tempLine.setGood(false);
						myLines.push_back(tempLine);
						tempLine.clearLine();
					}
					else{ // other lines already exist
						if(myLines[myLines.size() - 1].isGoodLine()){	//checks to see if there needs to be a new group of bad points
							tempLine.setFloats();
							tempLine.setGood(false);
							myLines.push_back(tempLine);
							tempLine.clearLine();
						}
						else{ 
							myLines[myLines.size()-1].mergeLines(tempLine);
							myLines[myLines.size()-1].setFloats();
							tempLine.clearLine();
						}
					}
					scopeSize = i;							//saves where it breaks for the next loop
					distToPoint = 0;
					twoPointLine = true;
					break;
				}
			}

			else if (i < scopeSize + 11) {							//checking the third point
				distToPoint = pt2PtDist(xVal_[i], yVal_[i], xVal_[i-1], yVal_[i-1]);
				distToLine = tempLine.findDist(xVal_[i], yVal_[i]);
				if ((distToLine < pointThreshold)&&(distToPoint < pointDistThresh)) {	//checking the point to the line model
					tempLine.addPointEnd(xVal_[i], yVal_[i]);
					tempLine.setFloats();
					if(i == scopeSize + 10){
						myLines.push_back(tempLine);				//adds the line to a vector of lines
					}
				}
				else {									//the point does not fit the line model
					if (myLines.size() == 0) {
						tempLine.setFloats();
						tempLine.setGood(false);
						myLines.push_back(tempLine);
						tempLine.clearLine();
					}
					else{
						if (myLines[myLines.size()-2].isGoodLine()){
							tempLine.setFloats();
							tempLine.setGood(false);
							myLines.push_back(tempLine);
							tempLine.clearLine();
                                        	}
                                        	else{
                                        		myLines[myLines.size() - 1].mergeLines(tempLine);
                                        		myLines[myLines.size() - 1].setFloats();
						}
					}
					scopeSize = i;							//saves where it breaks for the next loop
					break;
				}
			}

			else {										//goes through points 11 through maxPoint
				myLines[myLines.size()-1].setGood(true);
				distToPoint = pt2PtDist(xVal_[i], yVal_[i], xVal_[i-1], yVal_[i-1]);
				distToLine = myLines[myLines.size()-1].findDist(xVal_[i], yVal_[i]);
				if (distToLine < pointThreshold) {
					if(distToPoint <= pointDistThresh){
						myLines[myLines.size()-1].addPointEnd(xVal_[i], yVal_[i]);
						myLines[myLines.size()-1].setFloats();
					}
					else{
						scopeSize = i;
						break;
					}
				}
				else {
					scopeSize = i;
					break;
				}
			//	cout << "Part 4 done" << endl;
			}
		} // end of loop going thru all xVal_s

		tempLine.clearLine();
	}// end of while loop checking that i is < xVal_.size

	cout << "Lines have been made" << endl;

	for(int j = 0; j<myLines.size(); j++){
		if(j == myLines.size()-1){
			float xEnd2 = myLines[j].getEndPtX2();                                                                                                      
                        float yEnd2 = myLines[j].getEndPtY2();                                                                                                      
                        float xPrev = myLines[j].getXPoint(myLines[j].lineSize()-2);                                                                                
                        float yPrev = myLines[j].getYPoint(myLines[j].lineSize()-2);                                                                                
                        float xEnd1 = myLines[0].getEndPtX1();                                                                                                    
                        float yEnd1 = myLines[0].getEndPtY1();
			float endPt2PrevPt=pt2PtDist(xEnd2, yEnd2, xPrev, yPrev);
                        float endPt2NextPt = pt2PtDist(xEnd2, yEnd2, xEnd1, yEnd1);
                        if(endPt2NextPt < endPt2PrevPt){
                                myLines[0].addPointStart(xEnd2, yEnd2);
				myLines[j].clearPoint(myLines[j].lineSize() - 1);
				myLines[0].setFloats();
				myLines[j].setFloats();
			}

		}
		else{
			float xEnd2 = myLines[j].getEndPtX2();
			float yEnd2 = myLines[j].getEndPtY2();
			float xPrev = myLines[j].getXPoint(myLines[j].lineSize()-2);
			float yPrev = myLines[j].getYPoint(myLines[j].lineSize()-2);
			float xEnd1 = myLines[j+1].getEndPtX1();
			float yEnd1 = myLines[j+1].getEndPtY1();
			float endPt2PrevPt=pt2PtDist(xEnd2, yEnd2, xPrev, yPrev);
			float endPt2NextPt = pt2PtDist(xEnd2, yEnd2, xEnd1, yEnd1);
			if(endPt2NextPt < endPt2PrevPt){
				myLines[j+1].addPointStart(xEnd2, yEnd2);
				myLines[j].clearPoint(myLines[j].lineSize() - 1);
				myLines[j+1].setFloats();
				myLines[j].setFloats();
			}
		}
	}



        int numFake = 0;
        int numReal = 0;
	//cout << endl << endl;

	for(int j = 0; j < myLines.size(); j++){
		if(myLines[j].isGoodLine()==false){
			//check before and after the fake line
			if(j == 0){
				for(int g =0; g < myLines[j].lineSize(); g++){
                         	       distToLine = myLines[myLines.size()-1].findDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
                         	       distToPoint = pt2PtDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g), myLines[myLines.size()-1].getEndPtX2(),myLines[myLines.size()-1].getEndPtY2());
                         	       if((distToLine < pointThreshold)&&(distToPoint < pointDistThresh)){
                         	               //add the point to the line and delete it from the fake line
                         	               myLines[myLines.size()-1].addPointEnd(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
					       myLines[myLines.size()-1].setFloats();
                         	               myLines[j].clearPoint(g);
					       myLines[j].setFloats();
                         	               g--;
                         	       }
                       		}
			}
			else{
			//	cout << "Test 2" << endl;
				for(int g =0; g < myLines[j].lineSize(); g++){
					distToLine = myLines[j-1].findDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
					distToPoint = pt2PtDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g), myLines[j-1].getEndPtX2(),myLines[j-1].getEndPtY2());
					if((distToLine < pointThreshold)&&(distToPoint < pointDistThresh)){
						//add the point to the line and delete it from the fake line
						myLines[j-1].addPointEnd(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
						myLines[j-1].setFloats();
						myLines[j].clearPoint(g);
						myLines[j].setFloats();
						g--;
					}
				}
			}
			if(myLines[j].lineSize() == 0){
				myLines[j].clearLine();
				myLines.erase(myLines.begin() + j);
				j--;
			}
		}
	}
	for(int j = 0; j < myLines.size(); j++){
		if(myLines[j].isGoodLine() == false){
			myLines[j].reverseLine();
			if(j == myLines.size()-1){
			//	cout << "Test 3" << endl;
				for(int g = 0; g < myLines[j].lineSize(); g++){
                                	distToLine = myLines[0].findDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
                                	distToPoint = pt2PtDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g), myLines[0].getEndPtX1(), myLines[0].getEndPtY1());
                                	if((distToLine < pointThreshold)&&(distToPoint < pointDistThresh)){
                                        	myLines[0].addPointStart(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
						myLines[0].setFloats();
                                        	myLines[j].clearPoint(g);
						myLines[j].setFloats();
						g--;
                                	}
                        	}
			}
			else{
			//	cout << "Test 4" << endl;
				for(int g = 0; g < myLines[j].lineSize(); g++){
					distToLine = myLines[j+1].findDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
					distToPoint = pt2PtDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g), myLines[j+1].getEndPtX1(), myLines[j+1].getEndPtY1());
					if((distToLine < pointThreshold)&&(distToPoint < pointDistThresh)){
						myLines[j+1].addPointStart(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
						myLines[j+1].setFloats();
						myLines[j].clearPoint(g);
						myLines[j].setFloats();
						g--;
					}
				}
			}
			myLines[j].reverseLine();
			if(myLines[j].lineSize() == 0){
				myLines[j].clearLine();
				myLines.erase(myLines.begin() + j);
				j--;
			}
		}
	}
	int numMerge = 0;
	for (int g = 0; g < myLines.size(); g++){
                if(myLines[g].isGoodLine()){
                        for (int h = 0; h < g; h++){
                                if (( canMerge(myLines[g], myLines[h]) == true)&&(myLines[h].isGoodLine())){
				//	cout << "Lines merged" << endl;
					if((g == myLines.size()-1)&&(h == 0)){
						myLines[g].mergeLines(myLines[h]);
						myLines.erase(myLines.begin());
						g--;
						h--;
					}
					else{
                                        	myLines[h].mergeLines(myLines[g]);
				//		cout << "new slope: " << myLines[h].getSlope() << endl;
                                        	myLines.erase(myLines.begin() + g);
                                        	g--;
						h--;
						numMerge++;
					}
                                }
                        }
                }
	}
	cout << numMerge << endl;
        

//	cout << endl << "real lines after fake lines are added" << endl;
//	Output results to console for debugging
///*
	numFake = 0;
	numReal = 0;
	for (int j = 0; j < myLines.size(); j++) {
		if(myLines[j].isCandle()){
			cout << endl << "This is a candle" << endl;
		}
		else if(myLines[j].isFurniture()){
			cout << endl << "This is furniture" << endl;
		}
		if(myLines[j].isGoodLine()){
			numReal++;
                	float ang1, ang2, rad1, rad2;
                	ang1 = myLines[j].endPAngle(1);
                	ang2 = myLines[j].endPAngle(2);
                	rad1 = myLines[j].endPRad(1);
                	rad2 = myLines[j].endPRad(2);
                	cout << endl << "Line: " << numReal << " size: " << myLines[j].lineSize();
                	cout <<" Slope: " << myLines[j].getSlope() << " Intercept: " << myLines[j].getIntercept();
                	cout << " Endpoints: (" << myLines[j].getEndPtX1() << ", " << myLines[j].getEndPtY1();
                	cout << ") Angle: " << ang1;
                	cout << " Distance: " << rad1 << endl;
                	cout << "          (" << myLines[j].getEndPtX2() << ", " << myLines[j].getEndPtY2();
                	cout << ") Angle:" <<  ang2;
                	cout << " Distance: " << rad2 << endl;
			cout << "Line length: " << myLines[j].getLength() << endl; 
		//	myLines[j].printLine();
		}
		else{
			numFake++;
			cout << endl << "Fake line num " << numFake << " has size: " << myLines[j].lineSize() << 
				" getLength returns "<< myLines[j].getLength() << endl;
		}
        }


	cout << numMerge << endl;
//	*/
//	cout<<"Lines made\n";
	float distToEnd = 0;
	return myLines;
};

//it. canMerge
bool Lidar::canMerge(line a, line b){
	float distThresh = 5.75;
	float myDist;
	float tempDist;
	float slopeMult;

	
	myDist = pt2PtDist(a.getEndPtX1(), a.getEndPtY1(), b.getEndPtX1(), b.getEndPtY1());
	//cout << myDist << endl;

        tempDist = pt2PtDist(a.getEndPtX2(), a.getEndPtY2(), b.getEndPtX2(), b.getEndPtY2());
        if (tempDist < myDist) {myDist = tempDist;}
	//cout << tempDist << endl;

        tempDist = pt2PtDist(a.getEndPtX1(), a.getEndPtY1(), b.getEndPtX2(), b.getEndPtY2());
        if (tempDist < myDist) {myDist = tempDist;}
	//cout << tempDist << endl;

        tempDist = pt2PtDist(a.getEndPtX2(), a.getEndPtY2(), b.getEndPtX1(), b.getEndPtY1());
        if (tempDist < myDist) {myDist = tempDist;}
	//cout << tempDist << endl;

	if(myDist < distThresh){
		slopeMult = a.getSlope()*b.getSlope();

		//cout << "Slopes: " << endl << a.getSlope() << endl << b.getSlope() << endl << "Slopes multiplied: " << slopeMult << endl << endl;
		if((a.getSlope()*b.getSlope() > -5) && (a.getSlope()*b.getSlope() < 3))
			return false;
	
		else
			return true;
	}
	else
		return false;
}
void Lidar::findRoom(){
	int rm1 = 0;
	int rm2 = 0;
	int rm3 = 0;
	int rm4 = 0;
	float ratMult = 0.0016949;
	float rat23 = 0;
	float rat4 = 0;
	float maxDist = 0;
	int room23Count = 0;
	int room4Short = 0;
	int room4Long = 0;
	bool room1a = false;
	bool room1b = false;
	bool room2 = false;
	bool room3 = false;
	bool room4 = false;
	bool room4l = false;
	bool room4s = false;
	bool shouldBreak = false;
	vector <float> myLength;
	//vector <line> lineVec = findLine(xVal_,yVal_);

	float room4Vshort = 0;
	for(int i=0; i < rad_.size(); i++){
		if(rad_[i] < 45){ // max possible distance in Room4 is 65 cm
			room4Vshort++;
		}
		if(rad_[i] < 65){ // max possible distance in Room4 is 65 cm
			room4Short++;
		}
		else if(rad_[i] > 170){
			room4Long++;
		}
	}
	
	rat4 = room4Short*ratMult;

	float percentShort = (float) room4Short / (float) rad_.size();
	float percentVshort = (float) room4Vshort / (float) rad_.size();
	cout<<"Room 4 \% below 45 = "<<percentVshort<<" below 65cm = "<<percentShort<<" and "<< room4Long <<" long lines > 170cm\n";
	if(percentShort > 0.70){
		cout<<"Should be in room 4!!!\n";
		room4 = true;
		if(room4Long > 3){
			room4l = true;
		}
		else{
			room4s = true;
		}
	}
	
	/*
	for(int i=0; i<rad_.size(); i++){
		if(rad_[i] < 70){
			room23Count++;
		}
	}	
	rat23 = room23Count*ratMult;

	if(rat23 > .7){
		for(int i = 0; i < lineVec.size(); i++){
				myLength.push_back(lineVec[i].getLength());
		}

		for(int i=0; i<myLength.size(); i++){
			if(lineVec[i].findDist(0,0) < 65){
				if((myLength[i] < 135) && (myLength[i] > 124)){
					rm3++;
				}
				if((myLength[i] < 31) && (myLength[i] > 23)){
					rm3++;
				}
			}			
		}
		cout << endl << endl;
		if(rm3 > 0){
			room3 = true;
		}
		else {
			room2 = true;
		}
	}
		/*	
			EndPoint temp;
			temp.setCart(lineVec[i].getEndPtX1(), lineVec[i].getEndPtY1());
			EndPoint temp2;
			temp2.setCart(lineVec[i].getEndPtX2(), lineVec[i].getEndPtY2());
			EndPoint temp3;
			EndPoint temp4;
			temp3.setCart(.72, .46);
			temp4.setCart(myLength[i] + .72, .46);
			findStartLocation(temp, temp2, temp3, temp4);
		*/
	/*
	if((!room4l) && (!room4s) && (!room2) && (!room3)){
		for(int i=0; i < rad_.size(); i++){
			if((rad_[i] < 151) && (rad_[i] > 100)){
				rm1++;
			}
		}
		if(rm1 > 47) {room1b = true;}
		else {room1a = true;}
	}
	bool line1 = false;
	bool line2 = false;
	if(room4l){
		cout << endl << endl << "This is room 4 toward the maze" << endl << endl;
		for(int i = 0; i < lineVec.size(); i++){
			line1 = line2 = false;
			if((lineVec[i].getLength() > 20) && (lineVec[i].getLength() < 34)){
				cout << i << " Gotcha!" << endl;
				line1 = true;
			}
			else if((lineVec[i].getLength() > 64) && (lineVec[i].getLength() < 78)){
				cout << i << " Gotcha again!" << endl;
				line2 = true;
			}
			if(line1){
				EndPoint temp1;
				EndPoint temp2;
				EndPoint temp3;
				EndPoint temp4;
				temp1.setCart(lineVec[i].getEndPtX1(), lineVec[i].getEndPtY1());
				temp2.setCart(lineVec[i].getEndPtX2(), lineVec[i].getEndPtY2());
				temp3.setCart(164, 141);//point 19 164, 141
				temp4.setCart(188, 141);//point 14
				findStartLocation(temp1, temp2, temp3, temp4);
			}
			else if(line2){
				EndPoint temp1;
				EndPoint temp2;
				EndPoint temp3;
				EndPoint temp4;
				temp1.setCart(lineVec[i].getEndPtX1(), lineVec[i].getEndPtY1());
				temp2.setCart(lineVec[i].getEndPtX2(), lineVec[i].getEndPtY2());
				temp3.setCart(188, 192);//point 13
				temp4.setCart(118, 192);//point 12
				findStartLocation(temp1, temp2, temp3, temp4);
			}
		}
			
	}
	else if(room4s){
		cout << endl << endl << "This is room 4 toward the wall" << endl << endl;
		for(int i = 0; i < lineVec.size(); i++){
			if((lineVec[i].getLength() > 20) && (lineVec[i].getLength() < 34)){
				cout << "Gotcha!" << endl;
				line1 = true;
			}
			else if((lineVec[i].getLength() > 64) && (lineVec[i].getLength() < 78)){
				cout << "Gotcha again!" << endl;
				line2 = true;
			}
		}
	}
	else if(room2){
		cout << endl << endl << "This is room 2" << endl << endl;
		for(int i = 0; i < lineVec.size(); i++){
                        if((lineVec[i].getLength() > 95) && (lineVec[i].getLength() < 108)){
                                cout << "Gotcha!" << endl;
                        }
                        else if((lineVec[i].getLength() > 50) && (lineVec[i].getLength() < 64)){
                                cout << "Gotcha again!" << endl;
                        }
                }
	}
	else if(room3){
		cout << endl << endl << "This is room 3" << endl << endl;
		for(int i = 0; i < lineVec.size(); i++){
                        if((lineVec[i].getLength() > 120) && (lineVec[i].getLength() < 137)){
                                cout << "Gotcha!" << endl;
                        }
                        else if((lineVec[i].getLength() > 23) && (lineVec[i].getLength() < 31)){
                                cout << "Gotcha again!" << endl;
                        }
                }
	}
	else if(room1a){
		cout << endl << endl << "This is room 1 maze side" << endl << endl;
	}
	else if(room1b){
		cout << endl << endl << "This is room 1 wall side" << endl << endl;
	}
	else{
		cout << endl << endl << "Did not find the starting location" << endl << endl;
	}
	cout << room1a << " " << room1b << " " << room2 << " " << room3 << " " << room4l << " " << room4s << endl;
//	cout << rat4 << endl << rat23 << endl;
//	cout << "room 1 counter: " << rm1 << endl;
//	*/
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
// given the start of a jump index return the index of the closer pt in the jump
int Lidar::getCloserJumpPt(int i){
	return rad_[i] < rad_[getEndIdx(i+1)] ? i : getEndIdx(i+1);
}
int Lidar::getFurtherJumpPt(int i){
	return rad_[i] > rad_[getEndIdx(i+1)] ? i : getEndIdx(i+1);
}
float Lidar::getCloserJumpRadius(int i){
	return rad_[getCloserJumpPt(i)];
}
float Lidar::getFurtherJumpRadius(int i){
	return rad_[getFurtherJumpPt(i)];
}
bool Lidar::jumpAway(int i){
	return rad_[i] < rad_[getEndIdx(i)];
}

void Lidar::removePt(int i){
	rad_.erase(rad_.begin() + i );
	degrees_.erase(degrees_.begin() + i );
	xVal_.erase(xVal_.begin() + i );
	yVal_.erase(yVal_.begin() + i );
}
float Lidar::pt2PtDist(float x1, float y1, float x2, float y2){
	return pow(pow(x2-x1,2) + pow(y2-y1,2) ,0.5);
}

void Lidar::setNav(Nav *nav){
	nav_ = nav;
}
