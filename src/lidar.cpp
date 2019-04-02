#include "lidar.h"
Lidar::Lidar(){;} // do not use this
/*
Lidar::Lidar(Robot *robRef){

	prevOdom_ << -100, -100, 0;
	rob_ = robRef;
}*/

float Lidar::pt2PtDist(float x1, float y1, float x2, float y2){
	return pow(pow(x2-x1,2) + pow(y2-y1,2) ,0.5);
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
	degrees.resize(0);
	rad.resize(0);
	xVal.resize(0); 
	yVal.resize(0);
	time(&start);
	int crossed = -1;
	float prevAng = 0;

	// formats the data so that angle increases from 0 to 360
	for(int i = 0; i < num; i++) { 
		float radius = scan->ranges[i];
		if ((isinf(radius) == 0)/*&&(radius < 180)*/&&(radius > 18)){ // check if acceptable range measurement
			//if(radius > 180)radius == 180;
			float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
			float ang = degree>0 ? degree : degree + 360;
			if(ang<prevAng && crossed == -1){
				crossed = i;
			}
			/*if(crossed !=-1){ // once the crossover is hit start inserting from beginning
				degrees.insert(degrees.begin() + (i-crossed), ang);
				rad.insert(        rad.begin() + (i-crossed), radius);
				xVal.insert(      xVal.begin() + (i-crossed), POLAR2XCART(radius, ang));
				yVal.insert(      yVal.begin() + (i-crossed), POLAR2XCART(radius, ang));
			}*/
			//else{
				degrees.push_back(ang);
				rad.push_back(radius);
				xVal.push_back(POLAR2XCART(radius, ang));
				yVal.push_back(POLAR2YCART(radius, ang));
			//}
			prevAng = ang;
		}
		else if(crossed != -1){// when range measurement is skipped
			crossed++;         // must adjust the crossed point
		}
	}

	findJumps();
	cout<<"num jumps = "<<jump_.size()<<"\n";
	findRoomFromJumps();
	
/*	ROS_INFO("Starting findLine...");
	findLine(xVal, yVal);
	ROS_INFO("Finished findLine...");
	time(&finish);
	cout << "Time of program is " << difftime(finish, start) << " seconds" << endl;*/
//	for(int i = 0; i < xVal.size(); i++){
//		ROS_INFO(" Testing: X = %f, Y = %f", xVal[i], yVal[i]);
//	}

	// publish transformation from global to laser_frame
/*	static tf::TransformBroadcaster br;	
	tf::Transform trans;
	trans.setOrigin(tf::Vector3(currentPos(0), currentPos(1), 0));
	tf::Quaternion q;
	q.setRPY(0,0,currentPos(2));
	trans.setRotation(q);
	// determine the frame laser_frame in the global frame
	br.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "laser_frame", "global"));
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
			float thismin = std::min(rad[j],rad[getEndIdx(j+1)]);
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
	for(float d : rad) {if(d>160) r4long++;}
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
	closest = std::min(rad[closest],rad[getEndIdx(closest)])   == rad[closest]  ? closest  : getEndIdx(closest);
	nClosest= std::min(rad[nClosest],rad[getEndIdx(nClosest)]) == rad[nClosest] ? nClosest : getEndIdx(nClosest);
	EndPoint lDoorPt((xVal[closest] + xVal[nClosest]) / 2.0, (yVal[closest] + yVal[nClosest]) / 2.0);
	
	localizeFromPt(lDoorPt, gDoorPt);
}

void Lidar::localizeFromPt(EndPoint l, EndPoint g){

}

// to detect furniture look at dist between endpoint of first jump and start of second jump
void Lidar::findRoomFromJumps(){
	cout<<"finding room from jumps\n";

	// Check room4 first, if there are two close big jumps (the doorway) it's room 4
	float room4NearLimit = 70; 
	vector<int> closeJumps;
	for(int j : jump_){
		if(rad[j] < room4NearLimit || rad[getEndIdx(j+1)] < room4NearLimit){
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

void Lidar::getAveragePrePost(float &pre, float &post, int center, int offset){
	// determine averages excluding the jump pt (center + 1)
	if(offset<1){ cout<<"bad call to getAveragePrePost\n"; return;}
	float aPre = 0;
	float aPost = 0;

	for(int a=-(offset-1); a<1; a++){ aPre += rad[(center+a)%rad.size()]; }
	for(int a=2; a<(offset+2); a++){ aPost += rad[(center+a)%rad.size()]; }
	pre = aPre / (float) offset;
	post = aPost / (float) offset;
}

void Lidar::findJumps(){
	jump_.resize(0);
	cout<<"finding jumps\n";
	for(int i=0; i<rad.size(); i++){
		float diff = abs(rad[i] - rad[(i+1)%rad.size()]);
		if(diff > DoorJumpDist){ 
			float avgPre, avgPost; // new method using function
			getAveragePrePost(avgPre,avgPost,i,5); // this omits idx+1 because that could be the nasty pt

			if(abs(avgPre - avgPost) > doorJump*0.75){
				/* // debugging for checking what becomes a line
				cout<<"Pre: ";
				for(int a=-4; a<1; a++){ cout<<rad[(i+a)%rad.size()]<<"  "; }
				cout<<" Center: "<<rad[i+1]<<"   Post: ";
				for(int a=2; a<7; a++){ cout<<rad[(i+a)%rad.size()]<<"  "; }
				cout<<"AvgPre = "<<avgPre<<" AvgPost = "<<avgPost<<"\n";
				*/
				jump_.push_back(i);
				furnJump.push_back(i);
				// if there are two jumps right next to eachother delete both (caused by bad pt)
				// nearness can behave oddly since the LIDAR filters out points above 180cm
				if(jump_.size()>1 && abs(jump_[jump_.size()-2] - jump_[jump_.size()-1]) == 1){
					cout<<"Pre: ";
					for(int a=-4; a<1; a++){ cout<<rad[(i+a)%rad.size()]<<"  "; }
					cout<<" Center: "<<rad[i+1]<<"   Post: ";
					for(int a=2; a<7; a++){ cout<<rad[(i+a)%rad.size()]<<"  "; }
					cout<<"AvgPre = "<<avgPre<<" AvgPost = "<<avgPost<<"\n";
					cout<<"Two jumps next to eachother at angles: "<<degrees[i]<<" and "<<degrees[i-1]<<"\n";
					//jump_.erase(jump_.begin() + jump_.size()-1);
					//jump_.erase(jump_.begin() + jump_.size()-1);
					//also erase from furnJump if this is reinstated
				}
			}
		}
		else if(diff > FurnJumpDist){ // furn jump has lower tolerance for detecting furniture
			// use a smaller averaging scheme of only 3 pts before and after the jump
			// With the LIDAR getting 500pts/scan it's angle between pts is 0.0127 rad
			// which means the dist. between pts is R*0.0127, which equals 3cm at R = 238cm
			float avgPre, avgPost;
			getAveragePrePost(avgPre, avgPost, i, 3); // do 3pt averages before and after
			if(abs(avgPre - avgPost) > furnJump*0.75){ // filter out random bad pts
				furnJump.push_back(i);
			}
		}
	}

	cout<<"Big jumps at angles: ";
	for(int i=0; i<jump_.size(); i++) {cout<<degrees[jump_[i]]<<" "<<rad[jump_[i]]<<"   ";}
	cout<<"\n";
	
	cout<<"furn jumps at angles: ";
	for(int i=0; i<furnJump.size(); i++) {cout<<degrees[furnJump[i]]<<" "<<rad[furnJump[i]]<<"   ";}
	cout<<"\n";
	
	/*
	for(int j=0; j<jump.size(); j++)
		cout<<"at i="<<jump_[j]<<" rad is "<<rad[jump[j]]<<" xval is: "<<xVal[jump[j]] <<" yVal is: "<<yVal[jump[j]]<<"  ";
	cout<<"total i="<<rad.size()<<"\n";
	}*/	
	/*
	for(int j : jump){
		cout<<"prev Pt i="<<j-1<<" angle="<<degrees[j-1]<<" rad ="<<rad[j-1]<<"  ";
		cout<<"Pt i="<<j<<" angle="<<degrees[j]<<" rad ="<<rad[j]<<"  ";
		cout<<"next Pt i="<<j+1<<" angle="<<degrees[j+1]<<" rad ="<<rad[j+1]<<"  ";
		cout<<"next Pt i="<<j+2<<" angle="<<degrees[j+2]<<" rad ="<<rad[j+2]<<"  ";
		cout<<"\n";
	}*/
}

void Lidar::removePt(int i){
	rad.erase(rad.begin() + i );
	degrees.erase(degrees.begin() + i );
	xVal.erase(xVal.begin() + i );
	yVal.erase(yVal.begin() + i );
}

int Lidar::getEndIdx(int s){ return s+1 < rad.size() ? s+1 : 0; }

vector<line> Lidar::findLine(vector <float> xReal, vector <float> yReal){
	vector <line> myLines;
	line tempLine;
	tempLine.setGood(false);
	int scopeSize = 0;			// keeps track of where the line breaks
	int i = 0;					// iterator
	float distToLine = 0;		// distance from a point to a line
	bool twoPointLine = false;  // checks to see if the line has only 2 points
	float distToPoint = 0;			// distance from point to point (If to big, create new line

	//adjustable variables
	//Best Vals
	//pointThreshold:	 .02
	float pointThreshold = 2.5;		//distance between the point and line threshold
	float pointDistThresh = 5.75;		//distance between point to point in a line (if it exceeds this threshold it will make a new line)

	while (i < xReal.size()) {
		for (i; i < xReal.size(); i++) {
			if (i == scopeSize) {	//adding first point to a line
				tempLine.addPointEnd(xReal[i], yReal[i]);
			}

			else if (i == scopeSize +1){							//checking the second point
				distToPoint = pt2PtDist(xReal[i], yReal[i], xReal[i-1], yReal[i-1]);	//checks the distance between the first and second point
				if(distToPoint < pointDistThresh){					//if they are close enough, add second point to line
					tempLine.addPointEnd(xReal[i], yReal[i]);
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
				distToPoint = pt2PtDist(xReal[i], yReal[i], xReal[i-1], yReal[i-1]);
				distToLine = tempLine.findDist(xReal[i], yReal[i]);
				if ((distToLine < pointThreshold)&&(distToPoint < pointDistThresh)) {	//checking the point to the line model
					tempLine.addPointEnd(xReal[i], yReal[i]);
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
				distToPoint = pt2PtDist(xReal[i], yReal[i], xReal[i-1], yReal[i-1]);
				distToLine = myLines[myLines.size()-1].findDist(xReal[i], yReal[i]);
				if (distToLine < pointThreshold) {
					if(distToPoint <= pointDistThresh){
						myLines[myLines.size()-1].addPointEnd(xReal[i], yReal[i]);
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
		} // end of loop going thru all xReals

		tempLine.clearLine();
	}// end of while loop checking that i is < xReal.size

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
	//vector <line> lineVec = findLine(xVal,yVal);

	float room4Vshort = 0;
	for(int i=0; i < rad.size(); i++){
		if(rad[i] < 45){ // max possible distance in Room4 is 65 cm
			room4Vshort++;
		}
		if(rad[i] < 65){ // max possible distance in Room4 is 65 cm
			room4Short++;
		}
		else if(rad[i] > 170){
			room4Long++;
		}
	}
	
	rat4 = room4Short*ratMult;

	float percentShort = (float) room4Short / (float) rad.size();
	float percentVshort = (float) room4Vshort / (float) rad.size();
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
	for(int i=0; i<rad.size(); i++){
		if(rad[i] < 70){
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
		for(int i=0; i < rad.size(); i++){
			if((rad[i] < 150) && (rad[i] > 100)){
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



