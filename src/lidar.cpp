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


void Lidar::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{

	/*
	bool updatePosition = false;
	Vector3f currentPos = rob_->getOdomWorldLoc(); // this is an undefined ref for some reason...
	//Vector3f currentPos;
	if(! (prevOdom_(0) == -100 && prevOdom_(1) == -100)) // if not the first run (default odom loc) 
	{
		// if the angle has changed less than 5deg between the two positions
		if(abs(prevOdom_(2) - currentPos(2))*180/PI < 5) updatePosition = true;
	
	}*/ //stuff for position updates

	int count = scan->scan_time / scan->time_increment;
	//ROS_INFO("Testing %s[%d]:", scan->header.frame_id.c_str(), count);
	//ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
	time_t start, finish;
	std::vector <float> xVal;
	std::vector <float> yVal;
	float XRange[scan->ranges.size()];
	float YRange[scan->ranges.size()];
	time(&start);
	for(int i = 0; i < count; i++) {
		float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
	//	ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
		if ((isinf(scan->ranges[i]) == 0)&&(scan->ranges[i] < 180)&&(scan->ranges[i] > 18)){
			xVal.push_back(POLAR2XCART(scan->ranges[i], degree));
			yVal.push_back(POLAR2YCART(scan->ranges[i], degree));
		}
		

	}
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

void Lidar::findRoom(vector <line> lineVec){
	int rm1 = 0;
	int rm2 = 0;
	int rm3 = 0;
	int rm4 = 0;
	float ratMult = 0.0016949;
	float rat23 = 0;
	float rat4 = 0;
	float maxDist = 0;
	int room23Count = 0;
	int room4Count = 0;
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
	for(int i=0; i < lineVec.size(); i++){
		for(int j=0; j < lineVec[i].lineSize(); j++){
			if(lineVec[i].radDist(j) < 45){
				room4Count++;
			}
			else if(lineVec[i].radDist(j) > 135){
				room4Long++;
			}
		}
	}
	rat4 = room4Count*ratMult;
	if(rat4 >0.65){
		if(room4Long > 3){
			room4l = true;
		}
		else{
			room4s = true;
		}
	}
	
	for(int i=0; i<lineVec.size(); i++){
		for(int j=0; j < lineVec[i].lineSize(); j++){
			if(lineVec[i].radDist(j) > 70){
				maxDist = lineVec[i].radDist(j);
			}
			if(lineVec[i].radDist(j) < 70){
				room23Count++;
			}
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
		if(rm2 > rm3){
			room2 = true;
		}
		else {
			room3 = true;
		}
	}
		/*	
			refpoint temp;
			temp.setCart(lineVec[i].getEndPtX1(), lineVec[i].getEndPtY1());
			refpoint temp2;
			temp2.setCart(lineVec[i].getEndPtX2(), lineVec[i].getEndPtY2());
			refpoint temp3;
			refpoint temp4;
			temp3.setCart(.72, .46);
			temp4.setCart(myLength[i] + .72, .46);
			findStartLocation(temp, temp2, temp3, temp4);
		*/
	if((!room4l) && (!room4s) && (room2) && (!room3)){
		for(int i=0; i < lineVec.size(); i++){
			for(int j=0; j < lineVec[i].lineSize(); j++){
				if((lineVec[i].radDist(j) < 150) && (lineVec[i].radDist(j) > 100)){
					rm1++;
				}
			}
		}
		if(rm1 > 47) {room1b = true;}
		else {room1a = true;}
	}
	if(room4l){
		cout << endl << endl << "This is room 4 toward the maze" << endl << endl;
		for(int i = 0; i < lineVec.size(); i++){
			if((myLength[i] > 24) && (myLength[i] < 30)){
				cout << "Gotcha, bitch!" << endl;
			}
			else if((myLength[i] > 68) && (myLength[i] < 74)){
				cout << "Gotcha again, bitch!" << endl;
			}
		}
	}
	else if(room4s){
		cout << endl << endl << "This is room 4 toward the wall" << endl << endl;
		for(int i = 0; i < lineVec.size(); i++){
			if((myLength[i] > 24) && (myLength[i] < 30)){
				cout << "Gotcha, bitch!" << endl;
			}
			else if((myLength[i] > 68) && (myLength[i] < 74)){
				cout << "Gotcha again, bitch!" << endl;
			}
		}
	}
	else if(room2){
		cout << endl << endl << "This is room 2" << endl << endl;
		for(int i = 0; i < lineVec.size(); i++){
                        if((myLength[i] > 95) && (myLength[i] < 108)){
                                cout << "Gotcha, bitch!" << endl;
                        }
                        else if((myLength[i] > 50) && (myLength[i] < 64)){
                                cout << "Gotcha again, bitch!" << endl;
                        }
                }
	}
	else if(room3){
		cout << endl << endl << "This is room 3" << endl << endl;
		for(int i = 0; i < lineVec.size(); i++){
                        if((myLength[i] > 120) && (myLength[i] < 137)){
                                cout << "Gotcha, bitch!" << endl;
                        }
                        else if((myLength[i] > 23) && (myLength[i] < 31)){
                                cout << "Gotcha again, bitch!" << endl;
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
	cout << rat4 << endl << rat23 << endl;
	cout << "room 1 counter: " << rm1 << endl;
}

void Lidar::findStartLocation(refpoint endR1, refpoint endR2, refpoint endG1, refpoint endG2){
	refpoint endG;
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



