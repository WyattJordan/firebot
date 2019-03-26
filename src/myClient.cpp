#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#include <vector>
#include "myClient.h"
#include <ctime>

#define RAD2DEG(x) ((x)*180./M_PI)
#define POLAR2XCART(x, y) ((x)*cos((y)*M_PI/180.)) //get the x component when given a distance and angle in degrees
#define POLAR2YCART(x, y) ((x)*sin((y)*M_PI/180.)) //get the y component when given a distance and angle in degrees

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	int count = scan->scan_time / scan->time_increment;
	ROS_INFO("Testing %s[%d]:", scan->header.frame_id.c_str(), count);
	ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
	time_t start, finish;
	std::vector <float> xVal;
	std::vector <float> yVal;
	float XRange[scan->ranges.size()];
	float YRange[scan->ranges.size()];
	time(&start);
	for(int i = 0; i < count; i++) {
		float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
	//	ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
		if ((isinf(scan->ranges[i]) == 0)&&(scan->ranges[i] < 1.80)){
			xVal.push_back(POLAR2XCART(scan->ranges[i], degree));
			yVal.push_back(POLAR2YCART(scan->ranges[i], degree));
		}
		

	}
	ROS_INFO("Starting findLine...");
	findLine(xVal, yVal);
	ROS_INFO("Finished findLine...");
	time(&finish);
	cout << "Time of program is " << difftime(finish, start) << " seconds" << endl;
//	for(int i = 0; i < xVal.size(); i++){
//		ROS_INFO(" Testing: X = %f, Y = %f", xVal[i], yVal[i]);
//	}
}


void findLine(vector <float> xReal, vector <float> yReal){
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
	//pointDistThresh:	 .4
	float pointThreshold = 0.02;		//distance between the point and line threshold
	float pointDistThresh = .0575;		//distance between point to point in a line (if it exceeds this threshold it will make a new line)

	while (i < xReal.size()) {
		for (i; i < xReal.size(); i++) {
			if (i == scopeSize) {	//adding first point to a line
				
				tempLine.addPointEnd(xReal[i], yReal[i]);
			//	cout << "Part 1 done" << endl;
			}
			else if (i == scopeSize +1){							//checking the second point
				distToPoint = pt2PtDist(xReal[i], yReal[i], xReal[i-1], yReal[i-1]);	//checks the distance between the first and second point
			//	cout << "Part 2 start" << endl;
				if(distToPoint < pointDistThresh){					//if they are close enough, add second point to line
			//		cout << "Part 2 true" << endl;
					tempLine.addPointEnd(xReal[i], yReal[i]);
					tempLine.setFloats();
				}
				else{									//otherwise send to 'not line' array
			//		cout << "Part 2 false" << endl;
					if(myLines.size() == 0){
						tempLine.setFloats();
						tempLine.setGood(false);
						myLines.push_back(tempLine);
						tempLine.clearLine();
					}
					else{
						if(myLines[myLines.size() - 1].isGoodLine()){	//checks to see if there needs to be a new group of bad points
			//				cout << "Part 2 false a." << endl;
							tempLine.setFloats();
							tempLine.setGood(false);
							myLines.push_back(tempLine);
							tempLine.clearLine();
						}
						else{ 
							myLines[myLines.size()-1].mergeLines(tempLine);
							myLines[myLines.size()-1].setFloats();
						}
					}

						
					scopeSize = i;							//saves where it breaks for the next loop
					distToPoint = 0;
					twoPointLine = true;
					break;
				}
			//	cout << "Part 2 done" << endl;
			}


			
			else if (i < scopeSize + 11) {							//checking the third point

				
				distToPoint = pt2PtDist(xReal[i], yReal[i], xReal[i-1], yReal[i-1]);
				distToLine = tempLine.findDist(xReal[i], yReal[i]);
			//	cout << "Check 1" << endl;
			//	cout << distToLine << distToPoint << pointThreshold << pointDistThresh << endl;

				if ((distToLine < pointThreshold)&&(distToPoint < pointDistThresh)) {	//checking the point to the line model
			//		cout << "Check 2" << endl;
					tempLine.addPointEnd(xReal[i], yReal[i]);
					tempLine.setFloats();
					if(i == scopeSize + 10){
						myLines.push_back(tempLine);				//adds the line to a vector of lines
					}
			//		cout << "Check 3" << endl;
				}
				else {									//the point does not fit the line model
			//		cout << "Testing..." << endl;
			//		cout << myLines.size() << endl;
					if (myLines.size() == 0) {
						tempLine.setFloats();
						tempLine.setGood(false);
						myLines.push_back(tempLine);
						tempLine.clearLine();
					}
					else{
						if (myLines[myLines.size()-2].isGoodLine()){
			//				cout << "Check 7" << endl;
							tempLine.setFloats();
							tempLine.setGood(false);
							myLines.push_back(tempLine);
							tempLine.clearLine();
			//				cout << "Check 8" << endl;
						
                                        	}
                                        	else{
			//				cout << "Check 9" << endl;
                                        		myLines[myLines.size() - 1].mergeLines(tempLine);
                                        		myLines[myLines.size() - 1].setFloats();
			//				cout << "Check 10" << endl;
						}
					}


					scopeSize = i;							//saves where it breaks for the next loop
					break;

				}
			//	cout << "Part 3 done" << endl;
			}
			/*
			else if (i < scopeSize + 11) {							//checks points 4-10
                              	cout << "Ch1" << endl;
			      	distToPoint = pt2PtDist(xReal[i], yReal[i], xReal[i-1], yReal[i-1]);
                                distToLine = myLines[myLines.size()-1].findDist(xReal[i], yReal[i]);

                                if ((distToLine < pointThreshold)&&(distToPoint <pointDistThresh)) {	//checking point to line model
					cout << "Ch2" << endl;
					myLines[myLines.size()-1].addPointEnd(xReal[i], yReal[i]);	 //adding point to line
					myLines[myLines.size()-1].setFloats();
                                }
                                else {									//point did not match the line model
					cout << "Ch3" << endl;
					if ((myLines[myLines.size()-2].isGoodLine())||(myLines.size()==0)){
						cout << "Ch4" << endl;
                                        	myLines[myLines.size()-1].setFloats();
						myLines[myLines.size()-1].setGood(false);
                                        }
                                        else{
						cout << "Ch5" << endl;
				       		myLines[myLines.size() - 2].mergeLines(myLines[myLines.size()-1]);
                                        	cout << "Ch6" << endl;
						myLines[myLines.size() - 2].setFloats();
						cout << "Ch7" << endl;
						myLines[myLines.size() - 1].clearLine();
						cout << "Ch8" << endl;
						myLines.erase(myLines.begin() + myLines.size() -1);
						cout << "Ch9" << endl;
					}


                                        scopeSize = i;
                                        break;
                                }
				cout << "Part 4 done" << endl;
			}
			*/


			else {										//goes through points 11 through maxPoint
			//	cout << "New Test" << endl;
				myLines[myLines.size()-1].setGood(true);
				distToPoint = pt2PtDist(xReal[i], yReal[i], xReal[i-1], yReal[i-1]);
				distToLine = myLines[myLines.size()-1].findDist(xReal[i], yReal[i]);

				//if the point is within 1 cm to the line, add the point, otherwise end the line
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
		}

		tempLine.clearLine();
	}

	cout << "Lines have been made" << endl;


//        cout << "real lines before fake lines are added" << endl;
        int numFake = 0;
        int numReal = 0;
  /*      
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
                        cout << "Point distance: " << pt2PtDist(myLines[j].getEndPtX1(),myLines[j].getEndPtY1(), myLines[j].getEndPtX2(), myLines[j].getEndPtY2()) << endl;
                	myLines[j].printLine();
		}
                else{
                        //stopped here
                        //need to fix print, change fakelines to mylines, fix iterator
                        numFake++;
                        cout << endl << "fake line " << numFake << endl;
                        myLines[j].printLine();
                        cout << endl << myLines[j].getLength() << endl << endl;
                }
        }
	*/
	cout << endl << endl;

	for(int j = 0; j < myLines.size(); j++){
		if(myLines[j].isGoodLine()==false){
			//check before and after the fake line
			if(j == 0){
			//	cout << "Test 1" << endl;
				for(int g =0; g < myLines[j].lineSize(); g++){
                         	       distToLine = myLines[myLines.size()-1].findDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
                         	       distToPoint = pt2PtDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g), myLines[myLines.size()-1].getEndPtX2(),myLines[myLines.size()-1].getEndPtY2());
                         	       if((distToLine < pointThreshold)&&(distToPoint < pointDistThresh)){
                         	               //add the point to the line and delete it from the fake line
                         	               myLines[myLines.size()-1].addPointEnd(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
					       myLines[myLines.size()-1].setFloats();
                         	               myLines[j].clearPoint(g);
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
						myLines[g].setFloats();
						myLines.erase(myLines.begin());
						g--;
						h--;
					}
					else{
                                        	myLines[h].mergeLines(myLines[g]);
                                        	myLines[h].setFloats();
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
        

	cout << endl << "real lines after fake lines are added" << endl;
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
			cout << "Point distance: " << pt2PtDist(myLines[j].getEndPtX1(),myLines[j].getEndPtY1(), myLines[j].getEndPtX2(), myLines[j].getEndPtY2()) << endl;
		//	myLines[j].printLine();
		}
		else{
			numFake++;
			cout << endl << "fake line " << numFake << endl;
                	myLines[j].printLine();
                	cout << endl << myLines[j].getLength() << endl << endl;
		}
        }

	findRoom(myLines);

        cout << endl;


//	cout<<"Lines made\n";
	float distToEnd = 0;
};
float pt2PtDist(float x1, float y1, float x2, float y2){
	float dist;
	dist = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
	return dist;
}
//it. canMerge
bool canMerge(line a, line b){
	float slopeThresh = 7;
        float interceptThresh = 7;
        float sizeThresh = 100; 
	float distThresh = .0575;
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
void findRoom(vector <line> lineVec){
	int rm1 = 0;
	int rm2 = 0;
	int rm3 = 0;
	int rm4 = 0;
	bool room1 = false;
	bool room2 = false;
	bool room3 = false;
	bool room4 = false;
	bool shouldBreak = false;
	vector <float> myLength;
	for(int i = 0; i < lineVec.size(); i++){
		myLength.push_back(lineVec[i].getLength());
	}
	for(int i = 0; i < myLength.size(); i++){
		if((myLength[i] <.54) && (myLength[i] > .44)){
			rm1++;
		}
		if((myLength[i] < .81) && (myLength[i] > .71)){
			rm1++;
		}

		if((myLength[i] < 1.08) && (myLength[i] > .98)){
			rm2++;
		}
		if((myLength[i] < .62) && (myLength[i] > .52)){
			rm2++;
			/*
			endpoint temp;
			temp.setCart(lineVec[i].getEndPtX1(), lineVec[i].getEndPtY1());
			endpoint temp2;
			temp2.setCart(lineVec[i].getEndPtX2(), lineVec[i].getEndPtY2());
			endpoint temp3;
			endpoint temp4;
			temp3.setCart(.72, .46);
			temp4.setCart(myLength[i] + .72, .46);
			findStartLocation(temp, temp2, temp3, temp4);
*/

		}

		if((myLength[i] < .31) && (myLength[i] > .21)){
			rm3++;
		}
		if((myLength[i] < .77) && (myLength[i] > .67)){
			rm3++;
		}

		if((myLength[i] < .79) && (myLength[i] > .69)){
		       rm4++;
		}
 		if((myLength[i] < .56) && (myLength[i] > .46)){
			rm4++;
		}
		if((myLength[i] < .33) && (myLength[i] > .23)){
			rm4++;
		}
	}
	cout << "room 1 counter: " << rm1 << endl;
	cout << "room 2 counter: " << rm2 << endl;
	cout << "room 3 counter: " << rm3 << endl;
	cout << "room 4 counter: " << rm4 << endl;

	if(room1){
		cout << endl << endl << "This is room 1" << endl << endl;
	}
	else if(room2){
		cout << endl << endl << "This is room 2" << endl << endl;
	}
	else if(room3){
		cout << endl << endl << "This is room 3" << endl << endl;
	}
	else if(room4){
		cout << endl << endl << "This is room 4" << endl << endl;
	}
	else{
		cout << endl << endl << "Did not find the starting location" << endl << endl;
	}
}

void findStartLocation(endpoint endR1, endpoint endR2, endpoint endG1, endpoint endG2){
	endpoint endG;
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
	float xl = endR1.getX() + x;
	float yl = endR1.getY() + y;
	float a = endG1.getX();
	float b = endG1.getY();
	float num = a - (xl*b/yl);
	float den = yl + (xl*xl/yl);
	float thetaG = asin(num/den)*180/3.14159;
	cout << num << endl << den << endl;
	cout << "Robot orientation: " << thetaG << endl;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "rplidar_node_client");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

	ros::spin();

	return 0;
}

