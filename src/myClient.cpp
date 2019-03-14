#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#include <vector>
#include "incrementalLine.h"
#include "notLine.h"
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
		if ((isinf(POLAR2XCART(scan->ranges[i], degree))) == 0){
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
	float pointDistThresh = .4;		//distance between point to point in a line (if it exceeds this threshold it will make a new line)

	while (i < xReal.size()) {
		for (i; i < xReal.size(); i++) {
			if (i == scopeSize) {	//adding first point to a line
				
				tempLine.addPointEnd(xReal[i], yReal[i]);
				cout << "Part 1 done" << endl;
			}
			else if (i == scopeSize +1){							//checking the second point
				distToPoint = pt2PtDist(xReal[i], yReal[i], xReal[i-1], yReal[i-1]);	//checks the distance between the first and second point
				if(distToPoint < pointDistThresh){					//if they are close enough, add second point to line
					tempLine.addPointEnd(xReal[i], yReal[i]);
				}
				else{									//otherwise send to 'not line' array

					if( (myLines[myLines.size() - 2].isGoodLine()) ||(myLines.size() == 0)){	//checks to see if there needs to be a new group of bad points
						tempLine.setFloats();
						tempLine.setGood(false);
						myLines.push_back(tempLine);
						tempLine.clearLine();
					}
					else{ 
						myLines[myLines.size()-1].mergeLines(tempLine);
						myLines[myLines.size()-1].setFloats();
					}

						
					scopeSize = i;							//saves where it breaks for the next loop
					distToPoint = 0;
					twoPointLine = true;
					break;
				}
				cout << "Part 2 done" << endl;
			}


			
			else if (i == scopeSize + 2) {							//checking the third point

				
				tempLine.setFloats();
				distToPoint = pt2PtDist(xReal[i], yReal[i], xReal[i-1], yReal[i-1]);
				distToLine = tempLine.findDist(xReal[i], yReal[i]);
				cout << "Check 1" << endl;

				if ((distToLine < pointThreshold)&&(distToPoint <= pointDistThresh)) {	//checking the point to the line model
					cout << "Check 2" << endl;
					tempLine.addPointEnd(xReal[i], yReal[i]);
					myLines.push_back(tempLine);				//adds the line to a vector of lines
					cout << "Check 3" << endl;
				}
				else {									//the point does not fit the line model
					cout << "Testing..." << myLines[myLines.size()-2].isGoodLine() << endl;
					if ((myLines[myLines.size()-2].isGoodLine())||(myLines.size() == 0)){
						cout << "Check 7" << endl;
						tempLine.setFloats();
						tempLine.setGood(false);
						myLines.push_back(tempLine);
						tempLine.clearLine();
						cout << "Check 8" << endl;
						
                                        }
                                        else{
						cout << "Check 9" << endl;
                                        	myLines[myLines.size() - 1].mergeLines(tempLine);
                                        	myLines[myLines.size() - 1].setFloats();
						cout << "Check 10" << endl;
					}


					scopeSize = i;							//saves where it breaks for the next loop
					break;

				}
				cout << "Part 3 done" << endl;
			}
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


			else {										//goes through points 11 through maxPoint
				cout << "New Test" << endl;
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
				cout << "Part 4 done" << endl;
			}
		}

		tempLine.clearLine();
	}

	cout << "Lines have been made" << endl;


        cout << "real lines before fake lines are added" << endl;
        int numFake = 0;
        int numReal = 0;
        for (int j = 0; j < myLines.size(); j++) {
		if(myLines[j].isCandle()){
			cout << "This is a candle" << endl;
		}
		else if(myLines[j].isFurniture()){
			cout << "This is furniture" << endl;
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
                }
                else{
                        //stopped here
                        //need to fix print, change fakelines to mylines, fix iterator
                        numFake++;
                        cout << "fake line " << numFake << endl;
                        myLines[j].printLine();
                        cout << endl << myLines[j].getLength() << endl << endl;
                }
        }
	

	for(int j = 0; j < myLines.size(); j++){
		if(myLines[j].isGoodLine()==false){
			//check before and after the fake line
			if(j == 0){
				cout << "Test 1" << endl;
				for(int g =0; g < myLines[j].lineSize(); g++){
                         	       distToLine = myLines[myLines.size()-1].findDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
                         	       distToPoint = pt2PtDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g), myLines[myLines.size()-1].getEndPtX2(),myLines[myLines.size()-1].getEndPtY2());
                         	       if((distToLine < pointThreshold)&&(distToPoint < pointDistThresh)){
                         	               //add the point to the line and delete it from the fake line
                         	               myLines[myLines.size()-1].addPointEnd(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
					       myLines[myLines.size()-1].setFloats();
                         	               myLines[j].clearPoint(g);
                         	               //g--
                         	       }
                       		}
			}
			else{
				cout << "Test 2" << endl;
				for(int g =0; g < myLines[j].lineSize(); g++){
					distToLine = myLines[j-1].findDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
					distToPoint = pt2PtDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g), myLines[j-1].getEndPtX2(),myLines[j-1].getEndPtY2());
					if((distToLine < pointThreshold)&&(distToPoint < pointDistThresh)){
						//add the point to the line and delete it from the fake line
						myLines[j-1].addPointEnd(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
						myLines[j-1].setFloats();
						myLines[j].clearPoint(g);
						//g--;
					}
				}
			}
			myLines[j].reverseLine();
			if(j == myLines.size()-1){
				cout << "Test 3" << endl;
				for(int g = 0; g < myLines[j].lineSize(); g++){
                                	distToLine = myLines[0].findDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
                                	distToPoint = pt2PtDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g), myLines[0].getEndPtX1(), myLines[0].getEndPtY1());
                                	if((distToLine < pointThreshold)&&(distToPoint < pointDistThresh)){
                                        	myLines[0].addPointStart(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
						myLines[0].setFloats();
                                        	myLines[j].clearPoint(g);
                                	}
                        	}
			}
			else{
				cout << "Test 4" << endl;
				for(int g = 0; g < myLines[j].lineSize(); g++){
					distToLine = myLines[j+1].findDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
					distToPoint = pt2PtDist(myLines[j].getXPoint(g), myLines[j].getYPoint(g), myLines[j+1].getEndPtX1(), myLines[j+1].getEndPtY1());
					if((distToLine < pointThreshold)&&(distToPoint < pointDistThresh)){
						myLines[j+1].addPointStart(myLines[j].getXPoint(g), myLines[j].getYPoint(g));
						myLines[j+1].setFloats();
						myLines[j].clearPoint(g);
					}
				}
			}
			myLines[j].reverseLine();
			if(myLines[j].lineSize() == 0){
				myLines[j].clearLine();
				j--;
			}
		}

	}	

	for (int g = 0; g < myLines.size(); g++){
                if(myLines[g].isGoodLine()){
                        for (int h = 0; h < g; h++){
                                if (( canMerge(myLines[g], myLines[h]) == true)&&(myLines[h].isGoodLine())){
                                        myLines[h].mergeLines(myLines[g]);
                                        myLines[h].setFloats();
                                        myLines.erase(myLines.begin() + g);
                                        g--;
                                }
                        }
                }
	}
        

	cout << endl << "real lines after fake lines are added" << endl;
	numFake = 0;
	numReal = 0;
	for (int j = 0; j < myLines.size(); j++) {
		if(myLines[j].isCandle()){
			cout << "This is a candle" << endl;
		}
		else if(myLines[j].isFurniture()){
			cout << "This is furniture" << endl;
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
		}
		else{
			numFake++;
			cout << "fake line " << numFake << endl;
                	myLines[j].printLine();
                	cout << endl << myLines[j].getLength() << endl << endl;
		}
        }

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
	float distThresh = .3;
	float myDist;
	float tempDist;


	
	myDist = pt2PtDist(a.getEndPtX1(), a.getEndPtY1(), b.getEndPtX1(), b.getEndPtY1());

        tempDist = pt2PtDist(a.getEndPtX2(), a.getEndPtY2(), b.getEndPtX2(), b.getEndPtY2());
        if (tempDist < myDist) {myDist = tempDist;}

        tempDist = pt2PtDist(a.getEndPtX1(), a.getEndPtY1(), b.getEndPtX2(), b.getEndPtY2());
        if (tempDist < myDist) {myDist = tempDist;}

        tempDist = pt2PtDist(a.getEndPtX2(), a.getEndPtY2(), b.getEndPtX1(), b.getEndPtY1());
        if (tempDist < myDist) {myDist = tempDist;}
	
	if(myDist < distThresh){
		if((a.getSlope()*b.getSlope() > -1.7) && (a.getSlope()*b.getSlope() < -.3))
			return false;
	
		else
			return true;
	}
	else
		return false;
}

void line::setSlope(float s){
	slope = s;
};

void line::setIntercept(float i){
	intercept = i;
};

float line::getXPoint(int point){
	return x[point];
};

float line::getYPoint(int point){
	return y[point];
};

//it. setFloats
void line::setFloats() {
	float xAvg = 0, yAvg = 0;
	for (int i = 0; i < x.size(); i++) {
		xAvg += x[i];
		yAvg += y[i];
	};
	xAvg /= (x.size());
	yAvg /= (y.size());

	float num = 0, denum = 0;

	for (int i = 0; i < x.size(); i++) {
		num += (x[i] - xAvg)*(y[i] - yAvg);
		denum += pow(x[i] - xAvg, 2);
	};
	slope = num / denum;
	intercept = yAvg - slope * xAvg;
	setEndpts(x[0], y[0], x[x.size()-1], y[y.size()-1]);
	length = pt2PtDist(end1.getX(), end1.getY(), end2.getX(), end2.getY());
	if ((length > 0.01)&&(length < 0.04)){
                candle = true;
                furniture = false;
        }
        else if ((length > 0.09)&&(length < 0.13)){
                candle = false;
                furniture = true;
        }
        else{
                candle = false;
                furniture = false;
        }
};

float line::getIntercept() {

	return intercept;
};

float line::getSlope() {

	return slope;
};
//it. addPoint line
void line::addPointEnd(float xVal, float yVal) {
	x.push_back(xVal);
	y.push_back(yVal);
};

void line::addPointStart(float xVal, float yVal) {
	x.insert(x.begin(), xVal);
	y.insert(y.begin(), yVal);
};
//it. findDist
float line::findDist(float xPoint, float yPoint) {
	float a, b, c;
	a = slope;
	b = -1;
	c = intercept;
	float num, denum;
	num = abs(a*xPoint + b * yPoint + c);
	denum = sqrt(pow(a, 2) + pow(b, 2));
	float d;
	d = num / denum;

	return d;
};
//it. clearLine
void line::clearLine() {
	x.clear();
	y.clear();
	slope = 0;
	intercept = 0;
	lineDist = 0;
	end1.setCart(0,0);
	end2.setCart(0,0);
};
//it. clearPoint
void line::clearPoint(int i){
	x.erase(x.begin() + i);
	y.erase(y.begin() + i);
};
//it. printLine
void line::printLine() {
	cout << "Point count: " << x.size() << endl;
        cout << "First endpoint cartesian:  (" << end1.getX() << ", " << end1.getY() << ")" << endl;
        cout << "Second endpoint cartesian: (" << end2.getX() << ", " << end2.getY() << ")" << endl;
        cout << endl << "First endpoint polar:   R: " << end1.findRad() << " Angle: " << end1.findAngle() << endl;
        cout << "Second endpoint polar:  R: " << end2.findRad() << " Angle: " << end2.findAngle() << endl;
        cout << "Points" << endl;
        for(int i = 0; i < x.size(); i++){
                cout << "(" << x[i] << ", " << y[i] << ")" << endl;
        }
};
//it. endPAngle
float line::endPAngle(int num){
	if(num == 1) {return end1.findAngle();}
	else if (num == 2) {return end2.findAngle();}
	else {return 0;}
}
//it. endPRad
float line::endPRad(int num){
	if(num == 1) {return end1.findRad();}
	else if (num == 2) {return end2.findRad();}
	else {return 0;}
}


//it. lineSize
float line::lineSize() {
	return x.size();
}
//it. reverseLine
void line::reverseLine(){
	reverse(x.begin(), x.end());
	reverse(y.begin(), y.end());
}
//it. setEndpts
void line::setEndpts(float x1, float y1, float x2, float y2){
	end1.setCart(x1, y1);
	end2.setCart(x2, y2);
	lineDist = pt2PtDist(x1, y1, x2, y2);
}
//it. getEndPt line
float line::getEndPtX1(){
	return end1.getX();
}
float line::getEndPtY1(){
	return end1.getY();
}
float line::getEndPtX2(){
	return end2.getX();
}

float line::getEndPtY2(){
	return end2.getY();
}
//it. mergeLines
void line::mergeLines(line a) {//line a gets merged into the main line
	for(int i = 0; i < a.lineSize(); i++){
		x.push_back(a.getXPoint(i));
	     	y.push_back(a.getYPoint(i));
		//x.insert(x.begin(), a.getXPoint(a.lineSize() - i));
		//y.insert(y.begin(), a.getYPoint(a.lineSize() - i));
		//still have to delete the line a in findLine
	}
	a.clearLine();

}
//it. getLineDist
float line::getLineDist(){
	return lineDist;
}
//it. setGood
void line::setGood(bool a){
	isLine = a;
}
//it. isGoodLine
bool line::isGoodLine(){
	return isLine;
}
float line::getLength(){
	return length;
}
bool line::isCandle(){
	return candle;
}
bool line::isFurniture(){
	return furniture;
}



void endpoint::setCart(float xIn, float yIn){
	x = xIn;
	y = yIn;
}
float endpoint::getX(){
	return x;
}
float endpoint::getY(){
	return y;
}
float endpoint::findAngle(){
	float angle;
        float t = atan2(y, x) * 180 / 3.14159;
        angle = t>0 ? t : t + 360; // polar vals range 0:360
	return angle;
}
float endpoint::findRad(){
	float rad;
	rad = pow(x*x + y*y, 0.5);
	return rad;
}
void endpoint::clear(){
	x = 0;
	y = 0;
}


void notLine::addFront(float xVal, float yVal){
        x.push_back(xVal);
        y.push_back(yVal);
}
void notLine::addRear(float xVal, float yVal) {
        x.insert(x.begin(), xVal);
        y.insert(y.begin(), yVal);
}

void notLine::setEndpoints(){
        fEnd1.setCart(x[0], y[0]);
        fEnd2.setCart(x[x.size()-1], y[y.size()-1]);
	length = pt2PtDist(fEnd1.getX(), fEnd1.getY(), fEnd2.getX(), fEnd2.getY());
	if ((length > 0.01)&&(length < 0.04)){
		candle = true;
		furniture = false;
	}
	else if ((length > 0.09)&&(length < 0.13)){
		candle = false;
		furniture = true;
	}
	else{
		candle = false;
		furniture = false;
	}
}

int notLine::getSize(){
        return x.size();
}
void notLine::reverseNL(){
        reverse(x.begin(), x.end());
        reverse(y.begin(), y.end());
}
float notLine::getXPoint(int i){
        return x[i];
}
float notLine::getYPoint(int i){
        return y[i];
}
void notLine::clearNL(){
        x.clear();
        y.clear();
        fEnd1.clear();
        fEnd2.clear();
}
void notLine::clearPoint(int i){
        x.erase(x.begin() + i);
        y.erase(y.begin() + i);
}
float notLine::getEndPtX1(){
        return fEnd1.getX();
}
float notLine::getEndPtY1(){
        return fEnd1.getY();
}
float notLine::getEndPtX2(){
        return fEnd2.getX();
}
float notLine::getEndPtY2(){
        return fEnd2.getY();
}
void notLine::print(){
        cout << "Point count: " << x.size() << endl;
        cout << "First endpoint cartesian:  (" << fEnd1.getX() << ", " << fEnd1.getY() << ")" << endl;
        cout << "Second endpoint cartesian: (" << fEnd2.getX() << ", " << fEnd2.getY() << ")" << endl;
        cout << endl << "First endpoint polar:   R: " << fEnd1.findRad() << " Angle: " << fEnd1.findAngle() << endl;
        cout << "Second endpoint polar:  R: " << fEnd2.findRad() << " Angle: " << fEnd2.findAngle() << endl;
        cout << "Points" << endl;
        for(int i = 0; i < x.size(); i++){
                cout << "(" << x[i] << ", " << y[i] << ")" << endl;
        }
}
void notLine::lineToNL(line a){
        for(int i = 0; i < a.lineSize(); i++){
                x.push_back(a.getXPoint(i));
                y.push_back(a.getYPoint(i));
                //x.insert(x.begin(), a.getXPoint(a.lineSize() - i));
                //y.insert(y.begin(), a.getYPoint(a.lineSize() - i));
                //still have to delete the line a in findLine
        }
        a.clearLine();
}
void notLine::mergeFLine(notLine a){
	for(int i = 0; i < a.getSize(); i++){
		x.push_back(a.getXPoint(i));
		y.push_back(a.getYPoint(i));
	}
	a.clearNL();
}
float notLine::endPAngle(int num){
        if(num == 1) {return fEnd1.findAngle();}
        else if (num == 2) {return fEnd2.findAngle();}
        else {return 0;}
}
float notLine::getLength(){
	return length;
}
bool notLine::isCandle(){
	return candle;
}
bool notLine::isFurniture(){
	return furniture;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "rplidar_node_client");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

	ros::spin();

	return 0;
}

