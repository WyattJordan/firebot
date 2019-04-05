#include "line.h"
#include "Endpoint.h"
#include <iostream>
#include <vector>
#include "math.h"

float ptDist(float x1, float y1, float x2, float y2){
	return pow(pow(x2-x1,2) + pow(y2-y1,2) ,0.5);
}

line::line(){
        slope = 0;
        intercept = 0;
        EndPoint end1;
        EndPoint end2;
        lineDist = 0;
        isLine = false;
        length = 0;
        candle = false;
        furniture = false;
}

void line::buildLine() {
        float xAvg = 0;
       	float yAvg = 0;
        for (int i = 0; i < x.size(); i++) {
                xAvg += x[i];
                yAvg += y[i];
        };
	

        xAvg /= (x.size());
        yAvg /= (y.size());
		center = EndPoint(xAvg, yAvg);

		float num = 0, denum = 0;
        for (int i = 0; i < x.size(); i++) {
                num += (x[i] - xAvg)*(y[i] - yAvg);
                denum += pow(x[i] - xAvg, 2);
        };
        slope = num / denum;
        intercept = yAvg - slope * xAvg;
        setEndpts(x[0], y[0], x[x.size()-1], y[y.size()-1]);
        length = ptDist(end1.getX(), end1.getY(), end2.getX(), end2.getY());
        if ((length > 1)&&(length < 4)){
                candle = true;
                furniture = false;
        }
        else if ((length > 9)&&(length < 13)){
                candle = false;
                furniture = true;
        }
        else{
                candle = false;
                furniture = false;
        }
        if(isLine){}
        else{
               isLine = false;
        }
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

void line::clearLine() {
        x.clear();
        y.clear();
        slope = 0;
        intercept = 0;
        lineDist = 0;
        end1.setCart(0,0);
        end2.setCart(0,0);
        isLine = false;
        candle = false;
        furniture = false;
};
//it. clearPoint
void line::clearPoint(int i){
        x.erase(x.begin() + i);
        y.erase(y.begin() + i);
};
//it. printLine
void line::printLine() {
        cout << "Point count: " << x.size() << endl;
        cout << "First Endpoint cartesian:  (" << end1.getX() << ", " << end1.getY() << ")" << endl;
        cout << "Second Endpoint cartesian: (" << end2.getX() << ", " << end2.getY() << ")" << endl;
        cout << endl << "First Endpoint polar:   R: " << end1.findRad() << " Angle: " << end1.findAngle() << endl;
        cout << "Second Endpoint polar:  R: " << end2.findRad() << " Angle: " << end2.findAngle() << endl;
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
float line::endPRad(int num){
        if(num == 1) {return end1.findRad();}
        else if (num == 2) {return end2.findRad();}
        else {return 0;}
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
        lineDist = ptDist(x1, y1, x2, y2);
}
void line::mergeLines(line a) {//line a gets merged into the main line
	for(int i = 0; i < a.numPts(); i++){
			x.push_back(a.getXPoint(i));
			y.push_back(a.getYPoint(i));
			//x.insert(x.begin(), a.getXPoint(a.numPts() - i));
			//y.insert(y.begin(), a.getYPoint(a.numPts() - i));
			//still have to delete the line a in findLine
	}
	a.clearLine();
	buildLine();
}

//it. canMerge
bool line::canMerge(line a){
	float distThresh = 10;
	float percentSlopeErrorThresh = 0.1;

	// find closest distance between any of the two enpoints of the lines
	float myDist = ptDist(a.getEndPtX1(), a.getEndPtY1(), getEndPtX1(), getEndPtY1());

	float tempDist = ptDist(a.getEndPtX2(), a.getEndPtY2(), getEndPtX2(), getEndPtY2());
	if (tempDist < myDist) {myDist = tempDist;}

	tempDist = ptDist(a.getEndPtX1(), a.getEndPtY1(), getEndPtX2(), getEndPtY2());
	if (tempDist < myDist) {myDist = tempDist;}

	tempDist = ptDist(a.getEndPtX2(), a.getEndPtY2(), getEndPtX1(), getEndPtY1());
	if (tempDist < myDist) {myDist = tempDist;}

	if(myDist < distThresh){
		float slopeErr = abs(a.getSlope()-getSlope()) / a.getSlope();
		if(slopeErr < percentSlopeErrorThresh) return true; 
	}
	return false;
}

float line::radDist(int i){
	float rad;
        rad = pow(x[i]*x[i] + y[i]*y[i], 0.5);
        return rad;
}

void line::addPointEnd(float xVal, float yVal) {
        x.push_back(xVal);
        y.push_back(yVal);
};

void line::addPointStart(float xVal, float yVal) {
        x.insert(x.begin(), xVal);
        y.insert(y.begin(), yVal);
};

float line::getXPoint(int point){ return x[point]; };
float line::getYPoint(int point){ return y[point]; };
int line::numPts() { return x.size(); }

//it. getLineDist
float line::getLineDist(){ return lineDist; }
float line::getIntercept() { return intercept; };
float line::getSlope() { return slope; };
float line::getLength(){ return length; }
//it. setGood
void line::setSlope(float s){ slope = s; };
void line::setIntercept(float i){ intercept = i; };
void line::setGood(bool a){ isLine = a; }
//it. isGoodLine
bool line::isGoodLine(){ return isLine; }
bool line::isCandle(){ return candle; }
bool line::isFurniture(){ return furniture; }
//it. getEndPt line
float line::getEndPtX1(){ return end1.getX(); }
float line::getEndPtY1(){ return end1.getY(); }
float line::getEndPtX2(){ return end2.getX(); }
float line::getEndPtY2(){ return end2.getY(); }
float line::getCenterTheta(){ return center.findAngle();}
float line::getCenterRadius(){ return center.findAngle();}
float line::getCenterX(){ return center.getX(); }
float line::getCenterY(){ return center.getY(); }


