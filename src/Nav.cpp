/* nav.cpp
 *
 *
 */
#include "Nav.h"
#include "Endpoint.h"
#include "ros/ros.h"
#include <ros/console.h> 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//#include <rviz_visual_tools/rviz_visual_tools.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
//#include <pthread.h>
#include <time.h>
#include <chrono>
#include <algorithm>
using std::vector;
using std::atof;
// don't use default constuctor, it must load the map and way files
Nav::Nav(){}

// gets a point given an ID, assumes in order initially then searches if not
EndPoint& Nav::getPoint(int id, vector<EndPoint> &pts){
	if(pts[id].getID() == id) return pts[id];
	for(int i=0; i<pts.size(); i++){
		if(pts[i].getID() == id) return pts[i];
	}
	return badPt;
}

// this is the main constructor, initialize variables here
Nav::Nav(string mapfile, string wayfile){  
	smallRoomConf = bigRoomConf = false;
	vector<int> none;
	none.resize(0);
	badPt = EndPoint(-1, -1, -1, none);
	mapPoints.resize(0);
	wayPoints.resize(0);

	// the file loading into an Endpoint vector should be made into it's own function
	// though this does require identical formatting in the file fields
        
	std::ifstream file;
        file.open(mapfile.c_str());
        string line = "";

        vector<string> nums;
	if (file.is_open()){
                while(getline(file, line, ',')){
                        nums.push_back(line); //std::cout<<line<<"\n";
                }
                file.close();
        }
	else{ ROS_ERROR("WARNING: map file not found!\n"); }

	vector<int> temp;
	while(nums.size()>=7){
		temp.resize(0);
		// nums[3] and nums[4] are definite and potential corner types, ignored here	
		if(nums[5].compare("x") != 0) temp.push_back(std::atof(nums[5].c_str())); 
		if(nums[6].compare("x") != 0) temp.push_back(std::atof(nums[6].c_str())); 
		EndPoint tmp(std::atof(nums[0].c_str()), std::atof(nums[1].c_str()),
		    std::atof(nums[2].c_str()),temp);
		mapPoints.push_back(tmp);
		for(int i=0; i<7; i++) nums.erase(nums.begin()+0);
	}

	nums.resize(0);
	file.open(wayfile.c_str());
	std::cout<<"outputting "<<wayfile<<"\n";
        if (file.is_open()){
                while(getline(file, line, ',')){
                        nums.push_back(line); //std::cout<<line<<"\n";
                }
                file.close();
        }
	else{ ROS_ERROR("WARNING: way file not found!\n"); }

	while(nums.size()>=7){
		vector<int> temp;
		// nums[3] is radius here unused 
		if(nums[4].compare("x") != 0) temp.push_back(std::atof(nums[4].c_str())); 
		if(nums[5].compare("x") != 0) temp.push_back(std::atof(nums[5].c_str())); 
		if(nums[6].compare("x") != 0) temp.push_back(std::atof(nums[6].c_str())); 
		EndPoint tmp(std::atof(nums[0].c_str()), std::atof(nums[1].c_str()),
		    std::atof(nums[3].c_str()),temp); // nums[2] is radius set to 10 constant
		wayPoints.push_back(tmp);
		for(int i=0; i<7; i++) nums.erase(nums.begin()+0);
	}
}

// remove a point with a given ID
void Nav::removePoint(int id, vector<EndPoint> &pts){
	for(int i=0; i<pts.size(); i++){
		if(pts[i].getID() == id){
			pts.erase(pts.begin() + i);
			break;
		}
	}
}

// sets the door configuration for the small room, if up == true the 
// door is on the higher side (larger y coordinate) 
void Nav::setSmallRoomUpper(bool up){
	if(up){
		getPoint(18,mapPoints).setNeighbors(12,-1);		
		getPoint(13,mapPoints).setNeighbors(14,-1);
		getPoint(11,mapPoints).setNeighbors(12,14);
		getPoint(14,mapPoints).setNeighbors(13,11);
		removePoint(19,mapPoints);
	}
	else{
		getPoint(11,mapPoints).setNeighbors(12,-1);
		getPoint(19,mapPoints).setNeighbors(14,-1);
		getPoint(12,mapPoints).setNeighbors(11,13);
		getPoint(13,mapPoints).setNeighbors(12,14);
		removePoint(18,mapPoints);
	}
}

// sets the door configuration for the larger room, if up == true the door is
// in the higher location (larger y coordinate)
void Nav::setBigRoomUpper(bool up){
	if(up){
		getPoint(17,mapPoints).setNeighbors(20,-1);
		getPoint(16,mapPoints).setNeighbors(15,-1);
	}
	else{
		getPoint(17,mapPoints).setNeighbors(16,-1);
		removePoint(20,mapPoints);
	}	
}

void Nav::findExpected(float Rx, float Ry, vector<EndPoint> &pts){
	std::cout<<"instant inside findExpected\n";
	outputGraph(pts);
	for(EndPoint &ep : pts){
	       	ep.getPolar(Rx, Ry); // calculate all polars
		ep.setDone(false);
	}

	std::sort(pts.begin(), pts.end(), 
			[](const EndPoint& lhs, const EndPoint &rhs){
			return	lhs.getR() < rhs.getR();	
		}); // sort by points closest to robot
	std::cout<<"after sorting by polar:\n";
	outputGraph(pts);
	EndPoint ep;
	int index = 0;
	bool go = true;
	while(go){
		pts[index].setVisible(true);
		pts[index].setDone(true);
		// this will only ever loop once or twice
		for(int i=0; i<pts[index].getNumNeighbors(); i++){ 
			getNeighbor(pts[index].getID(), i, ep, pts);
			eliminatePts(ep, pts[index], Rx, Ry, pts);
		}

		for(EndPoint &p : pts){
			if(!p.getDone()){
				go = true;
				break;
			}
			else{ go = false;}
		}	

		while(index<pts.size() &&  pts[index].getDone()){ index++; }
	//	std::cout<<"index is: "<<index<<"\n";
	}
	std::cout<<"done finding expected\n";
}

void Nav::eliminatePts(EndPoint &ep1,EndPoint &ep2, float Rx, float Ry, vector<EndPoint> &pts){
	//std::cout<<"elim from "<<ep1.getID()<<" to "<<ep2.getID()<<"\n";
	for(EndPoint &p : pts){
		if(!p.getDone()){

		if(p.getR()<15 || p.getR() > 180){
			p.setVisible(false);
			p.setDone(true);
		}
		else{
			float theta = p.getTheta();
			bool vert = ep1.getx() == ep2.getx() ? 1 : 0; // otherwise horiz line	
			bool above = (vert && Rx < ep1.getx()) || (!vert && Ry < ep1.gety());
			float start = std::min(ep1.getTheta(),ep2.getTheta());
			float end =   std::max(ep1.getTheta(),ep2.getTheta());
			bool cross = false;
			if(end - start> 180) { // they are crossing the 0 
				cross = true;	
			}	
			float margin = 5; // if it's within 5 deg still eliminate
			if( ((!cross && theta > start - margin && theta < end + margin) ||
				(cross && (theta > end - margin || theta < start + margin))) 
			    &&   (	
			    ( vert &&  above && p.getx() > ep1.getx()) ||
			    ( vert && !above && p.getx() < ep1.getx()) ||
			    (!vert &&  above && p.gety() > ep1.gety()) ||
			    (!vert && !above && p.gety() < ep1.gety())    )
			  ){  
				p.setVisible(false);
				p.setDone(true);
//				std::cout<<"\tpoint "<<p.getID()<<" blocked and done\n";
			}
			else{ // if not blocked
				p.setVisible(true);
			}
		}
		}
	}
}

// give a point ID and a neighbor index returns true if a neighbor exists and copies that
// neighbor to the reference neigh
bool Nav::getNeighbor(int startID, int neighI, EndPoint &neigh, vector<EndPoint> &pts){ 
	EndPoint start = getPoint(startID, pts);
	if((start.getx() == -1 && start.gety() == -1)
			|| start.getNumNeighbors() - 1 < neighI){
		neigh = badPt;
		return false;
	}
	else {
		neigh = getPoint(start.getNeighborID(neighI), pts);	
		return true;
	}	
}

// move the robot around the map and publish what mapPoints it can see
void Nav::run(){
	float x,y;
	y = 30;
	unsigned int sleep = 10000;
	float iter = 100;
	for( int i = 0; i<iter; i++){
		x = 230.0/iter * i; publishGraph(x,y, "test", mapPoints);
	}	
	x = 230;
	for( int i = 0; i<iter; i++){
		y = 230.0/iter * i; publishGraph(x,y, "test", mapPoints);
	}	
	y = 230;
	for( int i = 0; i<iter; i++){
		x = 230.0/iter * (iter - i); publishGraph(x,y, "test", mapPoints);
	}	
	x = 30;
	for( int i = 0; i<iter; i++){
		y = 230.0/iter * (iter - i); publishGraph(x,y, "test", mapPoints);
	}	
}


void Nav::publishGraph(float Rx, float Ry, string NS, vector<EndPoint> &pts){
	auto start = std::chrono::steady_clock::now();
	string worldFrame = "map2";
	ros::NodeHandle n;
	ros::Publisher rvizMap = n.advertise<visualization_msgs::MarkerArray>("map",1000);
	visualization_msgs::MarkerArray marks;
	marks.markers.resize(2*pts.size());
	// add vertical arrows at walls corners and endpoints
	for(int i=0; i<pts.size(); i++){
		marks.markers[i].header.frame_id = worldFrame;
		marks.markers[i].ns = NS; 
		marks.markers[i].id = i; //pts[i].getID();
		marks.markers[i].type = visualization_msgs::Marker::ARROW;
		marks.markers[i].action = visualization_msgs::Marker::ADD;
		
		marks.markers[i].points.resize(2);
		marks.markers[i].points[0].x = pts[i].getx();
		marks.markers[i].points[0].y = pts[i].gety();
		marks.markers[i].points[0].z = 0;
		marks.markers[i].points[1].x = pts[i].getx();
		marks.markers[i].points[1].y = pts[i].gety();
		marks.markers[i].points[1].z = 10; // 10cm tall 

		marks.markers[i].scale.x = 3; 
		marks.markers[i].scale.y = 3;
		marks.markers[i].scale.z = 3;
		//bool visible = std::find(expectedIDs.begin(), expectedIDs.end(), 
				//pts[i].getID()) != expectedIDs.end();
		marks.markers[i].color.a = 1.0;	
		marks.markers[i].color.r = 0;	
		if(pts[i].isVisible()){ std::cout<< "drawing visible marks\n";	}
		marks.markers[i].color.g = pts[i].isVisible() ? 1.0 : 0;
		marks.markers[i].color.b = pts[i].isVisible() ? 0 : 1.0;	

		// show text ID above marker with id += 1000
		int text = i+pts.size();
		marks.markers[text].header.frame_id = worldFrame;
		marks.markers[text].id = i+1000;
		marks.markers[text].ns = NS;
		marks.markers[text].text = std::to_string(pts[i].getID());
		marks.markers[text].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marks.markers[text].action = visualization_msgs::Marker::ADD;
		marks.markers[text].scale.x = 3;
		marks.markers[text].scale.y = 3;
		marks.markers[text].scale.z = 8.0; // text height	
		marks.markers[text].pose.position.x = pts[i].getx();
		marks.markers[text].pose.position.y = pts[i].gety();
		marks.markers[text].pose.position.z = 15;
		marks.markers[text].pose.orientation.x = 0;
		marks.markers[text].pose.orientation.y = 0;
		marks.markers[text].pose.orientation.z = 0;
		marks.markers[text].pose.orientation.w = 1;
		marks.markers[text].color.a = 1.0; 
		marks.markers[text].color.r = 1; 
		marks.markers[text].color.g = 1; 
		marks.markers[text].color.b = 1; 
		//sleep(1); rvizMap.publish(marks); //uncomment to see point growth from closest to robot
	}

	// add robot sphere
	visualization_msgs::Marker robot;
	robot = marks.markers[0];
	robot.ns = NS;
	robot.id = 9999;
	robot.type = visualization_msgs::Marker::SPHERE;
	robot.pose.position.x = Rx;
	robot.pose.position.y = Ry;
	robot.pose.position.z = 5;
	robot.scale.x = 10;	
	robot.scale.y = 10;	
	robot.scale.z = 10;	
	robot.color.g = 1;
	robot.color.b = 1;
	marks.markers.push_back(robot);

	// add horizontal lines on floor
	visualization_msgs::Marker mark;
	mark.type = visualization_msgs::Marker::LINE_STRIP;
	mark.header.frame_id = worldFrame;
	mark.ns = NS;
	mark.color.a = 1;
	mark.color.r = 1;
	mark.color.g = 0;
	mark.color.b = 0;
	mark.scale.x = 3;
	mark.points.resize(2);
	mark.points[0].z = 0;
	mark.points[1].z = 0;

	EndPoint ep1, ep2;
	vector<bool> accountedForIDs(pts.size()*10, false); // index is ID
	int id;		
	
	for(int i=0; i<pts.size(); i++){ 	// loop thru points to find connections
		id = pts[i].getID();
				//std::cout<<"got ID\n";
		if(!accountedForIDs[id]){
			//TODO figure out why the second part of the condition must be
			// commented out for the waypoints
			bool add1 = getNeighbor(pts[i].getID(), 0, ep1, pts)  &&
			      	!accountedForIDs[ep1.getID()];
			///std::cout<<"neigh1 ID: "<<ep1.getID()<<"\n";
			bool add2 = getNeighbor(pts[i].getID(), 1, ep2, pts) && 
				!accountedForIDs[ep2.getID()];
			//std::cout<<"neigh2 ID: "<<ep2.getID()<<"\n";

			if( add1 || add2 ){
				mark.points.resize(2);
				mark.points[1].x = pts[i].getx();
				mark.points[1].y = pts[i].gety();
				if(add1){
					mark.points[0].x = ep1.getx();
					mark.points[0].y = ep1.gety();
				}
				if(add2){
					if(add1) mark.points.resize(3);
					mark.points[add1 ? 2 : 0].x = ep2.getx();
					mark.points[add1 ? 2 : 0].y = ep2.gety();
				}
				accountedForIDs[id] = true;
				mark.id = id*100; //i + pts.size() + 1;
				marks.markers.push_back(mark);	
				//sleep(1);  		// to watch the map build gradually
				rvizMap.publish(marks);	
			}
		}	
	}
	auto end = std::chrono::steady_clock::now();
	std::cout<<"time to draw: "<<std::chrono::duration_cast
		<std::chrono::milliseconds>(end-start).count()<<"\n";

	std::cout<<"size of marker array is: "<<marks.markers.size()<<"\n";

	rvizMap.publish(marks);	
	rvizMap.publish(marks);	
	rvizMap.publish(marks);	
	rvizMap.publish(marks);

	struct timespec req = {0};
	if(runBool){
		int milli = 40;
		req.tv_sec = 0;
		req.tv_nsec = milli * 1000000L;
		nanosleep(&req, (struct timespec *)NULL);
	}
	else{ sleep(3);}
	rvizMap.publish(marks);	
	rvizMap.publish(marks);	
	rvizMap.publish(marks);	
	
	if(runBool){nanosleep(&req, (struct timespec *)NULL);}
	else{ sleep(3);}

	rvizMap.publish(marks);	
	rvizMap.publish(marks);	
	rvizMap.publish(marks);	
	rvizMap.publish(marks);	
}

// prints the map point info to console
void Nav::outputGraph(vector<EndPoint> pts){
	EndPoint ep;
	std::cout<<"outputGraph with size: "<<pts.size()<<"\n";
	std::cout<<"////////////////////////////////////////////////////\n";
	for(int i=0; i<pts.size(); i++){
		std::cout<<" id: "<<pts[i].getID();
		std::cout<<"\tx: "<<pts[i].getx();
		std::cout<<"\ty: "<<pts[i].gety();
		std::cout<<"\tNEIGH: "<< pts[i].getNumNeighbors();
		for(int k=0; k<pts[i].getNumNeighbors(); k++){
			// for some reason this check needs to be here...
			bool check = getNeighbor(pts[i].getID(), k, ep, pts);
			if(check){
				std::cout<<"\tn: "<< ep.getID();
			}
			else{
				std::cout<<"\tn: N";
			}
		}
		std::cout<<"\n";
	}
	std::cout<<"////////////////////////////////////////////////////\n";
}

void Nav::setRun(bool t){ runBool = t;}
vector<EndPoint>* Nav::getMap() {return &mapPoints;}
vector<EndPoint>* Nav::getWays(){return &wayPoints;}
