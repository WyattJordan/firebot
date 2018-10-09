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

Nav::Nav(){}

EndPoint& Nav::getPoint(int id){
	if(mapPoints[id].getID() == id) return mapPoints[id];
	for(int i=0; i<mapPoints.size(); i++){
		if(mapPoints[i].getID() == id) return mapPoints[i];
	}
	return badPt;
}

Nav::Nav(string file){  // this is the main constructor, initialize variables here
	smallRoomConf = bigRoomConf = false;
	vector<int> none;
	badPt = EndPoint(-1, -1, -1, none);

	std::ifstream mapFile;
        mapFile.open(file.c_str());
        string line = "";

        vector<string> nums;
	std::cout<<"opening file..."<<file<<"\n";
        if (mapFile.is_open()){
                while(getline(mapFile, line, ',')){
                        //std::cout<<line<<"\n";
                        nums.push_back(line);
                }
                mapFile.close();
        }
	else{ ROS_ERROR("WARNING: map file not found!\n"); }

	while(nums.size()>=7){
		vector<int> temp;
		// nums[3] and nums[4] are definite and potential corner types, ignored here	
		// neighbors saved via next two lines
		if(nums[5].compare("x") != 0) temp.push_back(std::atof(nums[5].c_str())); 
		if(nums[6].compare("x") != 0) temp.push_back(std::atof(nums[6].c_str())); 
		EndPoint tmp(std::atof(nums[0].c_str()), std::atof(nums[1].c_str()),
		    std::atof(nums[2].c_str()),temp);
		mapPoints.push_back(tmp);
		for(int i=0; i<7; i++) nums.erase(nums.begin()+0);
	}
	std::cout<<"size of mapPoints after constructor is: "<<mapPoints.size()<<"\n";
}
void Nav::removePoint(int id){
	for(int i=0; i<mapPoints.size(); i++){
		if(mapPoints[i].getID() == id){
			mapPoints.erase(mapPoints.begin() + i);
			break;
		}
	}
}

void Nav::setSmallRoomUpper(bool up){
	if(up){
		getPoint(18).setNeighbors(12,-1);		
		getPoint(13).setNeighbors(14,-1);
		getPoint(11).setNeighbors(12,14);
		getPoint(14).setNeighbors(13,11);
		removePoint(19);
	}
	else{
		getPoint(11).setNeighbors(12,-1);
		getPoint(19).setNeighbors(14,-1);
		getPoint(12).setNeighbors(11,13);
		getPoint(13).setNeighbors(12,14);
		removePoint(18);
	}
}
void Nav::setBigRoomUpper(bool up){
	if(up){
		getPoint(17).setNeighbors(20,-1);
		getPoint(16).setNeighbors(15,-1);
	}
	else{
		getPoint(17).setNeighbors(16,-1);
		removePoint(20);
	}	
}

void Nav::findExpected(float Rx, float Ry){
	for(EndPoint &ep : mapPoints){
	       	ep.getPolar(Rx, Ry); // calculate all polars
		ep.setDone(false);
	}
	std::sort(mapPoints.begin(), mapPoints.end(), 
			[](const EndPoint& lhs, const EndPoint &rhs){
			return	lhs.getR() < rhs.getR();	
		}); // sort by points closest to robot
	//std::cout<<"after sorting by polar:\n";
	//outputMapPoints();
	EndPoint ep;
	int index = 0;
	bool go = true;
	while(go){
		mapPoints[index].setVisible(true);
		mapPoints[index].setDone(true);
		// this will only ever loop once or twice
		for(int i=0; i<mapPoints[index].getNumNeighbors(); i++){ 
			getNeighbor(mapPoints[index].getID(), i, ep);
			eliminatePts(ep, mapPoints[index], Rx, Ry);
		}

		for(EndPoint &p : mapPoints){
			if(!p.getDone()){
				go = true;
				break;
			}
			else{ go = false;}
		}	

		while(index<mapPoints.size() &&  mapPoints[index].getDone()){ index++; }
	//	std::cout<<"index is: "<<index<<"\n";
	}
}

void Nav::eliminatePts(EndPoint &ep1,EndPoint &ep2, float Rx, float Ry){
	//std::cout<<"elim from "<<ep1.getID()<<" to "<<ep2.getID()<<"\n";
	for(EndPoint &p : mapPoints){
		if(!p.getDone()){

		if(p.getR()<15 || p.getR() > 180){
			p.setVisible(false);
			p.setDone(true);
		}
		else{
			float theta = p.getTheta();
			bool vert = ep1.getx() == ep2.getx() ? 1 : 0; // otherwise shld be horiz line	
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

bool Nav::getNeighbor(int startID, int neighI, EndPoint &neigh){ 
	EndPoint start = getPoint(startID);
	if((start.getx() == -1 && start.gety() == -1)
			|| start.getNumNeighbors() - 1 < neighI){
		neigh = badPt;
		return false;
	}
	else {
		neigh = getPoint(start.getNeighborID(neighI));	
		return true;
	}	
}

void Nav::run(){
	float x,y;
	y = 30;
	unsigned int sleep = 10000;
	float iter = 100;
	for( int i = 0; i<iter; i++){
		x = 230.0/iter * i; publishMap(x,y);
	}	
	x = 230;
	for( int i = 0; i<iter; i++){
		y = 230.0/iter * i; publishMap(x,y);
	}	
	y = 230;
	for( int i = 0; i<iter; i++){
		x = 230.0/iter * (iter - i); publishMap(x,y);
	}	
	x = 30;
	for( int i = 0; i<iter; i++){
		y = 230.0/iter * (iter - i); publishMap(x,y);
	}	
}
void Nav::publishMap(float Rx, float Ry){
	auto start = std::chrono::steady_clock::now();

	findExpected(Rx, Ry);
	string worldFrame = "map2";
	ros::NodeHandle n;
	ros::Publisher rvizMap = n.advertise<visualization_msgs::MarkerArray>("map",1000);
	visualization_msgs::MarkerArray marks;
	marks.markers.resize(2*mapPoints.size());
	// add vertical arrows at walls corners and endpoints
	for(int i=0; i<mapPoints.size(); i++){
		marks.markers[i].header.frame_id = worldFrame;
		marks.markers[i].ns = "vertical markers";
		marks.markers[i].id = i; //mapPoints[i].getID();
		marks.markers[i].type = visualization_msgs::Marker::ARROW;
		marks.markers[i].action = visualization_msgs::Marker::ADD;
		
		marks.markers[i].points.resize(2);
		marks.markers[i].points[0].x = mapPoints[i].getx();
		marks.markers[i].points[0].y = mapPoints[i].gety();
		marks.markers[i].points[0].z = 0;
		marks.markers[i].points[1].x = mapPoints[i].getx();
		marks.markers[i].points[1].y = mapPoints[i].gety();
		marks.markers[i].points[1].z = 10; // 10cm tall 

		marks.markers[i].scale.x = 3; 
		marks.markers[i].scale.y = 3;
		marks.markers[i].scale.z = 3;
		//bool visible = std::find(expectedIDs.begin(), expectedIDs.end(), 
				//mapPoints[i].getID()) != expectedIDs.end();
		marks.markers[i].color.a = 1.0;	
		marks.markers[i].color.r = 0;	
		marks.markers[i].color.g = mapPoints[i].isVisible() ? 1.0 : 0;
		marks.markers[i].color.b = mapPoints[i].isVisible() ? 0 : 1.0;	

		// show text ID above marker with id += 1000
		int text = i+mapPoints.size();
		marks.markers[text].header.frame_id = worldFrame;
		marks.markers[text].id = i+1000;
		marks.markers[text].ns = "text";
		marks.markers[text].text = std::to_string(mapPoints[i].getID());
		marks.markers[text].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marks.markers[text].action = visualization_msgs::Marker::ADD;
		marks.markers[text].scale.x = 3;
		marks.markers[text].scale.y = 3;
		marks.markers[text].scale.z = 8.0; // text height	
		marks.markers[text].pose.position.x = mapPoints[i].getx();
		marks.markers[text].pose.position.y = mapPoints[i].gety();
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
	robot.ns = "robot";
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
	vector<bool> accountedForIDs(mapPoints.size()+10, false); // index is ID
	visualization_msgs::Marker mark;
	mark.type   = visualization_msgs::Marker::LINE_STRIP;
	mark.header.frame_id = worldFrame;
	mark.ns = "floor lines";
	mark.color.a = 1;
	mark.color.r = 1;
	mark.color.g = 0;
	mark.color.b = 0;
	mark.scale.x = 3;
	mark.points.resize(2);
	mark.points[0].z = 0;
	mark.points[1].z = 0;

	EndPoint ep1, ep2;
	int id;		
	
	for(int i=0; i<mapPoints.size(); i++){ 	// loop thru points to find connections
		id = mapPoints[i].getID();
		if(!accountedForIDs[id]){
			bool add1 = getNeighbor(mapPoints[i].getID(), 0, ep1) &&
			       	!accountedForIDs[ep1.getID()];
			bool add2 = getNeighbor(mapPoints[i].getID(), 1, ep2) && 
				!accountedForIDs[ep2.getID()];

			if( add1 || add2 ){
				mark.points.resize(2);
				mark.points[1].x = mapPoints[i].getx();
				mark.points[1].y = mapPoints[i].gety();
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
				mark.id = i + mapPoints.size() + 1;
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

void Nav::outputMapPoints(){
	EndPoint ep;
	for(int i=0; i<mapPoints.size(); i++){
		std::cout<<" id: "<<mapPoints[i].getID();
		std::cout<<"\tx: "<<mapPoints[i].getx();
		std::cout<<"\ty: "<<mapPoints[i].gety();
		for(int k=0; k<mapPoints[i].getNumNeighbors(); k++){
			bool check = getNeighbor(mapPoints[i].getID(), k, ep);
			if(check){
				std::cout<<"\tn: "<< ep.getID();
			}
			else{
				std::cout<<"\tn: N";
			}
		}
		std::cout<<"\n";
	}

}

void Nav::setRun(bool t){ runBool = t;}
int Nav::getSize(){return mapPoints.size();}
