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
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <pthread.h>
#include <algorithm>
using std::vector;
using std::atof;

Nav::Nav(){}

void Nav::outputMapPoints(){
	for(int i=0; i<mapPoints.size(); i++){
		std::cout<<"id: "<<mapPoints[i].getID();
		std::cout<<"   x: "<<mapPoints[i].getx();
		std::cout<<"   y: "<<mapPoints[i].gety();
		std::cout<<"\n";
	}

}

EndPoint Nav::getBadPoint(){
	vector<int> none;
	EndPoint notfound(-1, -1, -1, none);
	return notfound;	
}

EndPoint Nav::getPoint(int id){
	if(mapPoints[id].getID() == id) return mapPoints[id];
	for(int i=0; i<mapPoints.size(); i++){
		if(mapPoints[i].getID() == id) return mapPoints[i];
	}
	return getBadPoint();
}

Nav::Nav(string file){  // this is the main constructor, initialize variables here
	room1Conf = room4Conf = false;
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
	else{
		ROS_ERROR("WARNING: map file not found!\n");
	}

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
int Nav::getSize(){return mapPoints.size();}

// comparison function to std::sort the mapPoints by closest polar radius from robot
struct compareRLess{
	bool operator()(const EndPoint &l, const EndPoint &r){
	return l.getR()>r.getR();
	}
};

void Nav::findExpected(float Rx, float Ry, float theta){
	for(EndPoint &ep : mapPoints){
	       	ep.getPolar(Rx, Ry, theta); // calculate all polars
		ep.setDone(false);
	}
	std::sort(mapPoints.begin(), mapPoints.end(), [](const EndPoint& lhs, const EndPoint &rhs){
			return	lhs.getR() < rhs.getR();	
		}); // sort by points closest to robot

	EndPoint ep;
	int index = 0;
	bool go = true;
	while(go){
		mapPoints[index].setVisible(true);
		mapPoints[index].setDone(true);
		// this will only ever loop once or twice
		for(int i=0; i<mapPoints[index].getNumNeighbors(); i++){ 
			getNeighbor(mapPoints[index].getID(), i, ep); 
			if( !ep.getDone() ){
				eliminatePts(ep, mapPoints[index], Rx, Ry);
			}
		}

		for(EndPoint &p : mapPoints){
			if(!p.getDone()){
				go = true;
				break;
			}
			else{ go = false;}
		}	

		while(index<mapPoints.size() &&  mapPoints[index].getDone()) index++;
		std::cout<<"index is: "<<index<<"\n";
	}
}
void Nav::eliminatePts(EndPoint &ep1,EndPoint &ep2, float Rx, float Ry){
	float m = ((ep1.gety() - Ry) - (ep2.gety() - Ry)) / ((ep1.getx() - Rx) - (ep2.getx() - Rx));
	float b = m*(ep1.getx() - Rx) + ep1.gety() - Ry;

	/*float start = ep1.getTheta();
	float width = start - ep2.getTheta();
	if(width > 180) width = 
	if(t1 - t2 > 180) t2 -= 360;	
*/
	for(EndPoint &p : mapPoints){
		if(!p.getDone()){
			//if(p.getTheta() < ep1.getTheta() &&
			if((b<0 && p.gety() < m*p.getx() + b) || (b>0 && p.gety() > m*p.getx() + b)){
				p.setVisible(false);
				p.setDone(true);
			}	
			else{
				p.setVisible(true);
			}
		}
	}
}
bool Nav::getNeighbor(int startID, int neighNum, EndPoint &neigh){ 
	EndPoint start = getPoint(startID);
	if((start.getx() == -1 && start.gety() == -1)
			|| start.getNumNeighbors() < neighNum){
		neigh = getBadPoint();
		return false;
	}
	else {
		neigh = getPoint(start.getNeighborID(neighNum));	
		return true;
	}	
}

void Nav::publishMap(float Rx, float Ry, float theta){
	//rviz_visual_tools::RvizVisualTools::deleteAllMarkers();	
	findExpected(Rx, Ry, theta);
	string worldFrame = "map2";
	ros::NodeHandle n;
	ros::Publisher rvizMap = n.advertise<visualization_msgs::MarkerArray>("map",1000);
	visualization_msgs::MarkerArray marks;
	marks.markers.resize(mapPoints.size());
	// add vertical arrows at walls corners and endpoints
	visualization_msgs::Marker robot;
	for(int i=0; i<mapPoints.size(); i++){
		marks.markers[i].header.frame_id = worldFrame;
		marks.markers[i].ns = "vertical markers";
		marks.markers[i].id = mapPoints[i].getID();
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
		marks.markers[i].color.g = mapPoints[i].isVisible() ? 0 : 1.0;	
		marks.markers[i].color.b = mapPoints[i].isVisible() ? 1.0 : 0;
		rvizMap.publish(marks);
		//sleep(1);
	}

	// add robot sphere
	robot = marks.markers[0];
	robot.ns = "robot";
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
	std::cout<<"should be looping for size = " << mapPoints.size()<<"\n";

	int id;		
	for(int i=0; i<mapPoints.size(); i++){ 	// loop thru points to find connections
		id = mapPoints[i].getID();
		if(!accountedForIDs[id]){
			bool add1 = getNeighbor(mapPoints[i].getID(), 1, ep1) &&
			       	!accountedForIDs[ep1.getID()];
			bool add2 = getNeighbor(mapPoints[i].getID(), 2, ep2) && 
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
				mark.id = id;
				marks.markers.push_back(mark);	
				//sleep(1);  		// to watch the map build gradually
				rvizMap.publish(marks);	
			}
		}	
	/*	std::cout<<"pt "<<mapPoints[i].getID()<<" is connected to : "<<
				ep1.getID()<<" and : "<<
				ep2.getID()<<"\n";
	*/
	}
	std::cout<<"size of marker array is: "<<marks.markers.size()<<"\n";
	while(1){
		rvizMap.publish(marks);	
	}
}


