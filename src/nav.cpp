/* nav.cpp
 *
 *
 */
#include "nav.h"
#include "endpoint.h"
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
using std::vector;
using std::atof;


/*void Nav::loadMap(string file){
	Map tmp(file);
	universalMap = tmp;
}

Map* Nav::getMap(){ 
	return &universalMap;
	
}*/
Nav::Nav(){}

void Nav::publishMap(){
	ros::NodeHandle n;
	ros::Publisher rvizMap = n.advertise<visualization_msgs::MarkerArray>("map",1000);
	visualization_msgs::Marker mark;
	visualization_msgs::MarkerArray marks;
	marks.markers.resize(mapPoints.size());
	for(int i=0; i<mapPoints.size(); i++){
		EndPoint tmp = mapPoints[i];
		marks.markers[i].header.frame_id = "map2";
		marks.markers[i].ns = "my_namespace";
		marks.markers[i].id = tmp.getID();
		marks.markers[i].type = visualization_msgs::Marker::ARROW;
		marks.markers[i].action = visualization_msgs::Marker::ADD;
		marks.markers[i].scale.x = 10; // 10cm tall	
		marks.markers[i].scale.y = 1;
		marks.markers[i].scale.z = 1;
		marks.markers[i].pose.position.x = tmp.getx();
		marks.markers[i].pose.position.y = tmp.gety();
		marks.markers[i].pose.position.z = 0;
		marks.markers[i].pose.orientation.x = 0;
		marks.markers[i].pose.orientation.y = 0;
		marks.markers[i].pose.orientation.z = 1;
		marks.markers[i].pose.orientation.w = 0;
		marks.markers[i].color.a = 1.0;	
		marks.markers[i].color.r = 0;	
		marks.markers[i].color.g = 1.0;	
		marks.markers[i].color.b = 0;
	}
	std::cout<<"size of marker array is: "<<marks.markers.size()<<"\n";
	while(1){
		rvizMap.publish(marks);	
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

Nav::Nav(string file){
	room1Conf = room4Conf = false;
	
	std::ifstream mapFile;
        mapFile.open(file.c_str());
        string line = "";

        vector<string> nums;
        if (mapFile.is_open()){
                while(getline(mapFile, line, ',')){
                        //std::cout<<line<<"\n";
                        nums.push_back(line);
                }
                mapFile.close();
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
	std::cout<<"size of mapPoints is: "<<mapPoints.size()<<"\n";
}
int Nav::getSize(){return mapPoints.size();}

bool Nav::getNeighbor(int startID, int neighNum, EndPoint &neigh){ 
	EndPoint start = getPoint(startID);
	if((start.getx() == -1 && start.gety() == -1)
			|| start.getNumNeighbors() < neighNum){
		neigh = start;
		return false;
	}
	else {
		neigh = getPoint(start.getNeighborID(neighNum));	
		return true;
	}	
}


