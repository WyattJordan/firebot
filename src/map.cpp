#include "map.h"
#include "endpoint.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
using std::vector;
using std::atof;

Map::Map(){
	// used to get a bad point
}

void Map::publishMap(){
	ros::NodeHandle n;
	ros::Publisher rvizMap = n.advertise<visualization_msgs::MarkerArray>("map",1000);
	visualization_msgs::Marker mark;
	visualization_msgs::MarkerArray marks;
	for(int i=0; i<mapPoints.size(); i++){
		EndPoint tmp = mapPoints[i];
		mark.header.frame_id = "base_link";
		mark.ns = "my_namespace";
		mark.id = tmp.getID();
		mark.type = visualization_msgs::Marker::ARROW;
		mark.action = visualization_msgs::Marker::ADD;
		mark.scale.x = 10; // 10cm tall	
		mark.scale.y = 1;
		mark.scale.z = 1;
		mark.pose.position.x = tmp.getx();
		mark.pose.position.y = tmp.gety();
		mark.pose.position.z = 0;
		mark.pose.orientation.x = 0;
		mark.pose.orientation.y = 0;
		mark.pose.orientation.z = 1;
		mark.pose.orientation.w = 0;
		mark.color.a = 1.0;	
		mark.color.r = 0;	
		mark.color.g = 1.0;	
		mark.color.b = 0;
		marks.markers.push_back(mark);	
	}
	while(1){
		rvizMap.publish(marks);	
	}
}
EndPoint Map::getBadPoint(){
	vector<int> none;
	EndPoint notfound(this, -1, -1, -1, none);
	return notfound;	
}
EndPoint Map::getPoint(int id){
	if(mapPoints[id].getID() == id) return mapPoints[id];
	for(int i=0; i<mapPoints.size(); i++){
		if(mapPoints[i].getID() == id) return mapPoints[i];
	}
	return getBadPoint();
}

Map::Map(string file){
	
	std::ifstream mapFile;
        mapFile.open(file.c_str());
        string line = "";

        vector<string> nums;
        if (mapFile.is_open()){
                while(getline(mapFile, line, ',')){
                        std::cout<<line<<"\n";
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
		EndPoint tmp(this, std::atof(nums[0].c_str()), std::atof(nums[1].c_str()),
		    std::atof(nums[2].c_str()),temp);
		mapPoints.push_back(tmp);
		for(int i=0; i<7; i++) nums.erase(nums.begin()+0);
	}
}

void Map::printCode(int code){
	std::cout<<"the map code is: "<<code<<" \n";
}




