/* nav.cpp
 *
 *
 */
#include "nav.h"
#include "endpoint.h"
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
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

Nav::Nav(string file){
	room1Conf = room4Conf = false;
	
	std::ifstream mapFile;
        mapFile.open(file.c_str());
        string line = "";

        vector<string> nums;
	std::cout<<"opening file..."<<file<<"\n";
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
		/*std::cout<<"x: "<<std::atof(nums[0].c_str());
		std::cout<<"y: "<<std::atof(nums[1].c_str());
		std::cout<<"ID: "<<std::atof(nums[2].c_str());
*/
		EndPoint tmp(std::atof(nums[0].c_str()), std::atof(nums[1].c_str()),
		    std::atof(nums[2].c_str()),temp);
		mapPoints.push_back(tmp);
		for(int i=0; i<7; i++) nums.erase(nums.begin()+0);
	}
	std::cout<<"size of mapPoints after constructor is: "<<mapPoints.size()<<"\n";
}
int Nav::getSize(){return mapPoints.size();}

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

void Nav::publishMap(){
	ros::NodeHandle n;
	ros::Publisher rvizMap = n.advertise<visualization_msgs::MarkerArray>("map",1000);
	visualization_msgs::MarkerArray marks;
	marks.markers.resize(mapPoints.size());

	// add vertical arrows at walls
	for(int i=0; i<mapPoints.size(); i++){
//		EndPoint tmp = mapPoints[i];1
		marks.markers[i].header.frame_id = "map2";
		marks.markers[i].ns = "vertical markers";
		marks.markers[i].id = i;//mapPoints[i].getID();
		marks.markers[i].type = visualization_msgs::Marker::ARROW;
		marks.markers[i].action = visualization_msgs::Marker::ADD;
		marks.markers[i].points.resize(2);
		marks.markers[i].points[0].x = mapPoints[i].getx();
		marks.markers[i].points[0].y = mapPoints[i].gety();
		marks.markers[i].points[0].z = 0;
		marks.markers[i].points[1].x = mapPoints[i].getx();
		marks.markers[i].points[1].y = mapPoints[i].gety();
		marks.markers[i].points[1].z = 10; // 10cm tall 

		marks.markers[i].scale.x = 2; 
		marks.markers[i].scale.y = 2;
		marks.markers[i].scale.z = 3;
		/*k
		marks.markers[i].pose.position.x = mapPoints[i].getx();
		marks.markers[i].pose.position.y = mapPoints[i].gety();
		marks.markers[i].pose.position.z = 0;
		marks.markers[i].pose.orientation.x = 0;
		marks.markers[i].pose.orientation.y = 0;
		marks.markers[i].pose.orientation.z = 0;
		marks.markers[i].pose.orientation.w = 1;
		*/
		marks.markers[i].color.a = 1.0;	
		marks.markers[i].color.r = 0;	
		marks.markers[i].color.g = 1.0;	
		marks.markers[i].color.b = 0;
	}

	// add horizontal lines on floor
	vector<bool> accountedForIDs(mapPoints.size()+10, false); // index is ID
	visualization_msgs::Marker mark;
	mark.header.frame_id = "map2";
	mark.action = visualization_msgs::Marker::ADD;
	mark.type   = visualization_msgs::Marker::LINE_STRIP;
	//mark.ns = "floor lines";
	mark.color.a = 1;
	mark.color.r = 1;
	mark.color.g = 0;
	mark.color.b = 0;
	mark.scale.x = 3;
	mark.points.resize(2);
	mark.points[0].z = 0;
	mark.points[1].z = 0;

	geometry_msgs::Point pt;
	pt.z = 0;
	EndPoint ep1, ep2;
	std::cout<<"should be looping for size = " << mapPoints.size()<<"\n";

	int id;		
	for(int i=0; i<mapPoints.size(); i++){
		id = mapPoints[i].getID();
		if(!accountedForIDs[id]){
			mark.points.resize(2);
			bool add = false;
			bool found1 =  getNeighbor(mapPoints[i].getID(), 1, ep1);
			bool found2 =  getNeighbor(mapPoints[i].getID(), 2, ep2);
			if(found1 && !accountedForIDs[ep1.getID()]){
				mark.points[0].x = ep1.getx();
				mark.points[0].y = ep1.gety();
				add = true;
				std::cout<<"added pt 1\n";		
			}

			mark.points[1].x = mapPoints[i].getx();
			mark.points[1].y = mapPoints[i].gety();

			if(found2 && !accountedForIDs[ep2.getID()]){
				mark.points.resize(3);
				mark.points[2].x = ep2.getx();
				mark.points[2].y = ep2.gety();
				mark.points[2].z = 0; 
				add = true;
				std::cout<<"added a thing\n";		
	
			}	

			accountedForIDs[id] = true;
			mark.id = id;
			if(add) marks.markers.push_back(mark);	
			std::cout<<"pt "<<mapPoints[i].getID()<<" is connected to : "<<
				ep1.getID()<<" and : "<<
				ep2.getID()<<"\n";
		}
	}	
	std::cout<<"size of marker array is: "<<marks.markers.size()<<"\n";
	while(1){
		rvizMap.publish(marks);	
	}
}


