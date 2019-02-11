/* nav.cpp
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

#define LARGENUM 99999999
// don't use default constuctor, it must load the map and way files
Nav::Nav(){}

// This is the main constructor, initialize variables here
Nav::Nav(int lvl){  
	string mapfile, wayfile;
	if(lvl == 3){
		mapfile = "";
	    wayfile = "";  //root is catkin ws
	}	
	else {
		mapfile = "/home/wyatt/cat_ws/src/firebot/lvl1_map.txt"; 
		wayfile = "/home/wyatt/cat_ws/src/firebot/wayPoints.txt";	
	}

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

// Sets the door configuration for the small room, if up == true the 
// door is on the higher side (larger y coordinate) 
void Nav::setSmallRoomUpper(bool up){
	std::cout<<"setting small room upper\n";	
	if(up){
		getPoint(18,mapPoints).setNeighbors(1,12);		
		getPoint(13,mapPoints).setNeighbors(1,14);
		getPoint(11,mapPoints).setNeighbors(2,12,14);
		getPoint(14,mapPoints).setNeighbors(2,13,11);
		removePoint(19,mapPoints);

		getPoint(12,wayPoints).setNeighbors(1,11);
		getPoint(13,wayPoints).setNeighbors(2,5,15);
		}
	else{
		getPoint(11,mapPoints).setNeighbors(1,12);
		getPoint(19,mapPoints).setNeighbors(1,14);
		getPoint(12,mapPoints).setNeighbors(2,11,13);
		getPoint(13,mapPoints).setNeighbors(2,12,14);
		removePoint(18,mapPoints);

		getPoint(11,wayPoints).setNeighbors(1,12);
		getPoint(10,wayPoints).setNeighbors(2,9,14);
	}
}

// Sets the door configuration for the larger room, if up == true the door is
// in the higher location (larger y coordinate)
void Nav::setBigRoomUpper(bool up){
	std::cout<<"setting big room upper\n";	
	if(up){
		getPoint(17,mapPoints).setNeighbors(1,20);
		getPoint(16,mapPoints).setNeighbors(1,15);

		getPoint(3,wayPoints).setNeighbors(2,2,4);
		removePoint(17,wayPoints);
	}
	else{
		getPoint(17,mapPoints).setNeighbors(1,16);
		removePoint(20,mapPoints);

		getPoint(4,wayPoints).setNeighbors(2,3,5);
		getPoint(16,wayPoints).setNeighbors(2,15,17);
	}	
}

// Determines which map points are visible to the robot given 
// the robot's location on the map
void Nav::findExpected(float Rx, float Ry, vector<EndPoint> &pts){
	for(EndPoint &ep : pts){
		ep.getPolar(Rx, Ry); // calculate all polars
		ep.setDone(false);
	}
	std::sort(pts.begin(), pts.end(), 
			[](const EndPoint& lhs, const EndPoint &rhs){
			return	lhs.getR() < rhs.getR();	
		}); // sort by points closest to robot

	EndPoint ep;
	int index = 0;
	bool go = true;
	while(go){
		pts[index].setVisible(true);
		pts[index].setDone(true);
		// this will only ever loop once or twice since all corners are just 2 walls
		for(int i=0; i<pts[index].getNumNeighbors(); i++){ 
			getNeighbor(pts[index].getID(), i, ep, pts);
			eliminatePts(ep, pts[index], Rx, Ry, pts); // removes points hidden by the wall
		}

		go = false;
		for(EndPoint &p : pts){
			if(!p.getDone()){
				go = true;
				break;
			}
		}	

		while(index<pts.size() &&  pts[index].getDone()){ index++; } // find next point to check
	}
}

// Given endpoints for a wall and robot location, sets all points "visible" and "done" 
// values accordingly.
void Nav::eliminatePts(EndPoint &ep1,EndPoint &ep2, float Rx, float Ry, vector<EndPoint> &pts){
	//std::cout<<"elim from "<<ep1.getID()<<" to "<<ep2.getID()<<"\n";
	for(EndPoint &p : pts){
		if(!p.getDone()){

			if(p.getR()<15 || p.getR() > 130){ // must be within LIDAR range
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
				}
				else{ // if not blocked
					p.setVisible(true);
				}
			}
		}
	}
}

// Given a point ID and a neighbor index returns true if a neighbor exists and that neigh
// is in pts<> and copies that neighbor to the reference neigh
bool Nav::getNeighbor(int startID, int neighI, EndPoint &neigh, vector<EndPoint> &pts){ 
	EndPoint start = getPoint(startID, pts);
	if(start.getID()!=-1){ 
		neigh = getPoint(start.getNeighborID(neighI), pts);
		if(neigh.getID()==-1){
			neigh = badPt;
			return false;
		}
		return true;
	}
	else{
		neigh = badPt;
		return false;
	}
}

// Move the robot around the map and publish what mapPoints it can see
void Nav::run(){
	color mapLine = {1.0, 0.5, 0.5};
	color mapMark = {1.0, 1.0, 1.0};
	float x,y;
	y = 30;
	unsigned int sleep = 10000;
	float iter = 100;
	for( int i = 0; i<iter; i++){
		x = 230.0/iter * i; publishGraph(x,y, "test", 
				mapPoints, mapLine, mapMark);
	}	
	x = 230;
	for( int i = 0; i<iter; i++){
		y = 230.0/iter * i; publishGraph(x,y, "test", 
				mapPoints, mapLine, mapMark);
	}	
	y = 230;
	for( int i = 0; i<iter; i++){
		x = 230.0/iter * (iter - i); publishGraph(x,y, "test", 
				mapPoints, mapLine, mapMark);
	}	
	x = 30;
	for( int i = 0; i<iter; i++){
		y = 230.0/iter * (iter - i); publishGraph(x,y, "test", 
				mapPoints, mapLine, mapMark);
	}	
	
}

void Nav::publishMapAndWays(float Rx, float Ry){
	//findExpected(Rx, Ry, mapPoints);
	color mapLine = {0.9, 0.5, 0.0};
	color mapMark = {1.0, 0.1, 0.1};
	color wayLine = {0.1, 0.9, 0.9};
	color wayMark = {0.8, 0.1, 0.9};


	publishGraph(Rx, Ry, "map", mapPoints, mapLine, mapMark);
	publishGraph(Rx, Ry, "way", wayPoints, wayLine, wayMark);
		
}
void Nav::populatMarks(string which, string NS, string frame){
	visualization_msgs::MarkerArray tmp;
	vector<EndPoint> *pts;
	if(which == "map") { pts = &mapPoints; }
	if(which == "way") { pts = &wayPoints; }

	// = map ? &mapPoints : &wayPoints;
	tmp.markers.resize(2*pts->size());
	

	//if(map) {mapMarks = tmp;}
	//else    {wayMarks = tmp;}
}
visualization_msgs::MarkerArray Nav::makeMarks(vector<EndPoint> &pts, color lncol, color markcol,
		string NS, string frame){
	visualization_msgs::MarkerArray marks;
	marks.markers.resize(2*pts.size());
	// add vertical arrows at pt locations
	for(int i=0; i<pts.size(); i++){
		marks.markers[i].header.frame_id = frame;
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
		marks.markers[i].color.r = pts[i].isVisible() ? 0.0 : markcol.r;	
		marks.markers[i].color.g = pts[i].isVisible() ? 1.0 : markcol.g;
		marks.markers[i].color.b = pts[i].isVisible() ? 0.0 : markcol.b;	

		// show text ID above marker with id += 1000
		int idx = i+pts.size();
		marks.markers[idx].header.frame_id = frame;
		marks.markers[idx].id = i+1000;
		marks.markers[idx].ns = NS;
		marks.markers[idx].text = NS=="way" ? std::to_string(pts[i].getID()) : "";
		marks.markers[idx].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marks.markers[idx].action = visualization_msgs::Marker::ADD;
		marks.markers[idx].scale.x = 3;
		marks.markers[idx].scale.y = 3;
		marks.markers[idx].scale.z = 8.0; // text height	
		marks.markers[idx].pose.position.x = pts[i].getx();
		marks.markers[idx].pose.position.y = pts[i].gety();
		marks.markers[idx].pose.position.z = 15;
		marks.markers[idx].pose.orientation.x = 0;
		marks.markers[idx].pose.orientation.y = 0;
		marks.markers[idx].pose.orientation.z = 0;
		marks.markers[idx].pose.orientation.w = 1;
		marks.markers[idx].color.a = 1.0;  // white
		marks.markers[idx].color.r = 1; 
		marks.markers[idx].color.g = 1; 
		marks.markers[idx].color.b = 1; 
		//sleep(1); rvizMap.publish(marks); //uncomment to see point growth from closest to robot
	}

	// Now add the lines 
	visualization_msgs::Marker mark;
	mark.type = visualization_msgs::Marker::LINE_STRIP;
	mark.header.frame_id = frame;
	mark.ns = NS;
	mark.color.a = 1;
	mark.color.r = lncol.r; 
	mark.color.g = lncol.g;
	mark.color.b = lncol.b;
	mark.scale.x = 3;
	mark.points.resize(2);
	mark.points[0].z = 0;
	mark.points[1].z = 0;

	EndPoint ep1;
	vector<bool> accountedForIDs(pts.size()*10, false); // index is ID
	int id;		

	// LINE_STRIP is defined by n points, here always n = 2
	for(int i=0; i<pts.size(); i++){ 
		mark.points[0].x = pts[i].getx(); // add the ith point to the line
		mark.points[0].y = pts[i].gety();

		// loop through all the ith point's neighbors adding 	
		// and then the main MarkerArray them to the LINE_STRIP
		for(int k=0; k<pts[i].getNumNeighbors(); k++){
			
			if(getNeighbor(pts[i].getID(), k, ep1, pts) &&
				       	!accountedForIDs[ep1.getID()]){
				mark.points[1].x = ep1.getx();
				mark.points[1].y = ep1.gety();
				mark.id = pts[i].getID() * 10000 + k; 
				marks.markers.push_back(mark);	
			}
		}		
		accountedForIDs[pts[i].getID()] = true;
	}

}


void Nav::publishGraph(float Rx, float Ry, string NS,
	   	vector<EndPoint> &pts, color lncol, color  markcol){
	auto start = std::chrono::steady_clock::now();
	string worldFrame = "map2";
	ros::NodeHandle n;
	ros::Publisher rvizMap = n.advertise<visualization_msgs::MarkerArray>("map",1000);
	visualization_msgs::MarkerArray marks;
	marks.markers.resize(2*pts.size());
	// add vertical arrows at pt locations
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
		marks.markers[i].color.r = pts[i].isVisible() ? 0.0 : markcol.r;	
		marks.markers[i].color.g = pts[i].isVisible() ? 1.0 : markcol.g;
		marks.markers[i].color.b = pts[i].isVisible() ? 0.0 : markcol.b;	

		// show text ID above marker with id += 1000
		int text = i+pts.size();
		marks.markers[text].header.frame_id = worldFrame;
		marks.markers[text].id = i+1000;
		marks.markers[text].ns = NS;
		marks.markers[text].text = NS=="way" ? std::to_string(pts[i].getID()) : "";
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
	mark.color.r = lncol.r;
	mark.color.g = lncol.g;
	mark.color.b = lncol.b;
	mark.scale.x = 3;
	mark.points.resize(2);
	mark.points[0].z = 0;

	mark.points[1].z = 0;

	EndPoint ep1;
	vector<bool> accountedForIDs(pts.size()*10, false); // index is ID
	int id;		

	for(int i=0; i<pts.size(); i++){ // loop thru points to find connections
		mark.points[0].x = pts[i].getx();
		mark.points[0].y = pts[i].gety();

		for(int k=0; k<pts[i].getNumNeighbors(); k++){
			
			if(getNeighbor(pts[i].getID(), k, ep1, pts) &&
				       	!accountedForIDs[ep1.getID()]){
				mark.points[1].x = ep1.getx();
				mark.points[1].y = ep1.gety();
				mark.id = pts[i].getID() * 10000 + k; 
				marks.markers.push_back(mark);	
			}
		}		
		accountedForIDs[pts[i].getID()] = true;
	}

	auto end = std::chrono::steady_clock::now();
	std::cout<<"time to draw: "<<std::chrono::duration_cast
		<std::chrono::milliseconds>(end-start).count()<<"\n";

	rvizMap.publish(marks);	

	// If running continually have a small delay otherwise 
	struct timespec req = {0};
	if(runBool){
		int milli = 40;
		req.tv_sec = 0;
		req.tv_nsec = milli * 1000000L;
		nanosleep(&req, (struct timespec *)NULL);
	}
	else{ sleep(3);}

	rvizMap.publish(marks);	
	
	if(runBool){nanosleep(&req, (struct timespec *)NULL);}
	else{ sleep(5);}

	rvizMap.publish(marks);	
} 

// Returns list of ids from start to end that represent the shortest
// path between two waypoints
vector<int> Nav::findPath(int start, int end, vector<EndPoint> &pts){

	float record = LARGENUM;
	vector<vector<int>> paths(100); // store each path which itself is a list of point IDs 
	vector<float> dists(100, 0);    // store the corresponding travel distance for a path
	vector<int> finalpath;
	EndPoint tail, neigh;           // tail is the last endpoint of i_th path (paths[i].back())
	bool done = false;
	paths[0] = {start}; // initalize
	dists[0] = 0;
	int last = 0; // index of last valid path in paths since the vector is a large constant size
	int index;    
	if(start == end) return finalpath; // check for same point or not in list
	if(getPoint(start,pts).getID()<0 || getPoint(end,pts).getID()<0) return finalpath;
	while(!done){
		done = true;

		// debugging output code, leave here for when it inevitable breaks
	//	/*
		for(int k=0; k<9; k++){
			for(int i=0; i<=last; i++){
				if(k==0){
					for(int tmp = 999; tmp>dists[i]; tmp/=10){
						std::cout<<" ";
						//tmp*=10;
					}	
					std::cout<<(int) dists[i];
				}
				else{
					int id = -1;
					if(k<paths[i].size()){ id = paths[i][k];}
					if(id == -1){ std::cout<<"   _";} 
					else{
						if(id<10){std::cout<<" ";}
						std::cout<<"  "<<id;
					}
				}
			}
			std::cout<< "\n";
		}
		std::cout<<"----------------------------"<<" record = "<<record<<"\n";
	//	*/

		if(last>90) ROS_ERROR("More paths encountered than expected! increase size? Nav.cpp ln 460\n");
		int len = last+1; // last is going to change as new paths are added
		for(int p=0; p<len; p++){
			if(paths[p].size()>0 /*&& dists[p] < record*/) { // if not deleted and still viable 
				tail = getPoint(paths[p].back(), pts);
				vector<int> neighs = tail.getNeighborList();  	

				vector<int> todelete;
				bool usefirst = true;
				for(int i=0; i<neighs.size(); i++){
					if(paths[p].size() < 2 || 
							paths[p].size() > 1 && neighs[i] != paths[p][paths[p].size() - 2]){ // no retracing

						if(usefirst) {
							index = p;
							usefirst = false;
						}
						else{
							paths[++last] = paths[p]; // make one longer and copy path
							// since the first vector was already used, subtract the prev. added distance
							// and remove the neighbor so this second neighbor can be added
							dists[last] = dists[p] - getDistance(getPoint(paths[p].back(), pts), neigh);
							paths[last].pop_back();
							index = last;
						}
						neigh = getPoint(neighs[i], pts);
						// check if the path closed on itself
						bool closed = std::find(paths[index].begin(),
							   	paths[index].end(), neighs[i]) != paths[index].end();

						paths[index].push_back(neighs[i]);
						dists[index] = dists[index] + getDistance(neigh, tail);
						if(neighs[i] == end ){
							todelete.push_back(index);
							if( dists[index] < record) { 
								record = dists[index]; 
								finalpath = paths[index];	
						///* // output the connected valid path 		
								std::cout<<"connected a valid loop record = "<<record<<" loop : ";
								
								for(int t=0; t<finalpath.size(); t++){
									std::cout<<"  "<<finalpath[t];
								}
								std::cout<<"\n";
								for(int t=0; t<finalpath.size()-1; t++){
									std::cout<<"  "<<getDistance(getPoint(finalpath[t],pts),
										   	getPoint(finalpath[t+1], pts));
								}
								std::cout<<"\n";
					//	*/

							}
						}
						else if(closed){
							todelete.push_back(index);
						//	std::cout<<"closed at index "<<index<<"\n";
						}
						else if(dists[index]>record){
							todelete.push_back(index);
						//	std::cout<<"above record at index "<<index<<"\n";
						}
						else{
							done = false; // still a valid path being built, keep going
						}
					} // done checking retracing	
				} // done looping through neighbors

				for(int i=0; i<todelete.size(); i++){ // could delete entries but resizing might be faster
					paths[todelete[i]].resize(0);
					dists[todelete[i]] = 1;
				}
			} 
		
		}	// done looping through paths	
	}
	if(record == LARGENUM) { ROS_ERROR("no suitable path found!\n");}
	return finalpath;
}

float Nav::getDistance(EndPoint &ep1, EndPoint &ep2){
	return pow(pow(ep1.getx() - ep2.getx(),2) + pow(ep1.gety() - ep2.gety(),2), 0.5);
}

// prints the map point info to console
void Nav::outputGraph(vector<EndPoint> &pts){
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

// gets a point given an ID, assumes in order initially then searches 
EndPoint& Nav::getPoint(int id, vector<EndPoint> &pts){
	if(pts[id].getID() == id) return pts[id];
	for(int i=0; i<pts.size(); i++){
		if(pts[i].getID() == id) return pts[i];
	}
	return badPt;
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

void Nav::setRun(bool t){ runBool = t;}
vector<EndPoint>* Nav::getMap() {return &mapPoints;}
vector<EndPoint>* Nav::getWays(){return &wayPoints;}
