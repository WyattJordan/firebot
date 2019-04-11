/* nav.cpp
 *
 */
#include "Nav.h"
void Nav::defaults(){
	smallRoomConf_ = bigRoomConf_ = false;
	pubMap_ = pubWays_ = pubRob_ = false;
	vector<int> none;
	none.resize(0);
	badPt_ = EndPoint(-1, -1, -1, none);
	mapPoints_.resize(0);
	wayPoints_.resize(0);
	cmapLine_ = {0.9, 0.5, 0.0};
	cmapMark_ = {1.0, 0.1, 0.1};
	cwayLine_ = {0.1, 0.9, 0.9};
	cwayMark_ = {0.8, 0.1, 0.9};
}
// don't use default constuctor, it must load the map and way files
Nav::Nav(){defaults();}

Nav::Nav(int lvl, ros::Publisher* pub){  
	defaults();
	markerPub_ = pub;
	cout<<"going to load files with just pub\n";
	loadFiles(lvl);	
	cout<<"going to init marks with just pub\n";
	initRobotMarks();
}

// This is the main constructor, initialize variables here
Nav::Nav(int lvl, ros::Publisher* pub, Robot* rob){  
	defaults();
	markerPub_ = pub;
	rob_ = rob;
	cout<<"going to load files with rob\n";
	loadFiles(lvl);	
	cout<<"going to init marks with rob\n";
	initRobotMarks();
}
void Nav::setOdomLoc(Vector3f od){
	odomWorldLocCpy_ = od;
}

void Nav::updatePositionAndMap(vector<line> lns, Vector3f pos, tf::Transform trans, Ref<Vector3f> travelDist){
	// GIVEN - there is at least one line with R^2 > 0.85, R^2 values have been made
	
	// sort by RSquared to use for pose calculations
	std::sort(lns.begin(),lns.end(),[](line a, line b) -> bool {return a.getRSquared() > b.getRSquared();});

	vector<float> poses, R; // in radians and unitless respectively
	vector<int> lineIdxs; // indexes of lines sorted by R squared and useable (within LineAngleThresh) if idx<0 it's a vertical line
	
	for(int i=0; i<lns.size(); i++){ // this is confusing stuff
		line l = lns[i];
		if(l.getRSquared() > 0.8){
			// angle is wall angle in global frame (= angle in lidar frame + pose angle) should be close to vertical or horizontal
			float angle = 180.0/PI * (atan(l.getSlope()) + pos(2)); // range is unknown due to pos(2) from 'bot
			angle = fmod(angle, 360.0);  
			angle = angle < 0 ? angle + 360.0 : angle; // range is now 0 to 360

			// check if the global angle is closer to vertical or horizontal within a limit
			if( ab(90.0-angle) < LineAngleThresh || ab(270.0-angle) < LineAngleThresh){ // use as vertical line
				float error = ab(90.0-angle) < ab(270.0-angle) ? (90.0-angle) : (270.0-angle);
				poses.push_back(pos(2) - PI/180.0*error); // subtract the error from the thought position to get the actual
				R.push_back(l.getRSquared());
				lineIdxs.push_back(-1*i); // save as negative idx for vertical line
			}
			else if( ab(angle) < LineAngleThresh || ab(180.0-angle) < LineAngleThresh){ // use as horizontal line
				float error = ab(angle) < ab(180.0-angle)      ?     angle    : (180.0-angle);
				poses.push_back(pos(2) - PI/180.0*error);
				R.push_back(l.getRSquared());
				lineIdxs.push_back(i); // keep as positive for horizontal line
			}
		}

	}

	float sumR = std::accumulate(R.begin(), R.end(), 0.0);
	float finalLidarPose = 0;
	float justAvgPose = std::accumulate(poses.begin(), poses.end(), 0.0) / (float)poses.size();
	for(int i=0; i<poses.size(); i++){ // weighted average for pose based on R^2 value of lines used
		finalLidarPose += poses[i] * R[i] / sumR;
	}

	float sumWeights = LidarErrorEquivalentDist + travelDist(2);
	// Lidar is weighted heavier when the robot has traveled further between updates
	float finalPose = finalLidarPose*travelDist(2)/ sumWeights + pos(2)*LidarErrorEquivalentDist/ sumWeights;
					
	cout<<"Lidar used "<<poses.size()<<" lines and avg pose is: "<<180.0/PI*justAvgPose<<" but weighted is: "<<180.0/PI*finalLidarPose<<
		" but after weighting with odom: "<<180.0/PI*finalPose<< " with dist = "<<travelDist(2)<<"\n";
	travelDist(2) = 0;
	// use old pos and get current position to get position difference?
	Vector3f tmp;
	tmp << 0, 0, finalLidarPose; // VERY IMPORTANT WHICH GETS SENT
	rob_->setExperimental(tmp);


	// TODO make updates for locations
	int updateX = 0;
	int updateY = 0;
	// should these have different travel distances ? one for x,y, and theta?
	if(lineIdxs.size()>0){

	}
	


}
// Robot.cpp drive loop sets the bool flags at set rates
void Nav::publishLoop(){
	while(1){
		if(pubWays_ || pubMap_ || pubRob_){
			//cout<<"published ways/map/robot\n";
			if(pubMap_){
				markerPub_->publish(wayMarks_);
				pubWays_ = false;
			}
			if(pubMap_){
				markerPub_->publish(mapMarks_);
				pubMap_ = false;
			}
			if(pubRob_){
				calcRobotMarks(); // rob obj passed in odomloc, set mark vals
				markerPub_->publish(robMarks_);
				pubRob_ = false;
			}
		}
	}
}

void Nav::publishLoopContinual(){
	while(1){
		markerPub_->publish(wayMarks_);
		markerPub_->publish(mapMarks_);
		calcRobotMarks(); // rob obj passed in odomloc, set mark vals
		markerPub_->publish(robMarks_);
		sleep(1);
	}
}


// Sets the door configuration for the small room, if up == true the 
// door is on the higher side (larger y coordinate) 
void Nav::setSmallRoomUpper(bool up){
	if(up){
		getPoint(18,mapPoints_).setNeighbors(1,12);		
		getPoint(13,mapPoints_).setNeighbors(1,14);
		getPoint(11,mapPoints_).setNeighbors(2,12,14);
		getPoint(14,mapPoints_).setNeighbors(2,13,11);
		removePoint(19,mapPoints_);

		getPoint(12,wayPoints_).setNeighbors(1,11);
		getPoint(13,wayPoints_).setNeighbors(2,5,15);
		}
	else{
		getPoint(11,mapPoints_).setNeighbors(1,12);
		getPoint(19,mapPoints_).setNeighbors(1,14);
		getPoint(12,mapPoints_).setNeighbors(2,11,13);
		getPoint(13,mapPoints_).setNeighbors(2,12,14);
		removePoint(18,mapPoints_);

		getPoint(11,wayPoints_).setNeighbors(1,12);
		getPoint(10,wayPoints_).setNeighbors(2,9,14);
	}
	makeMapMarks("smallRoom_map");
}

// Sets the door configuration for the larger room, if up == true the door is
// in the higher location (larger y coordinate)
void Nav::setBigRoomUpper(bool up){
	if(up){
		getPoint(17,mapPoints_).setNeighbors(1,20);
		getPoint(16,mapPoints_).setNeighbors(1,15);

		getPoint(3,wayPoints_).setNeighbors(2,2,4);
		removePoint(17,wayPoints_);
	}
	else{
		getPoint(17,mapPoints_).setNeighbors(1,16);
		removePoint(20,mapPoints_);

		getPoint(4,wayPoints_).setNeighbors(2,3,5);
		getPoint(16,wayPoints_).setNeighbors(2,15,17);
	}	
	makeMapMarks("bigRoom_map");
}

// Determines which map points are visible to the robot given 
// the robot's location on the map
void Nav::findExpected(float Rx, float Ry, vector<EndPoint> &pts){
	for(EndPoint &ep : pts){
		ep.calcPolar(Rx, Ry); // calculate all polars
		ep.setDone(false);
	}
	std::sort(pts.begin(), pts.end(), 
			[](const EndPoint& lhs, const EndPoint &rhs){
			return	lhs.getCalculatedR() < rhs.getCalculatedR();	
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
	//cout<<"elim from "<<ep1.getID()<<" to "<<ep2.getID()<<"\n";
	for(EndPoint &p : pts){
		if(!p.getDone()){

			if(p.getCalculatedR()<15 || p.getCalculatedR() > 130){ // must be within LIDAR range
				p.setVisible(false);
				p.setDone(true);
			}
			else{
				float theta = p.getCalculatedTheta();
				bool vert = ep1.getX() == ep2.getX() ? 1 : 0; // otherwise horiz line	
				bool above = (vert && Rx < ep1.getX()) || (!vert && Ry < ep1.getY());
				float start = std::min(ep1.getCalculatedTheta(),ep2.getCalculatedTheta());
				float end =   std::max(ep1.getCalculatedTheta(),ep2.getCalculatedTheta());
				bool cross = false;
				if(end - start> 180) { // they are crossing the 0 
					cross = true;	
				}	
				float margin = 5; // if it's within 5 deg still eliminate
				if( ((!cross && theta > start - margin && theta < end + margin) ||
					(cross && (theta > end - margin || theta < start + margin))) 
					&&   (	
					( vert &&  above && p.getX() > ep1.getX()) ||
					( vert && !above && p.getX() < ep1.getX()) ||
					(!vert &&  above && p.getY() > ep1.getY()) ||
					(!vert && !above && p.getY() < ep1.getY())    )){  
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
bool Nav::getNeighbor(int ID, int neighI, EndPoint &neigh, vector<EndPoint> &pts){ 
	EndPoint start = getPoint(ID, pts);
	if(start.getID()!=-1){ 
		neigh = getPoint(start.getNeighborID(neighI), pts);
		if(neigh.getID()==-1){
			neigh = badPt_;
			return false;
		}
		return true;
	}
	else{
		neigh = badPt_;
		return false;
	}
}

void Nav::makeLineMarks(vector<line> lines, bool merged, bool addIDs){
	visualization_msgs::MarkerArray tmp;
	visualization_msgs::Marker txt;
	tmp.markers.resize(1+lines.size()); // 1 is the line list, rest are ID texts

	tmp.markers[0].header.frame_id = ROBOTFRAME;
	tmp.markers[0].ns = merged ? "merged_lines":"broken_lines"; 
	tmp.markers[0].id = 0; 
	tmp.markers[0].type = visualization_msgs::Marker::LINE_LIST;
	tmp.markers[0].action = visualization_msgs::Marker::ADD;
	tmp.markers[0].scale.x = 2;  // width of the lines 

	tmp.markers.resize(2*lines.size());
	tmp.markers[0].points.resize(2*lines.size()); // add two points for every line
	for(int i=0; i<lines.size(); i++){
		float x1 = lines[i].getEndPtX1();
		float y1 = lines[i].getEndPtY1();
		float x2 = lines[i].getEndPtX2();
		float y2 = lines[i].getEndPtY2();
		bool vertical = false;
		if(ab(x1-x2) < ab(y1-y2)) vertical = true; // pick which dimensions of endpoints to use

		if(!vertical){
			y1 = x1 * lines[i].getSlope() + lines[i].getIntercept();
			y2 = x2 * lines[i].getSlope() + lines[i].getIntercept();
		}
		else{
			x1 = (y1 - lines[i].getIntercept())/lines[i].getSlope();
			x2 = (y2 - lines[i].getIntercept())/lines[i].getSlope();
		}

		if(merged) {
			x1+=2;
			y1+=2;
			x2+=2;
			y2+=2;
		}
		tmp.markers[0].points.at(2*i).x = x1;
		tmp.markers[0].points.at(2*i).y = y1;
		tmp.markers[0].points.at(2*i).z = 3;
		tmp.markers[0].points.at(2*i+1).x = x2;
		tmp.markers[0].points.at(2*i+1).y = y2;
		tmp.markers[0].points.at(2*i+1).z = 3;

		if(addIDs){
			tmp.markers[1+i].header.frame_id = ROBOTFRAME;
			tmp.markers[1+i].id = i+1;
			tmp.markers[1+i].ns = merged ? "merged_lines":"broken_lines";
			tmp.markers[1+i].text = std::to_string(i);
			tmp.markers[1+i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			tmp.markers[1+i].action = visualization_msgs::Marker::ADD;
			tmp.markers[1+i].scale.x = 3;
			tmp.markers[1+i].scale.y = 3;
			tmp.markers[1+i].scale.z = 6.0; // text height	
			tmp.markers[1+i].pose.position.x = (x1+x2)/2.0;
			tmp.markers[1+i].pose.position.y = (y1+y2)/2.0;
			tmp.markers[1+i].pose.position.z = 15;
			tmp.markers[1+i].pose.orientation.x = 0;
			tmp.markers[1+i].pose.orientation.y = 0;
			tmp.markers[1+i].pose.orientation.z = 0;
			tmp.markers[1+i].pose.orientation.w = 1;
			tmp.markers[1+i].color.a = 1.0;  // white
			tmp.markers[1+i].color.r = 1; 
			tmp.markers[1+i].color.g = merged? 0:1; 
			tmp.markers[1+i].color.b = merged? 0:1; 
		}

	}
		
	tmp.markers[0].pose.position.x = 0;
	tmp.markers[0].pose.position.y = 0;
	tmp.markers[0].pose.position.z = 0;
	tmp.markers[0].pose.orientation.x = 0;
	tmp.markers[0].pose.orientation.y = 0;
	tmp.markers[0].pose.orientation.z = 0;
	tmp.markers[0].pose.orientation.w = 1;

	if(merged){ // merged lines are yellow w/ red ID nums
		tmp.markers[0].color.a = 1.0;	
		tmp.markers[0].color.r = 235.0/255.0;
		tmp.markers[0].color.g = 1.0;
		tmp.markers[0].color.b = 22.0/225.0;
	}
	else{ // unmerged are blue with white ID nums
		tmp.markers[0].color.a = 1.0;	
		tmp.markers[0].color.r = 0;
		tmp.markers[0].color.g = 0;
		tmp.markers[0].color.b = 1.0;
	}

	/*for(int i=0; i<lineMarks_.markers.size(); i++){
		lineMarks_.markers[i].action = visualization_msgs::Marker::DELETE;
	}
	if(lineMarks_.markers.size()!=0) markerPub_->publish(lineMarks_);// delete all old ones	
	*/
	
	//cout<<"pubbing line marks of size "<<lines.size()<<"\n";
	lineMarks_ = tmp;
	markerPub_->publish(lineMarks_);
}

void Nav::makeFurnMarks(vector<EndPoint> furns){
	//cout<<"making furn marks from endpoint vec with size "<<furns.size()<<"\n";
	visualization_msgs::MarkerArray tmp;
	tmp.markers.resize(furns.size());

	for(int i=0; i<tmp.markers.size(); i++){
		tmp.markers[i].header.frame_id = ROBOTFRAME;
		tmp.markers[i].ns = "furn_Arr"; 
		tmp.markers[i].id = i; 
		tmp.markers[i].type = visualization_msgs::Marker::CYLINDER;
		tmp.markers[i].action = visualization_msgs::Marker::ADD;
		tmp.markers[i].scale.x = FurnWidth; 
		tmp.markers[i].scale.y = FurnWidth;
		tmp.markers[i].scale.z = 15;

		tmp.markers[i].pose.position.x = furns[i].getX();
		tmp.markers[i].pose.position.y = furns[i].getY();
		tmp.markers[i].pose.position.z = 0;
		tmp.markers[i].pose.orientation.x = 0;
		tmp.markers[i].pose.orientation.y = 0;
		tmp.markers[i].pose.orientation.z = 15.0/2.0; // half of scale.z for cylinder height
		tmp.markers[i].pose.orientation.w = 1;

		tmp.markers[i].color.a = 1.0;	
		tmp.markers[i].color.r = 0.5;
		tmp.markers[i].color.g = 1.0;
		tmp.markers[i].color.b = 0.5;
	}

	for(int i=0; i<furnMarks_.markers.size(); i++){
		furnMarks_.markers[i].action = visualization_msgs::Marker::DELETE;
	}
	if(furnMarks_.markers.size()!=0) markerPub_->publish(furnMarks_);// delete all old ones	
	
	furnMarks_ = tmp;
	markerPub_->publish(furnMarks_);
	
	//cout<<"size of furn array is: "<<furnMarks_.markers.size()<<"\n";
}

// populate MarkerArray members (run by the makeMapMarks() and makeWayMarks()) 
void Nav::populateMarks(string which, string NS,
	   	visualization_msgs::MarkerArray &marks, color lncol, color markcol){
	vector<EndPoint>* pts;
	if(which == "map") { pts = &mapPoints_; }
	if(which == "way") { pts = &wayPoints_; }
	marks.markers.resize(2 * (pts->size()));
	
	// add vertical arrows at pt locations
	for(int i=0; i<pts->size(); i++){
		marks.markers[i].header.frame_id = GLOBALFRAME;
		marks.markers[i].ns = NS; 
		marks.markers[i].id = i; //pts[i].getID();
		marks.markers[i].type = visualization_msgs::Marker::ARROW;
		marks.markers[i].action = visualization_msgs::Marker::ADD;
		
		marks.markers[i].points.resize(2);
		marks.markers[i].points[0].x = (*pts)[i].getX();
		marks.markers[i].points[0].y = (*pts)[i].getY();
		marks.markers[i].points[0].z = 0;
		marks.markers[i].points[1].x = (*pts)[i].getX();
		marks.markers[i].points[1].y = (*pts)[i].getY();
		marks.markers[i].points[1].z = 10; // 10cm tall 

		marks.markers[i].scale.x = 3; 
		marks.markers[i].scale.y = 3;
		marks.markers[i].scale.z = 3;
		marks.markers[i].color.a = 1.0;	
		marks.markers[i].color.r = (*pts)[i].isVisible() ? 0.0 : markcol.r;	
		marks.markers[i].color.g = (*pts)[i].isVisible() ? 1.0 : markcol.g;
		marks.markers[i].color.b = (*pts)[i].isVisible() ? 0.0 : markcol.b;	

		// show text ID above marker with id += 1000
		int idx = i+pts->size();
		marks.markers[idx].header.frame_id = GLOBALFRAME;
		marks.markers[idx].id = i+1000;
		marks.markers[idx].ns = NS;
		marks.markers[idx].text = std::to_string((*pts)[i].getID());
		marks.markers[idx].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marks.markers[idx].action = visualization_msgs::Marker::ADD;
		marks.markers[idx].scale.x = 3;
		marks.markers[idx].scale.y = 3;
		marks.markers[idx].scale.z = 8.0; // text height	
		marks.markers[idx].pose.position.x = (*pts)[i].getX();
		marks.markers[idx].pose.position.y = (*pts)[i].getY();
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
	//cout<<"done adding points for "<< which<<"\n";

	// Now add the lines 
	visualization_msgs::Marker mark;
	mark.type = visualization_msgs::Marker::LINE_STRIP;
	mark.header.frame_id = GLOBALFRAME;
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
	vector<bool> accountedForIDs(pts->size()*10, false); // index is ID
	int id;		

	// LINE_STRIP is defined by n points, here n always = 2
	for(int i=0; i<pts->size(); i++){ 
		mark.points[0].x = (*pts)[i].getX(); // add the ith point to the line
		mark.points[0].y = (*pts)[i].getY();

		// loop through all the ith point's neighbors adding 	
		// and then the main MarkerArray them to the LINE_STRIP
		//cout<<"entering neghbor loop for the "<<i<<" time\n";
		for(int k=0; k<(*pts)[i].getNumNeighbors(); k++){
			
			if(getNeighbor((*pts)[i].getID(), k, ep1, *pts) &&
				       	!accountedForIDs[ep1.getID()]){
				mark.points[1].x = ep1.getX();
				mark.points[1].y = ep1.getY();
				mark.id = (*pts)[i].getID() * 10000 + k; 
				marks.markers.push_back(mark);	
			}
		}		
//		accountedForIDs[(*pts)[i].getID()] = true; // this worked until adding waypoint w/ 4 neighs, not time to fix now
		//cout<<"done neghbor loop for the "<<i<<" time\n";
	}
	if(which == "map") mapMarks_ = marks;
	if(which == "way") wayMarks_ = marks;
	
	
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
		/*
		for(int k=0; k<9; k++){
			for(int i=0; i<=last; i++){
				if(k==0){
					for(int tmp = 999; tmp>dists[i]; tmp/=10){
						cout<<" ";
						//tmp*=10;
					}	
					cout<<(int) dists[i];
				}
				else{
					int id = -1;
					if(k<paths[i].size()){ id = paths[i][k];}
					if(id == -1){ cout<<"   _";} 
					else{
						if(id<10){cout<<" ";}
						cout<<"  "<<id;
					}
				}
			}
			cout<< "\n";
		}
		cout<<"----------------------------"<<" record = "<<record<<"\n";
		//	*/

		if(last>90) ROS_ERROR("More paths encountered than expected! increase size? Nav.cpp ln 460\n");
		int len = last+1; // last is going to change as new paths are added
		for(int p=0; p<len; p++){
			if(paths[p].size()>0 /*&& dists[p] < record*/) { // if !deleted and still viable 
				tail = getPoint(paths[p].back(), pts);
				vector<int> neighs = tail.getNeighborList();  	

				vector<int> todelete;
				bool usefirst = true;
				for(int i=0; i<neighs.size(); i++){
					// parenthesis added so like x || (y && z), might break it...
					if(paths[p].size() < 2 || 
							(paths[p].size() > 1 && neighs[i] != paths[p][paths[p].size() - 2])){ // no retracing

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
						/* // debug output the connected valid path 		
								cout<<"connected a valid loop record = "<<record<<" loop : ";
								for(int t=0; t<finalpath.size(); t++){
									cout<<"  "<<finalpath[t];
								}
								cout<<"\n";
								for(int t=0; t<finalpath.size()-1; t++){
									cout<<"  "<<getDistance(getPoint(finalpath[t],pts),
										   	getPoint(finalpath[t+1], pts));
								}
								cout<<"\n";
						//	*/

							}
						}
						else if(closed){
							todelete.push_back(index);
						//	cout<<"closed at index "<<index<<"\n";
						}
						else if(dists[index]>record){
							todelete.push_back(index);
						//	cout<<"above record at index "<<index<<"\n";
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


// prints the info for a vec<EndPoint> to console
void Nav::outputGraph(vector<EndPoint> &pts){
	EndPoint ep;
	cout<<"outputGraph with size: "<<pts.size()<<"\n";
	cout<<"////////////////////////////////////////////////////\n";
	for(int i=0; i<pts.size(); i++){
		cout<<" id: "<<pts[i].getID();
		cout<<"\tx: "<<pts[i].getX();
		cout<<"\ty: "<<pts[i].getY();
		cout<<"\tNEIGH: "<< pts[i].getNumNeighbors();
		for(int k=0; k<pts[i].getNumNeighbors(); k++){
			// for some reason this check needs to be here...
			bool check = getNeighbor(pts[i].getID(), k, ep, pts);
			if(check){
				cout<<"\tn: "<< ep.getID();
			}
			else{
				cout<<"\tn: N";
			}
		}
		cout<<"\n";
	}
	cout<<"////////////////////////////////////////////////////\n";
}
// gets a point given an ID, assumes in order initially then searches 
EndPoint Nav::getWayPoint(int id){
	return getPoint(id, wayPoints_);
}

EndPoint Nav::getMapPoint(int id){
	return getPoint(id, mapPoints_);
}



EndPoint& Nav::getPoint(int id, vector<EndPoint> &pts){
	if(pts.size() > 0 && pts[id].getID() == id) return pts[id];
	for(int i=0; i<pts.size(); i++){
		if(pts[i].getID() == id) return pts[i];
	}
	return badPt_ ;
}

// remove a point with a given ID
bool Nav::removePoint(int id, vector<EndPoint> &pts){
	for(int i=0; i<pts.size(); i++){
		if(pts[i].getID() == id){
			pts.erase(pts.begin() + i);
			return true;
		}
	}
	return false;
}

float Nav::getDistance(EndPoint &ep1, EndPoint &ep2){
	return pow(pow(ep1.getX() - ep2.getX(),2) + pow(ep1.getY() - ep2.getY(),2), 0.5);
}

void Nav::publishMapAndWays(){
	markerPub_->publish(mapMarks_);
	markerPub_->publish(wayMarks_);
}

void Nav::makeMapMarks(string NS){
	populateMarks("map", NS, mapMarks_, cmapLine_, cmapMark_); }
void Nav::makeWayMarks(string NS){
	populateMarks("way", NS, wayMarks_, cwayLine_, cwayMark_); }

void Nav::outputWays(){ outputGraph(wayPoints_);	}
void Nav::outputMap(){  outputGraph(mapPoints_);	}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Nav::loadFiles(int lvl){
	// the file loading into an Endpoint vector should be made into it's own function
	// though this does require identical formatting in the file fields
 	string mapfile, wayfile;
	if(lvl == 3){
		mapfile = "";
	    wayfile = "";  //root is catkin ws
	}	
	else {
		mapfile = "/home/eli/cat_ws/src/firebot/load/lvl1_map.txt"; 
		wayfile = "/home/eli/cat_ws/src/firebot/load/wayPoints.txt";	
	}

   
	std::ifstream file;
	file.open(mapfile.c_str());
	string line = "";

	vector<string> nums;
	if (file.is_open()){
                while(getline(file, line, ',')){
                        nums.push_back(line); //cout<<line<<"\n";
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
		mapPoints_.push_back(tmp);
		for(int i=0; i<7; i++) nums.erase(nums.begin()+0);
	}

	// SAME THING FOR WAY FILE WITH SOME ADJUSTMENTS /////////////////////////////////////
	nums.resize(0); // vector of comma delimited strings (all numbers, except for  terminating x char)
	file.open(wayfile.c_str());
	cout<<"outputting "<<wayfile<<"\n";
        if (file.is_open()){
                while(getline(file, line, ',')){
                        nums.push_back(line); //cout<<line<<"\n";
                }
                file.close();
        }
	else{ ROS_ERROR("WARNING: way file not found!\n"); }

	while(nums.size()>=4){ // at least 4 fields for each, rest are neighbors
		vector<int> temp;
		// nums[3] is radius here unused 
		int idx = 4;
		int count = 0;
		while(nums[idx].compare("x") != 0){
			temp.push_back(std::atof(nums[idx++].c_str())); 
			count++;
		}
		EndPoint tmp(std::atof(nums[0].c_str()), std::atof(nums[1].c_str()),
		    std::atof(nums[3].c_str()),temp); // nums[2] is radius set to 10 constant
		wayPoints_.push_back(tmp);
		for(int i=0; i<5+count; i++) nums.erase(nums.begin()+0);
	}
}

void Nav::calcRobotMarks(){
	// REMEMBER: values are in cm!!
	visualization_msgs::Marker robMarkSphere_, robMarkArr_;	
	robMarkArr_.header.frame_id = GLOBALFRAME;
	robMarkArr_.ns = "robot_Arr"; 
	robMarkArr_.id = 1; 
	robMarkArr_.type = visualization_msgs::Marker::ARROW;
	robMarkArr_.action = visualization_msgs::Marker::ADD;
	robMarkArr_.points.resize(2);
	robMarkArr_.scale.x = 4; 
	robMarkArr_.scale.y = 5;
	robMarkArr_.scale.z = 6;

	robMarkArr_.color.a = 1.0;	
	robMarkArr_.color.r = 1.0;
	robMarkArr_.color.g = 0.0;
	robMarkArr_.color.b = 0.0;


	robMarkSphere_.header.frame_id = GLOBALFRAME;
	robMarkSphere_.ns = "robot_sph"; 
	robMarkSphere_.id = 1; 
	robMarkSphere_.type = visualization_msgs::Marker::SPHERE;
	robMarkSphere_.action = visualization_msgs::Marker::ADD;

	robMarkSphere_.scale.x = WheelDist*2; // WheelLCM = 13.75
	robMarkSphere_.scale.y = WheelDist*2;
	robMarkSphere_.scale.z = 5;

	robMarkSphere_.color.a = 1.0;
	robMarkSphere_.color.r = 0.0;
	robMarkSphere_.color.g = 0.5;
	robMarkSphere_.color.b = 1.0;

	robMarks_.markers.resize(0);
	robMarks_.markers.push_back(robMarkArr_);
	robMarks_.markers.push_back(robMarkSphere_);

	// robMarks_[0] is arrow, [1] is sphere
	int z = 6;
	float len = 23.0; // length of the arrow
	float tipx = odomWorldLocCpy_(0) + len*cos(odomWorldLocCpy_(2)); 
	float tipy = odomWorldLocCpy_(1) + len*sin(odomWorldLocCpy_(2)); 
       	robMarks_.markers[0].points[0].x = odomWorldLocCpy_(0);
	robMarks_.markers[0].points[0].y = odomWorldLocCpy_(1);
	robMarks_.markers[0].points[0].z = z;
	robMarks_.markers[0].points[1].x = tipx;
	robMarks_.markers[0].points[1].y = tipy;
	robMarks_.markers[0].points[1].z = z; 


	robMarks_.markers[1].pose.position.x = odomWorldLocCpy_(0);
	robMarks_.markers[1].pose.position.y = odomWorldLocCpy_(1);
	robMarks_.markers[1].pose.position.z = z;
	robMarks_.markers[1].pose.orientation.x = 0;
	robMarks_.markers[1].pose.orientation.y = 0;
	robMarks_.markers[1].pose.orientation.z = odomWorldLocCpy_(2);
	robMarks_.markers[1].pose.orientation.w = 1;
}

void Nav::initRobotMarks(){
	visualization_msgs::Marker robMarkSphere_, robMarkArr_;	
	robMarkArr_.header.frame_id = GLOBALFRAME;
	robMarkArr_.ns = "robot_Arr"; 
	robMarkArr_.id = 1; 
	robMarkArr_.type = visualization_msgs::Marker::ARROW;
	robMarkArr_.action = visualization_msgs::Marker::ADD;
	robMarkArr_.points.resize(2);
	robMarkArr_.scale.x = 0.5; 
	robMarkArr_.scale.y = 0.9;
	robMarkArr_.scale.z = 0.5;

	robMarkArr_.color.a = 1.0;	
	robMarkArr_.color.r = 1.0;
	robMarkArr_.color.g = 0.0;
	robMarkArr_.color.b = 0.0;


	robMarkSphere_.header.frame_id = GLOBALFRAME;
	robMarkSphere_.ns = "robot_sph"; 
	robMarkSphere_.id = 1; 
	robMarkSphere_.type = visualization_msgs::Marker::SPHERE;
	robMarkSphere_.action = visualization_msgs::Marker::ADD;

	robMarkSphere_.scale.x = 3;
	robMarkSphere_.scale.y = 3;
	robMarkSphere_.scale.z = 3;

	robMarkSphere_.color.r = 0.0;
	robMarkSphere_.color.g = 0.5;
	robMarkSphere_.color.b = 1.0;

	robMarks_.markers.resize(0);
	robMarks_.markers.push_back(robMarkArr_);
	robMarks_.markers.push_back(robMarkSphere_);
}

