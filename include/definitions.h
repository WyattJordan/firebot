#pragma once
// Constants for various settings throughout the program
// Comments say "X is ..." where X is whatever the value it is defined as is 

// physical constants of the 'bot for odometry (see Robot::calculateOdom())
#define WheelDist 13.15 // width is 28.8cm outer, 24cm inner, x is half the average of these two (roughly)
#define WheelRad  6.2   // diameter is 12.4, radius is 6.2cm

// constants for Robot::executeNavStack()
#define WayPointStartThreshold 4.0  // pt must be at least X cm away for Robot to move towards it
#define MinDistFor50 20.0 		// must be at least X cm away from pt to use 0.5 speed
#define StartBigTurnDist50 17.5 // for a >70 deg turn start X cm before the corner pt
#define StartSmallTurnDist50 12 // for <70 deg turns get closer to the point (center of map has slight turns)
#define StopDist50 13.0 // takes X cm to stop the robot when driving at 50% (found experimentally)
#define StopDist20 3.8  // takes X cm to stop the robot when driving at 20% (found experimentally)
#define SamePoseThreshDeg 1 // if pose is within X deg for next pt just pop from nav stack

#define PI2	  6.28319
#define PI	  3.14159 
#define ab std::abs // this might be unnecessary

// names of the two frames the robot uses
#define GLOBALFRAME "global"
#define ROBOTFRAME "laser_frame" // don't edit this so the robot frame is the same as the lidar node

// Definitions for lidar class
#define MAXDIST 180 // sets all range measurements above X cm to X
#define MINDIST 16  // deletes all range measurements below X Cm
#define DoorJumpDist 40.0 // X cm or more to be labeled as a jump in jumps_ for detecting door edges
#define SmallJumpDist 8.0 // X cm or more to be labeled as a potential furn jump
#define FurnWidth 13.0    // expected furniture width (X cm)
#define FurnWidthTolerance 5.0 // distance between furn endpoints should be FurnWidth +/- X, this just filters out large differences in jumps
#define FurnDistTolerance 2.0  // point between furn endpoints must have at least X smaller polar radius than the average of the endpoints

// used for building lines
#define PerpThresh 2.0 			// point err from line model cannot exceed this X cm to be added to the line
#define PrevPointDistThresh 10  // point cannot be greater than X cm away from prev pt, stop lines from adding points that are far away (closing off doorways)
#define MinPtsForLine 10   
#define MergeLineDistThresh 35 // two lines must have a combination of endpoints with distance < X cm to be merged (should be less than DoorWidth

// used for classifying as room 4 or 1
#define DoorWidth 46.0 
#define DoorWidthTol 8.0  // when checking if the 2 big jumps in room 4 are close enough to eachother to be a doorway

// used for position updates in Nav
#define LineAngleThresh 10.0 		  // must be within X deg to use for pose update
#define LidarErrorEquivalentDist 10.0 // weighs the theta updates as if the lidar has the same amount of error as X cm of travel w/ encoders
#define LidarUpdateRate 3 			  // will update every X scans (scans occure every 0.1s)


