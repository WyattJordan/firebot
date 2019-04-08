#pragma once

// was 13,95
#define WheelDist 13.15 // width is 28.8cm outer, 24cm inner, L is half this center value
#define WheelRad  6.2  // diameter is 12.4, radius is 6.2cm

#define WayPointStartThreshold 4.0  // pt must be at least 2cm away for Robot to move towards it
#define MinDistFor50 20.0 // must be at least 20cm away from pt to use 0.5 speed
#define StartBigTurnDist50 17.5 // for a >70 deg turn start 17.5cm before the corner pt
#define StartSmallTurnDist50 10 // for <70 deg turns get closer to the point (center of map has slight turns)
#define StopDist50 13.0
#define StopDist20 3.8
#define SamePoseThreshDeg 1 // if pose is within 3deg for next pt just pop from nav stack

#define PI2	  6.28319
#define PI	  3.14159 

#define ab std::abs

#define GLOBALFRAME "global"
#define ROBOTFRAME "laser_frame" // don't edit this so the robot frame is the same as the lidar node

// Definitions for lidar class
#define MAXDIST 180 // sets all range measurements above xCm to x
#define MINDIST 16 // deletes all range measurements below xCm
#define DoorJumpDist 40.0 // 40cm or more to be labeled as a jump in jumps_ for detecting door edges
#define SmallJumpDist 8.0 // Xcm or more to be labeled as a potential furn jump
#define FurnWidth 13.0 // expected furniture width (X cm)
#define FurnWidthTolerance 4.0 // distance between furn endpoints should be FurnWidth +/- X
#define FurnDistTolerance 2.0 // point between furn endpoints must have at least X smaller polar radius

#define PerpThresh 1.5 // err from line model cannot exceed this
#define PrevPointDistThresh 10 // stop lines from adding points that are far away (closing off doorways)
#define MinPtsForLine 10
#define MergeLineDistThresh 35
