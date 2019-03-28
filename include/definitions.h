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


