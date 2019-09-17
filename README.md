# firebot
The Trinity College fire-fighting robotics competition is an automonous robotics challenge held annually for students and professional roboticists. Teams must design, fabricate, and program a completely autonomous robot to navigate a reconfigurable "floor layout" and extinguish a candle.  

## Demo
Demonstration of navigation with ROS visualization tools. Automatic localization updates occur as the robot navigates which corrects for odometric error. The red markers (indicating wall corners) turn green when that particular wall is used to update the location. Additionally, the green waypoints indicate the path chosen by the robot's planning algorithm which guarantees the shortest route between rooms.  
![Demo](/gif/FF_demo.gif)  

## "Furniture" Detection
Detecting obstacles and walls in the maze using the RPLIDAR sensor.  
![Furniture Detection](/gif/cut_furniture.gif)  

## ROS Network
The ROS network implemented was simple but effective. The main "Firebot" node contained the three necessary C++ classes in order for the robot to function. The Lidar class subscribed to the RPLIDAR node and thereby gained access to the sensor data across the ROS network.  
![ROS Network](/gif/ROS_net.png)  

## Multi-Threading
Several threads interacted seamlessly which enabled the localization updates while completing other tasks. The mainLogic() thread decided what major actions the robot should take and generated the navigation stack which was followed by the PID loop. The driveLoop() function handled calculating the robot's location and controlling the various actuators. As RPLIDAR data was collected and computed the robot could update it's position based on a reconfigurable map stored in the Navigation class. This update would then be integrated back into the driveLoop when the odometry was calculated.   
![Threads](/gif/threads.png)  

## Top View
The RPLIDAR sensor was mounted directly in the center of the robot for best viewing. The Odroid computer (blue breadboard) handled all the data processing, the black circuit board above the RPLIDAR is the Cytron 10A motor controller and the PCB on the opposite side was our embedded arduino for PWM motor control.  
![top](/gif/top.png) 

## Front View
The spray nozzle was actuated via a solenoid valve and a kill switch was wired up for quick e-stop. Yellow tubing is the water lines (pump hidden underneath) and the arms were for an additional part of the challenge involving resuing a baby doll from a crib.  
![front](/gif/front.jpg)
