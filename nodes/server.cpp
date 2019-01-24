#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_tutorials/TutorialsConfig.h>

void callback(firebot::ReconConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.double_param, config.double_param, 
            config.double_param, config.double_param , 
            config.double_param, config.double_param);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "firebot");

  dynamic_reconfigure::Server<firebot::ReconConfig> server;
  dynamic_reconfigure::Server<firebot::ReconConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}

