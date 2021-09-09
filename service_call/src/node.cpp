#include <ros/ros.h>

#include <memory>

#include "service_call/incrementer.h"
#include "service_call/ros_middleware.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "Incrementer");
  ros::NodeHandle n;
  Incrementer incrementer{std::make_unique<RosMiddleware>(n, "/serve")};

  ros::spin();

  return 0;
}
