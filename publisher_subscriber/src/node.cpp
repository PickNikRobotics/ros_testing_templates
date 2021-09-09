#include <ros/ros.h>

#include <memory>

#include "publisher_subscriber/incrementer.h"
#include "publisher_subscriber/ros_middleware.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "Incrementer");
  ros::NodeHandle n;
  Incrementer incrementer{std::make_unique<RosMiddleware>(n, "/in", "/out")};

  ros::spin();

  return 0;
}
