#include "naive/incrementer.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "Incrementer");
  ros::NodeHandle nh;
  Incrementer incrementer{nh, "/in", "/out"};

  ros::spin();

  return 0;
}
