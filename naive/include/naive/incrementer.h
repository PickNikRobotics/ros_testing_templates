#pragma once

#include <ros/ros.h>

#include <string>

class Incrementer {
 public:
  Incrementer(ros::NodeHandle& nh, const std::string& in_topic,
              const std::string& out_topic);

 private:
  static constexpr int QUEUE_SIZE = 10;
  ros::Publisher pub_;
  ros::Subscriber sub_;
};
