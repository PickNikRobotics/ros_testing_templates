#pragma once

#include <publisher_subscriber/incrementer.h>
#include <ros/ros.h>

#include <string>

class RosMiddleware : public Incrementer::MiddlewareHandle {
 public:
  RosMiddleware(const ros::NodeHandle& nh, std::string in_topic,
                const std::string& out_topic);
  void registerCallback(Callback cb) override;
  void publish(std_msgs::Int64 msg) override;

 private:
  static constexpr int QUEUE_SIZE = 10;
  ros::NodeHandle nh_;
  std::string in_topic_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};
