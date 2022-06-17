#pragma once

#include <string>

#include <ros/ros.h>
#include <service_call/incrementer.h>

class RosMiddleware : public Incrementer::MiddlewareHandle {
 public:
  RosMiddleware(const ros::NodeHandle& nh, std::string topic);
  void registerCallback(Callback cb) override;

 private:
  ros::NodeHandle nh_;
  std::string topic_;
  ros::ServiceServer service_;
};
