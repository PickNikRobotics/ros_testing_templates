#include <injection_messages/OneNumber.h>
#include <ros/ros.h>
#include <service_call/ros_middleware.h>

#include <string>

RosMiddleware::RosMiddleware(const ros::NodeHandle& nh, std::string topic)
    : nh_{nh}, topic_{std::move(topic)} {}

void RosMiddleware::registerCallback(Callback cb) {
  service_ =
      nh_.advertiseService<injection_messages::OneNumber::Request,
                           injection_messages::OneNumber::Response>(topic_, cb);
}
