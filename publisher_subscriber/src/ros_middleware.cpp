#include <publisher_subscriber/ros_middleware.h>
#include <ros/ros.h>
#include <std_msgs/Int64.h>

#include <string>

RosMiddleware::RosMiddleware(const ros::NodeHandle& nh, std::string in_topic,
                             const std::string& out_topic)
    : nh_{nh},
      in_topic_{std::move(in_topic)},
      pub_{nh_.advertise<std_msgs::Int64>(out_topic, QUEUE_SIZE)} {}

void RosMiddleware::registerCallback(Callback cb) {
  sub_ = nh_.subscribe<std_msgs::Int64>(in_topic_, QUEUE_SIZE, cb);
}

void RosMiddleware::publish(std_msgs::Int64 msg) { pub_.publish(msg); }
