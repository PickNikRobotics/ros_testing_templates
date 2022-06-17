#include "naive/incrementer.h"

#include <ros/ros.h>
#include <std_msgs/Int64.h>

Incrementer::Incrementer(ros::NodeHandle& nh, const std::string& in_topic,
                         const std::string& out_topic)
    : pub_{nh.advertise<std_msgs::Int64>(out_topic, QUEUE_SIZE)},
      sub_{nh.subscribe<std_msgs::Int64>(in_topic, QUEUE_SIZE,
                                         [this](const auto& msg) {
                                           std_msgs::Int64 incremented;
                                           incremented.data = msg->data + 1;
                                           pub_.publish(incremented);
                                         })} {}
