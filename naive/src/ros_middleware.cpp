#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

#include <naive/ros_middleware.hpp>

RosMiddleware::RosMiddleware(const rclcpp::Node::SharedPtr node_handle, std::string in_topic,
                const std::string& out_topic)
	: node_handle_{node_handle}, 
	  in_topic_{std::move(in_topic)}, 
	  publisher_{node_handle_->create_publisher<std_msgs::msg::Int64>(out_topic, QUEUE_SIZE)} {}

void RosMiddleware::registerCallback(Callback cb) {
	subscriber_ = node_handle_->create_subscription<std_msgs::msg::Int64>(in_topic_, QUEUE_SIZE, cb);
}

void RosMiddleware::publish(std_msgs::msg::Int64 msg) { publisher_->publish(msg); }