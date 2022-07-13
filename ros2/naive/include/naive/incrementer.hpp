#pragma once

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class Incrementer {
public:
	Incrementer(std::shared_ptr<rclcpp::Node> node, const std::string& in_topic, const std::string& out_topic);
private:
	static constexpr int QUEUE_SIZE = 10;
	std::shared_ptr<rclcpp::Node> node_;
	std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int64>> pub_;
	std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int64>> sub_;
};