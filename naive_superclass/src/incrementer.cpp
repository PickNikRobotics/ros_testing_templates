#include "naive_superclass/incrementer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int64.hpp>

Incrementer::Incrementer(const std::string node_name, const std::string in_topic, const std::string out_topic)
	: Node(node_name)
{
	publisher_ = this->create_publisher<std_msgs::msg::Int64>(out_topic, QUEUE_SIZE);
	subscriber_ = this->create_subscription<std_msgs::msg::Int64>(in_topic, QUEUE_SIZE, std::bind(&Incrementer::callback, this, std::placeholders::_1));
}

void Incrementer::callback(const std_msgs::msg::Int64::SharedPtr msg) {
    std_msgs::msg::Int64 incremented;
    incremented.data = msg->data + 1;
    publisher_->publish(incremented);
}
