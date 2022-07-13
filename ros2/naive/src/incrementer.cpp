#include "naive/incrementer.hpp"

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

Incrementer::Incrementer(std::shared_ptr<rclcpp::Node> node, const std::string& in_topic, const std::string& out_topic)
: node_{std::move(node)}, 
  pub_{node_->create_publisher<std_msgs::msg::Int64>(out_topic,QUEUE_SIZE)},
  sub_{node_->create_subscription<std_msgs::msg::Int64>(in_topic, QUEUE_SIZE, [this](const std_msgs::msg::Int64::UniquePtr msg)
                                                                                  {
                                                                                    std_msgs::msg::Int64 incremented;
                                                                                    incremented.data = msg->data + 1;
                                                                                    pub_->publish(incremented);
                                                                                  })} {}