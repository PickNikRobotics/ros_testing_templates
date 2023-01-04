#include "naive/incrementer.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

namespace {
  constexpr auto queue_size = 10;
}

Incrementer::Incrementer(std::shared_ptr<rclcpp::Node> node, std::string const& in_topic, std::string const& out_topic)
: node_{std::move(node)},
  pub_{node_->create_publisher<std_msgs::msg::Int64>(out_topic,queue_size)},
  sub_{node_->create_subscription<std_msgs::msg::Int64>(in_topic, queue_size,
                                                        [this](std_msgs::msg::Int64::UniquePtr const msg) {
                                                          std_msgs::msg::Int64 incremented;
                                                          incremented.data = msg->data + 1;
                                                          pub_->publish(incremented);
                                                        })} {}
