#include "naive/incrementer.hpp"
#include "naive/ros_middleware.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int64.hpp>

Incrementer::Incrementer(std::unique_ptr<MiddlewareHandle> mw)
    : mw_{std::move(mw)}
      {
        mw_->registerCallback([this](const std_msgs::msg::Int64::SharedPtr msg) {
            std_msgs::msg::Int64 incremented;
            incremented.data = msg->data + 1;
            mw_->publish(incremented);
          });
      }
