// Copyright 2022 Griswald Brooks
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "example_msgs/action/simple.hpp"  // for test_action_msgs::action::Simple
#include "gmock/gmock.h"
#include "rcl_handle/action.hpp"  // for ActionServer

#include <chrono>  // for seconds
#include <memory>  // for make_shared
#include <string>  // for string

#include <rclcpp/rclcpp.hpp>                // for Node, NodeOptions
#include <rclcpp_action/rclcpp_action.hpp>  // for rclcpp_action::ClientGoalHandle

namespace {

using namespace std::chrono_literals;
using Simple = example_msgs::action::Simple;

/// \brief Blocking call to send goal to action server and get result
rclcpp_action::ClientGoalHandle<Simple>::WrappedResult send(
    std::shared_ptr<rclcpp::Node> node, Simple::Goal const& goal) {
  auto client = rclcpp_action::create_client<Simple>(node, "Simple");
  if (!client->wait_for_action_server(20s)) {
    throw std::runtime_error("send goal service didn't become available");
  }

  auto future = client->async_send_goal(goal);

  if (rclcpp::FutureReturnCode::SUCCESS !=
      rclcpp::spin_until_future_complete(node, future)) {
    throw std::runtime_error("send goal future didn't complete successfully");
  }
  auto const result = client->async_get_result(future.get());
  if (rclcpp::FutureReturnCode::SUCCESS !=
      rclcpp::spin_until_future_complete(node, result)) {
    throw std::runtime_error("result goal future didn't complete successfully");
  }

  return result.get();
}

}  // namespace

namespace rcl_handle {
TEST(SimpleAction, ResultTrue) {
  // GIVEN A simple action server that sets results true
  auto const node = std::make_shared<rclcpp::Node>("actiontest");
  auto as = action_server<Simple>{node, "Simple"};
  as.register_handles(
      [](auto const&, auto) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](auto) { return rclcpp_action::CancelResponse::REJECT; },
      [](auto handle) {
        auto result = std::make_shared<Simple::Result>();
        result->succeed = true;
        handle->succeed(std::move(result));
      });
  // WHEN a goal is sent
  auto const result = send(node, Simple::Goal{});
  // THEN the result will be successful.
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.result->succeed);
}

TEST(SimpleAction, ResultFalse) {
  // GIVEN A simple action server that sets results false
  auto const node = std::make_shared<rclcpp::Node>("actiontest");
  auto as = action_server<Simple>{node, "Simple"};
  as.register_handles(
      [](auto const&, auto) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](auto) { return rclcpp_action::CancelResponse::REJECT; },
      [](auto handle) {
        auto result = std::make_shared<Simple::Result>();
        result->succeed = false;
        handle->succeed(std::move(result));
      });
  // WHEN a goal is sent
  auto const result = send(node, Simple::Goal{});
  // THEN the result will not be successful.
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_FALSE(result.result->succeed);
}

}  // namespace rcl_handle

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  auto const test_result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return test_result;
}
