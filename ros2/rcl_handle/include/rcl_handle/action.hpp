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

#pragma once
#include <functional>  // for function
#include <memory>      // for unique_ptr, shared_ptr
#include <string>      // for string
#include <utility>     // for move

#include <rclcpp/rclcpp.hpp>                // for rclcpp::Node
#include <rclcpp_action/rclcpp_action.hpp>  // for rclcpp_action::Server

/// \namespace rcl_handle
/// \brief Wrappers for rclcpp action servers so they can be mocked for testing
namespace rcl_handle {

/// \namespace interface
/// \brief Interfaces for wrappers
namespace interface {
/// \brief Matches rclcpp_action::ServerGoalHandle to enable mocking.
/// \tparam T Action message type
template <typename T>
struct server_goal {
  /// \brief Send an update about the progress of a goal.
  virtual void publish_feedback(std::shared_ptr<typename T::Feedback>) = 0;
  /// \brief Indicate that a goal could not be reached and has been aborted.
  virtual void abort(std::shared_ptr<typename T::Result>) = 0;
  /// \brief Indicate that a goal has succeeded.
  virtual void succeed(std::shared_ptr<typename T::Result>) = 0;
  /// \brief Indicate that a goal has been canceled.
  virtual void canceled(std::shared_ptr<typename T::Result>) = 0;
  /// \brief Indicate that the server is starting to execute a goal.
  virtual void execute() = 0;
  /// \brief Get the user provided message describing the goal.
  virtual std::shared_ptr<typename T::Goal const> const get_goal() const = 0;
  /// \brief Get the unique identifier of the goal.
  virtual rclcpp_action::GoalUUID const &get_goal_id() const = 0;
  /// \brief Indicate if client has requested this goal be cancelled.
  virtual bool is_canceling() const = 0;
  /// \brief Indicate if goal is pending or executing.
  virtual bool is_active() const = 0;
  /// \brief Indicate if goal is executing.
  virtual bool is_executing() const = 0;
  /// \brief Virtual dtor is required for inheritance.
  virtual ~server_goal() = default;
};
}  // namespace interface
template <typename T>
using goal_callback = std::function<rclcpp_action::GoalResponse(
    rclcpp_action::GoalUUID const &, std::shared_ptr<typename T::Goal const>)>;

template <typename T>
using cancel_callback = std::function<rclcpp_action::CancelResponse(
    std::shared_ptr<interface::server_goal<T>> const)>;

template <typename T>
using accepted_callback =
    std::function<void(std::shared_ptr<interface::server_goal<T>> const)>;

namespace interface {
/// \brief Allows action server to be mocked
template <typename T>
struct action_server {
  /// \brief Allows action server to be instantiated outside consumer and
  ///        for the consumer to then pass in methods or lambdas as callbacks
  ///        Example:
  /// \code{.cpp}
  /// // test.cpp
  /// auto fake_server = std::make_unique<fake_action_server>();
  /// // decorate or define behavior for fake server
  /// auto consumer = consumer{std::move(fake_server)};
  /// // consumer.cpp
  /// consumer(std::unique_ptr<action_server<Message>> as)
  ///     : as_{std::move(as)} {
  ///       as_->register_handles(
  ///       [](auto const&, auto) {
  ///         return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  ///       },
  ///       [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
  ///       std::bind(&Consumer::handle_accepted, this, _1)
  ///       );
  /// \endcode
  virtual void register_handles(goal_callback<T>, cancel_callback<T>,
                                accepted_callback<T>) = 0;
  virtual ~action_server() = default;
};
}  // namespace interface

/// \brief Wrapper for server goal
/// \tparam T Action message type
template <typename T>
struct server_goal final : public interface ::server_goal<T> {
  explicit server_goal(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> handle)
      : handle_{std::move(handle)} {}
  void publish_feedback(
      std::shared_ptr<typename T::Feedback> feedback) override {
    return handle_->publish_feedback(std::move(feedback));
  }
  void abort(std::shared_ptr<typename T::Result> result) override {
    handle_->abort(std::move(result));
  }
  void succeed(std::shared_ptr<typename T::Result> result) override {
    handle_->succeed(std::move(result));
  }
  void canceled(std::shared_ptr<typename T::Result> result) override {
    handle_->canceled(std::move(result));
  }
  void execute() override { handle_->execute(); }
  std::shared_ptr<typename T::Goal const> const get_goal() const override {
    return handle_->get_goal();
  }
  rclcpp_action::GoalUUID const &get_goal_id() const override {
    return handle_->get_goal_id();
  }
  bool is_canceling() const override { return handle_->is_canceling(); }
  bool is_active() const override { return handle_->is_active(); }
  bool is_executing() const override { return handle_->is_executing(); }

 private:
  std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> handle_;
};

/// \brief Wrapper for action server
/// \tparam T Action message type
template <typename T>
struct action_server final : public interface::action_server<T> {
  action_server(std::shared_ptr<rclcpp::Node> node, std::string topic)
      : node_{std::move(node)}, topic_{std::move(topic)} {}

  void register_handles(goal_callback<T> goal, cancel_callback<T> cancel,
                        accepted_callback<T> accept) override {
    action_server_ = rclcpp_action::create_server<T>(
        node_, topic_, goal,
        [cancel](auto const goal_handle) {
          return cancel(
              std::make_shared<server_goal<T>>(std::move(goal_handle)));
        },
        [accept](auto const goal_handle) {
          accept(std::make_shared<server_goal<T>>(std::move(goal_handle)));
        });
  }

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::string topic_;
  std::shared_ptr<rclcpp_action::Server<T>> action_server_;
};

}  // namespace rcl_handle
