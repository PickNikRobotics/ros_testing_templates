// C++ Standard Library
#include <memory>

// Boost
#include <boost/make_shared.hpp>

// Gtest
#include <gmock/gmock.h>
#include <gtest/gtest.h>

// ROS
#include <std_msgs/Int64.h>

// Code being tested
#include <publisher_subscriber/incrementer.h>

using ::testing::_;
using ::testing::Invoke;

/**
 \brief Mock middleware handle, allowing us to define test behavior.
 */
struct MockMiddleware : public Incrementer::MiddlewareHandle {
  MOCK_METHOD1(registerCallback, void(Callback cb));
  MOCK_METHOD1(publish, void(std_msgs::Int64 msg));
};

/** \brief Callback is given to subscriber on construction. */
TEST(IncrementerTests, RegisterSubscriber) {
  // GIVEN a middleware handle
  auto mw = std::make_unique<MockMiddleware>();
  // THEN a callback should be registered for topic subscription.
  EXPECT_CALL(*mw, registerCallback(_)).Times(1);
  // WHEN the incrementer is created with that handle
  Incrementer incrementer{std::move(mw)};
}

/** \brief Incremented number is published when a message is received. */
TEST(IncrementerTests, PublishIncrement) {
  // GIVEN a middleware handle
  auto mw = std::make_unique<MockMiddleware>();
  /*
  Configure the mock handle to capture the incrementer callback so we can
  publish a message to it as the test trigger event. This is analogous to ROS
  publishing a message to a topic if the ROS middleware handle was used.
  We know that registerCallback will be called from a previous test so we
  do not need to test that again, only define the behavior.
  */
  Incrementer::MiddlewareHandle::Callback callback;
  ON_CALL(*mw, registerCallback(_))
      .WillByDefault(Invoke(
          [&](Incrementer::MiddlewareHandle::Callback cb) { callback = cb; }));

  // THEN the incrementer should publish an incremented message.
  std_msgs::Int64 expected;
  expected.data = 8;
  EXPECT_CALL(*mw, publish(expected)).Times(1);
  /* Create the System Under Test (SUT)*/
  Incrementer incrementer{std::move(mw)};

  // WHEN the middleware publishes a message to the callback
  std_msgs::Int64::Ptr msg = boost::make_shared<std_msgs::Int64>();
  msg->data = 7;
  callback(msg);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
