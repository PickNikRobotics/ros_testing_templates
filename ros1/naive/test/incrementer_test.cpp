// C++ Standard Library
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

// Boost
#include <boost/make_shared.hpp>

// Gtest
#include <gtest/gtest.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Int64.h>

// Code being tested
#include <naive/incrementer.h>

using namespace std::chrono_literals;

class Spinner {
 public:
  /**
   \brief Launches a thread that spins ROS.
          This is usually used to ensure that callbacks are processed.
   */
  Spinner()
      : spin{[this] {
          while (!done && ros::ok()) {
            ros::spinOnce();
            ros::Duration(0.01).sleep();
          }
        }} {}

  ~Spinner() {
    done = true;
    spin.join();
  }

 private:
  std::atomic<bool> done{false};  ///< Flag used to join thread.
  std::thread spin;               ///< Thread to spin ROS.
};

/** \brief Incremented number is published when a message is received. */
TEST(IncrementerTests, PublishIncrementSpinnerClass) {
  // GIVEN an incrementer
  ros::NodeHandle nh;
  Incrementer incrementer{nh, "/in", "/out"};

  /* Create a spinner to make sure callbacks are processed. */
  Spinner spin;
  /* Create a subscriber to capture the message published by incrementer so
     we can check the result.
  */
  std::condition_variable cv;
  std_msgs::Int64::Ptr result;
  ros::Subscriber sub =
      nh.subscribe<std_msgs::Int64>("/out", 0, [&](const auto& msg) {
        result = boost::make_shared<std_msgs::Int64>(*msg);
        cv.notify_one();
      });
  EXPECT_EQ(sub.getNumPublishers(), 1);

  // WHEN a message is published
  ros::Publisher pub = nh.advertise<std_msgs::Int64>("/in", 0);
  EXPECT_EQ(pub.getNumSubscribers(), 1);
  std_msgs::Int64 msg;
  msg.data = 7;
  pub.publish(msg);

  /* Wait until we receive a result or time out. */
  std::mutex m;
  std::unique_lock<std::mutex> lock{m};
  cv.wait_for(lock, 10s, [&] { return result != nullptr; });

  // THEN the incrementer should publish an incremented message.
  EXPECT_EQ(result->data, 8);
}

/** \brief Incremented number is published when a message is received. */
TEST(IncrementerTests, PublishIncrementInlineSpin) {
  // GIVEN an incrementer
  ros::NodeHandle nh;
  Incrementer incrementer{nh, "/in", "/out"};

  /* Create a subscriber to capture the published message so we can
     check the result from the incrementer.
     This could be put in a GTest fixture to as it's a test implementation
     detail that might detract from what is being tested.
  */
  std_msgs::Int64::Ptr result;
  ros::Subscriber sub =
      nh.subscribe<std_msgs::Int64>("/out", 0, [&](const auto& msg) {
        result = boost::make_shared<std_msgs::Int64>(*msg);
      });
  EXPECT_EQ(sub.getNumPublishers(), 1);

  // WHEN a message is published
  ros::Publisher pub = nh.advertise<std_msgs::Int64>("/in", 0);
  EXPECT_EQ(pub.getNumSubscribers(), 1);
  std_msgs::Int64 msg;
  msg.data = 7;
  pub.publish(msg);

  /* Spin the thread until a result is received or we "time out". */
  {
    size_t count = 0;  // Number of times to check before giving up.
    while (result == nullptr && ros::ok() && count < 100) {
      ros::spinOnce();
      ros::Duration(0.01).sleep();
      count++;
    }
    /* Fail the test if we left the loop without getting a result message */
    if (result == nullptr) FAIL();
  }

  // THEN the incrementer should publish an incremented message.
  EXPECT_EQ(result->data, 8);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "naive_test");
  return RUN_ALL_TESTS();
}
