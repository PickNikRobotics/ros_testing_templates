// Description: Test if a simple task plan works

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <memory>

namespace minimal_integration_test {
class TaskPlanningFixture : public testing::Test {
 public:
  TaskPlanningFixture()
      : node_(std::make_shared<rclcpp::Node>("basic_test"))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }

  void SetUp() override {
    // Setup things that should occur before every test instance should go here
    RCLCPP_ERROR_STREAM(node_->get_logger(), "DONE WITH SETUP!!");
  }

  void TearDown() override {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "DONE WITH TEARDOWN!!");
  }

 protected:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TaskPlanningFixture, TrueIsTrueTest) {
  RCLCPP_ERROR_STREAM(node_->get_logger(), "TESTING!!");
  EXPECT_TRUE(true);
}
}  // namespace minimal_integration_test

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("basic_test"), "DONE SHUTTING DOWN ROS!!");
  return result;
}
