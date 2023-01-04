// Description: Test if a simple task plan works

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

namespace minimal_integration_test {
class TaskPlanningFixture : public testing::Test {
 public:
  // Setup things that should occur before every test instance should go here
  // https://google.github.io/googletest/faq.html#CtorVsSetUp
  TaskPlanningFixture() : node_(std::make_shared<rclcpp::Node>("basic_test")) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }

  // Things that can't be done in the ctor should go here, such as calling
  // ASSERT_XX macros in the fixture vs the TEST_F
  void SetUp() override {
    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH SETUP!!");
  }

  // Place most cleanup actions here, if needed
  ~TaskPlanningFixture() {
    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH DESTRUCTOR!!");
  }

  // Cleanup actions that could throw an exception
  void TearDown() override {
    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH TEARDOWN!!");
  }

 protected:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TaskPlanningFixture, TrueIsTrueTest) {
  RCLCPP_INFO_STREAM(node_->get_logger(), "TESTING!!");
  EXPECT_TRUE(true);
}
}  // namespace minimal_integration_test

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("basic_test"),
                      "DONE SHUTTING DOWN ROS WITH RESULT " << result << "!!");
  return result;
}
