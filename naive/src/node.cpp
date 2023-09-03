#include "naive/incrementer.hpp"
#include "naive/ros_middleware.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("incrementer");
  Incrementer incrementer{std::make_unique<RosMiddleware>(node, "/in", "/out")};
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
