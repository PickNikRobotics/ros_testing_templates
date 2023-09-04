#include "naive_superclass/incrementer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<Incrementer>("incrementer", "in", "out");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
