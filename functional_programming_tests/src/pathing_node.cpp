#include "pathing/pathing_manager.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <example_srvs/srv/get_path.hpp>
#include <example_srvs/srv/set_map.hpp>

using SetMap = example_srvs::srv::SetMap;
using GetPath = example_srvs::srv::GetPath;

struct RosMiddleware : public pathing::Manager::MiddlewareHandle {
  /**
   * @brief Construct a new Rosful Pathing Manager middleware handle
   * @param node to use for the ROS services
   */
  RosMiddleware(std::shared_ptr<rclcpp::Node> node) : node_{std::move(node)} {}

  void register_set_map_service(SetMapCallback callback) override {
    map_setter_ = node_->create_service<SetMap>(
        "set_costmap",
        [callback](std::shared_ptr<SetMap::Request> const request,
                   std::shared_ptr<SetMap::Response> response) {
          callback(request, response);
        });
  }
  void register_generate_path_service(GeneratePathCallback callback) override {
    path_generator_ = node_->create_service<GetPath>(
        "generate_global_path",
        [callback](std::shared_ptr<GetPath::Request> const request,
                   std::shared_ptr<GetPath::Response> response) {
          callback(request, response);
        });
  }

  void log_info(std::string const& msg) override {
    RCLCPP_INFO_STREAM(node_->get_logger(), msg);
  }

  void log_error(std::string const& msg) override {
    RCLCPP_ERROR_STREAM(node_->get_logger(), msg);
  }

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Service<SetMap>> map_setter_;
  std::shared_ptr<rclcpp::Service<GetPath>> path_generator_;
};

int main(int argc, char** argv) {
  // Turned everything into a pure function
  // Used a closure to capture the costmap (Is it a true closure?)
  // Using a higher order function to change the functionality of the generate
  // path callback Using monads (hopefully) to process errors

  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("PathGenerator");
  pathing::Manager pm{std::make_unique<RosMiddleware>(node)};

  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
