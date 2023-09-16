#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <example_srvs/srv/get_path.hpp>
#include <example_srvs/srv/set_map.hpp>
#include <gtest/gtest.h>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <tl_expected/expected.hpp>

// production_code.h/cc
// X, Y position
struct Position {
  size_t x;
  size_t y;
};

template <typename T>
class Map {
 public:
  Map(){};
  Map(std::vector<std::vector<T>> data) : data_{data} {};
  T at(Position const& pos) const { return data_.at(pos.y).at(pos.x); }
  std::vector<std::vector<T>>& get_data() { return data_; }
  const std::vector<std::vector<T>>& get_data() const { return data_; }
  bool empty() { return data_.empty(); }

 private:
  std::vector<std::vector<T>> data_;
};

using Path = std::vector<Position>;

// Operator overload to print path
std::ostream& operator<<(std::ostream& os, std::vector<Position> const& path) {
  for (auto const& pose : path) {
    os << "(" << pose.x << ", " << pose.y << ")\n";
  }
  return os;
}

// Operator overload for position comparison
bool operator==(Position const& lhs, Position const& rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

namespace pathing {

using PathingGeneratorFunctionType = std::function<std::optional<Path>(
    Position const&, Position const&, Map<unsigned char> const&)>;

/**
 * @brief      Generates a path
 *
 * @param      start          The start position
 * @param      goal           The goal position
 * @param      occupancy_map  The occupancy_map
 *
 * @return     std::optional containing the Path
 */
std::optional<Path> generate_global_path(
    Position const& start, Position const& goal,
    Map<unsigned char> const& occupancy_map) {  // Calculation
  // Some cool and nifty algorithm
  // What is the delta in position
  int const del_x = goal.x - start.x;
  int const del_y = goal.y - start.y;

  // What direction to move in for each dimension
  int const del_x_sign = std::copysign(1.0, del_x);
  int const del_y_sign = std::copysign(1.0, del_y);

  // Push start onto the path
  std::optional<Path> path;
  path.value().push_back(start);

  auto const is_occupied = [&occupancy_map](auto const x,
                                            auto const y) -> bool {
    return occupancy_map.at(Position{x, y}) == 255;
  };

  auto const repeat = [](size_t n, auto f) {
    while (n--) f();
  };

  auto const move = [&is_occupied, &path](int const x_dir, int const y_dir) {
    if (!path) {
      return;
    }
    if (is_occupied(path.value().back().x + x_dir,
                    path.value().back().y + y_dir)) {
      path = std::nullopt;
      return;
    }
    path.value().push_back(
        {path.value().back().x + x_dir, path.value().back().y + y_dir});
  };

  // Fails if there is any obstacle in the way
  // Move horizontally
  repeat(std::abs(del_x), [&move, &del_x_sign]() { move(del_x_sign, 0); });
  // Move vertically
  repeat(std::abs(del_y), [&move, &del_y_sign]() { move(0, del_y_sign); });

  return path;
}

namespace utilities {

std::optional<Map<unsigned char>> parseSetMapRequest(
    std::shared_ptr<example_srvs::srv::SetMap::Request> const request) {
  auto const occupancy_map = request->map;
  ;
  // Get the occupancy_map from a ros topic
  // Check that map layout makes sense
  if ((occupancy_map.layout.dim[0].size * occupancy_map.layout.dim[1].size) !=
      occupancy_map.layout.dim[0].stride) {
    return std::nullopt;
  }
  if (occupancy_map.layout.dim[0].stride != occupancy_map.data.size()) {
    return std::nullopt;
  }
  auto const begin = std::begin(occupancy_map.data);

  // Populate the map
  Map<unsigned char> map;
  for (size_t row = 0; row < occupancy_map.layout.dim[0].size; row++) {
    auto const row_beginning = begin + row * occupancy_map.layout.dim[1].size;
    auto const row_end = begin + (row + 1) * occupancy_map.layout.dim[1].size;
    map.get_data().push_back({row_beginning, row_end});
  }
  return map;
}

std_msgs::msg::UInt8MultiArray createUInt8MultiArrayMessageFromPath(
    const Path& path) {
  auto message = std_msgs::msg::UInt8MultiArray();

  message.layout.dim.resize(3, std_msgs::msg::MultiArrayDimension());

  message.layout.dim[0].label = "rows";
  message.layout.dim[0].size = path.size();
  message.layout.dim[0].stride = path.size() * 2;

  message.layout.dim[1].label = "columns";
  message.layout.dim[1].size = 2;
  message.layout.dim[1].stride = 1;

  message.layout.dim[2].label = "channel";
  message.layout.dim[2].size = 1;
  message.layout.dim[2].stride = 1;

  // Start pushing back the path only if there is one
  if (path.size() > 0) {
    for (auto const& position : path) {
      message.data.push_back(position.x);
      message.data.push_back(position.y);
    }
  }
  return message;
}
};  // namespace utilities
};  // namespace pathing

namespace generate_path {

/**
 * @brief      Types of errors expected in the generate path callback function
 */
enum class errors {
  NO_OCCUPANCY_MAP,
  INVALID_START_SIZE,
  INVALID_GOAL_SIZE,
  NO_VALID_PATH
};

/**
 * @brief      Gets the error description from the generate_path::errors error
 * type
 *
 * @param[in]  error  The error type
 *
 * @return     The error description.
 */
std::string get_error_description(generate_path::errors error) {
  switch (error) {
    case errors::NO_OCCUPANCY_MAP:
      return "The Occupancy Map is empty.";
    case errors::INVALID_START_SIZE:
      return "The start field in the request is not of size 2.";
    case errors::INVALID_GOAL_SIZE:
      return "The goal field in the request is not of size 2.";
    case errors::NO_VALID_PATH:
      return "There is no valid path between the start and goal.";
    default:
      return "No description for given error type";
  }
}

tl::expected<std::shared_ptr<example_srvs::srv::GetPath::Response>,
             generate_path::errors>
callback(std::shared_ptr<example_srvs::srv::GetPath::Request> const request,
         Map<unsigned char> const& occupancy_map,
         pathing::PathingGeneratorFunctionType path_generater) {
  if (occupancy_map.get_data().size() == 0) {
    return tl::unexpected(errors::NO_OCCUPANCY_MAP);
  }
  // Check to make sure start and goal fields of the request are of size 2
  if (request->start.data.size() != 2) {
    return tl::unexpected(errors::INVALID_START_SIZE);
  }
  if (request->goal.data.size() != 2) {
    return tl::unexpected(errors::INVALID_GOAL_SIZE);
  }

  auto const start = Position{request->start.data[0], request->start.data[1]};
  auto const goal = Position{request->goal.data[0], request->goal.data[1]};

  // Generate the path using the path generator function that was input
  auto const path = path_generater(start, goal, occupancy_map);

  if (!path.has_value()) {
    return tl::unexpected(errors::NO_VALID_PATH);
  }

  auto const response =
      std::make_shared<example_srvs::srv::GetPath::Response>();
  response->success.data = path.has_value();
  response->path =
      pathing::utilities::createUInt8MultiArrayMessageFromPath(path.value());

  return response;
}

};  // namespace generate_path

struct MainObject {
  MainObject()
      : set_costmap_wrapper_{[this](const std::shared_ptr<
                                        example_srvs::srv::SetMap::Request>
                                        request,
                                    std::shared_ptr<
                                        example_srvs::srv::SetMap::Response>
                                        response) -> void {
          const auto path = pathing::utilities::parseSetMapRequest(request);
          if (path) this->map_ = path.value();
          response->success.data = path.has_value();
        }},
        generate_path_wrapper_{
            [this](
                const std::shared_ptr<example_srvs::srv::GetPath::Request>
                    request,
                std::shared_ptr<example_srvs::srv::GetPath::Response> response)
                -> void {
              auto const print_error = [](std::string error)
                  -> tl::expected<
                      std::shared_ptr<example_srvs::srv::GetPath::Response>,
                      std::string> {
                std::cout << error << "\n";
                return tl::expected<
                    std::shared_ptr<example_srvs::srv::GetPath::Response>,
                    std::string>(tl::unexpect, std::string()); // Need to tell linter to skip this line
              };

              auto const return_empty_response = [](std::string /* not used */)
                  -> tl::expected<
                      std::shared_ptr<example_srvs::srv::GetPath::Response>,
                      std::string> {
                auto response =
                    std::make_shared<example_srvs::srv::GetPath::Response>();
                response->success.data = false;
                response->path = std_msgs::msg::UInt8MultiArray();
                return tl::expected<
                    std::shared_ptr<example_srvs::srv::GetPath::Response>,
                    std::string>(response);
              };

              response = generate_path::callback(request, this->map_,
                                                 pathing::generate_global_path)
                             .map_error(generate_path::get_error_description)
                             .or_else(print_error)
                             .or_else(return_empty_response)
                             .value();
            }} {}

  std::function<void(const std::shared_ptr<example_srvs::srv::SetMap::Request>,
                     std::shared_ptr<example_srvs::srv::SetMap::Response>)>
      set_costmap_wrapper_;

  std::function<void(const std::shared_ptr<example_srvs::srv::GetPath::Request>,
                     std::shared_ptr<example_srvs::srv::GetPath::Response>)>
      generate_path_wrapper_;

  Map<unsigned char> map_;
};

int main(int argc, char** argv) {
  // Turned everything into a pure function
  // Used a closure to capture the costmap (Is it a true closure?)
  // Using a higher order function to change the functionality of the generate
  // path callback Using monads (hopefully) to process errors

  MainObject mo;

  rclcpp::init(argc, argv);

  const auto node = std::make_shared<rclcpp::Node>("PathGenerator");

  rclcpp::Service<example_srvs::srv::SetMap>::SharedPtr set_map_service =
      node->create_service<example_srvs::srv::SetMap>("set_costmap",
                                                      mo.set_costmap_wrapper_);

  rclcpp::Service<example_srvs::srv::GetPath>::SharedPtr get_path_service =
      node->create_service<example_srvs::srv::GetPath>(
          "generate_global_path", mo.generate_path_wrapper_);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
