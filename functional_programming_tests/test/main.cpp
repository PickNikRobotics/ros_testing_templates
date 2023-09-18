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

using SetMap = example_srvs::srv::SetMap;
using GetPath = example_srvs::srv::GetPath;

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

  /**
   * @brief      Checks if the map is empty
   * @return     bool indicating if the map is empty
   */
  bool empty() const { return data_.empty(); }

  /**
   * @brief      Gets the dimensions of the map
   * @return     std::pair containing the shape of the map
   *            first: number of rows
   *            second: number of columns
   */
  std::pair<size_t, size_t> shape() const {
    return std::make_pair(data_.size(), data_.at(0).size());
  }

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

using PathingGenerator = std::function<std::optional<Path>(
    Position const&, Position const&, Map<unsigned char> const&)>;

/**
 * @brief      Generates a path
 *
 * @param      start position
 * @param      goal position
 * @param      occupancy_map to path through
 *
 * @return     std::optional containing the Path
 */
std::optional<Path> generate_global_path(
    Position const& start, Position const& goal,
    Map<unsigned char> const& occupancy_map) {  // Calculation
  // Pre-checks
  if (occupancy_map.empty()) {
    return std::nullopt;
  }
  // Get the dimensions of the map for bounds checking
  auto const [dim_x, dim_y] = occupancy_map.shape();

  if (start.x > dim_x) {
    return std::nullopt;
  }
  if (start.y > dim_y) {
    return std::nullopt;
  }
  if (goal.x > dim_x) {
    return std::nullopt;
  }
  if (goal.y > dim_y) {
    return std::nullopt;
  }
  // What is the delta in position
  int const del_x = goal.x - start.x;
  int const del_y = goal.y - start.y;

  // What direction to move in for each dimension
  int const del_x_sign = std::copysign(1.0, del_x);
  int const del_y_sign = std::copysign(1.0, del_y);

  // Push start onto the path
  std::optional<Path> path = Path{start};

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

/**
 * @brief      Converts map from message to occupancy map if possible
 *
 * @param[in]  request containing the map
 *
 * @return     std::optional containing the occupancy map
 */
std::optional<Map<unsigned char>> parseSetMapRequest(
    std::shared_ptr<SetMap::Request> const request) {
  if (request->map.layout.dim.empty() || request->map.data.empty()) {
    return std::nullopt;
  }
  // Check that map layout makes sense
  if ((request->map.layout.dim[0].size * request->map.layout.dim[1].size) !=
      request->map.layout.dim[0].stride) {
    return std::nullopt;
  }
  if (request->map.layout.dim[0].stride != request->map.data.size()) {
    return std::nullopt;
  }

  auto const occupancy_map = request->map;
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

/**
 * @brief      Creates a UInt8MultiArray message from a path
 *
 * @param[in]  path to convert
 *
 * @return     The UInt8MultiArray message
 */
std_msgs::msg::UInt8MultiArray createUInt8MultiArrayMessageFromPath(
    Path const& path) {
  if (path.empty()) {
    return std_msgs::msg::UInt8MultiArray();
  }
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
enum class error {
  NO_OCCUPANCY_MAP,
  INVALID_START_SIZE,
  INVALID_GOAL_SIZE,
  NO_VALID_PATH
};

/**
 * @brief      Descriptions of the errors
 */
std::map<error, std::string> const error_description = {
    {error::NO_OCCUPANCY_MAP, "The Occupancy Map is empty."},
    {error::INVALID_START_SIZE,
     "The start field in the request is not of size 2."},
    {error::INVALID_GOAL_SIZE,
     "The goal field in the request is not of size 2."},
    {error::NO_VALID_PATH,
     "There is no valid path between the start and goal."}};

/**
 * @brief      Converts between types and dispatches to path generator
 *
 * @param[in]  request contains the start and goal positions
 * @param[in]  occupancy_map to path through
 * @param[in]  path_generator algorithm to use for pathing
 *
 * @return     The path if successful, otherwise an error
 */
tl::expected<GetPath::Response, error> generate_path(
    std::shared_ptr<GetPath::Request> const request,
    Map<unsigned char> const& occupancy_map,
    pathing::PathingGenerator path_generator) {
  if (occupancy_map.get_data().size() == 0) {
    return tl::unexpected(error::NO_OCCUPANCY_MAP);
  }
  // Check to make sure start and goal fields of the request are of size 2
  if (request->start.data.size() != 2) {
    return tl::unexpected(error::INVALID_START_SIZE);
  }
  if (request->goal.data.size() != 2) {
    return tl::unexpected(error::INVALID_GOAL_SIZE);
  }

  auto const start = Position{request->start.data[0], request->start.data[1]};
  auto const goal = Position{request->goal.data[0], request->goal.data[1]};

  // Generate the path using the path generator function that was input
  auto const path = path_generator(start, goal, occupancy_map);
  if (!path.has_value()) {
    return tl::unexpected(error::NO_VALID_PATH);
  }

  auto response = GetPath::Response{};
  response.success.data = path.has_value();
  response.path =
      pathing::utilities::createUInt8MultiArrayMessageFromPath(path.value());

  return response;
}

};  // namespace generate_path

struct PathingManager {
  struct MiddlewareHandle {
    // Define map service callback type
    using SetMapCallback =
        std::function<void(std::shared_ptr<SetMap::Request> const request,
                           std::shared_ptr<SetMap::Response> response)>;

    // Define path generation service callback type
    using GeneratePathCallback =
        std::function<void(std::shared_ptr<GetPath::Request> const request,
                           std::shared_ptr<GetPath::Response> response)>;
    /**
     * @brief Ensure public dtor is virtual
     * @note
     * http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rc-dtor-virtual
     */
    virtual ~MiddlewareHandle() = default;

    /**
     * @brief Register a callback for the set map service
     * @param callback The callback to register
     */
    virtual void register_set_map_service(SetMapCallback callback) = 0;

    /**
     * @brief Register a callback for the generate path service
     * @param callback The callback to register
     */
    virtual void register_generate_path_service(
        GeneratePathCallback callback) = 0;

    /**
     * @brief Log error message
     * @param msg The message to log
     */
    virtual void log_error(std::string const& msg) = 0;

    /**
     * @brief Log info message
     * @param msg The message to log
     */
    virtual void log_info(std::string const& msg) = 0;
  };

  /**
   * @brief Manages occupancy_map and path generation for the pathing node
   * @param mw The middleware handle for interacting with services
   */
  PathingManager(std::unique_ptr<MiddlewareHandle> mw) : mw_{std::move(mw)} {
    mw_->register_set_map_service(
        [this](auto const request, auto response) -> void {
          auto const path = pathing::utilities::parseSetMapRequest(request);
          if (path) this->map_ = path.value();
          response->success.data = path.has_value();
        });
    mw_->register_generate_path_service(
        [this](auto const request, auto response) {
          auto const print_error = [this](std::string_view error)
              -> tl::expected<GetPath::Response, std::string> {
            mw_->log_error(std::string{error});
            return tl::make_unexpected("");
          };

          auto const return_empty_response = []([[maybe_unused]] auto const)
              -> tl::expected<GetPath::Response, std::string> {
            auto response = GetPath::Response{};
            response.success.data = false;
            response.path = std_msgs::msg::UInt8MultiArray();
            return response;
          };
          auto const stringify_error = [](auto const error) {
            return generate_path::error_description.at(error);
          };
          *response = generate_path::generate_path(
                          request, this->map_, pathing::generate_global_path)
                          .map_error(stringify_error)
                          .or_else(print_error)
                          .or_else(return_empty_response)
                          .value();
        });
  }

  std::unique_ptr<MiddlewareHandle> mw_;
  Map<unsigned char> map_;
};

struct RosMiddleware : public PathingManager::MiddlewareHandle {
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
  PathingManager pm{std::make_unique<RosMiddleware>(node)};

  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
