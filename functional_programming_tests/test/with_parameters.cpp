#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <example_srvs/srv/get_path.hpp>
#include <example_srvs/srv/set_map.hpp>
#include <gtest/gtest.h>
#include <std_msgs/msg/u_int8_multi_array.hpp>

using namespace std::chrono_literals;

// TODO: Add more comments

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
  std::vector<std::vector<T>> const& get_data() const { return data_; }
  std::pair<int, int> dim() const {
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

template <typename T>
struct CollisionChecker {
  /**
   * @brief Checks if the robot is colliding with an obstacle or out of bounds
   * @param robot_size The size of the robot in pixels
   */
  CollisionChecker(int robot_size) : robot_size_{robot_size} {}

  /**
   * @brief Checks if the robot is colliding with an obstacle or out of bounds
   * @param map The map to check for collision
   * @param x position of the center of robot
   * @param y position of the center of robot
   * TODO: This check doesn't make any physical sense. The robot should be
   *       checked radially from the center of the robot. This is a square
   *       starting from the corner of the robot as the origin.
   * @return True if the robot is colliding with an obstacle or out of bounds
   */
  bool operator()(Map<T> const& map, auto const x, auto const y) const {
    // Get the dimensions of the map for bounds checking
    auto const [dim_x, dim_y] = map.dim();

    // Check if the robot is out of bounds or colliding with an obstacle
    for (int dx = 0; dx < robot_size_; ++dx) {
      for (int dy = 0; dy < robot_size_; ++dy) {
        // Get the position of the robot to check for collision
        int const px = x + dx;
        int const py = y + dy;

        // Check if the robot is out of bounds
        if (px < 0 || px >= dim_x || py < 0 || py >= dim_y) {
          return true;
        }
        // Check if the robot is colliding with an obstacle
        if (map.at(Position{x + dx, y + dy}) == 255) {
          return true;
        }
      }
    }
    return false;
  }

 private:
  int robot_size_;
};

class PathGenerator : public rclcpp::Node {
 public:
  explicit PathGenerator(
      rclcpp::NodeOptions const& options = rclcpp::NodeOptions{})
      : Node("path_generator", options) {
    robot_size_ = this->declare_parameter<int>("robot_size", 1);
    RCLCPP_INFO(this->get_logger(), "Robot size: %d", robot_size_);
    is_occupied_ =
        std::make_unique<CollisionChecker<unsigned char>>(robot_size_);

    // Services for setting the map and generating the path
    map_setter_service_ = this->create_service<example_srvs::srv::SetMap>(
        "set_costmap", std::bind(&PathGenerator::set_map_service, this,
                                 std::placeholders::_1, std::placeholders::_2));
    path_generator_service_ = this->create_service<example_srvs::srv::GetPath>(
        "generate_global_path",
        std::bind(&PathGenerator::generate_path_service, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

 private:
  void set_map_service(
      const std::shared_ptr<example_srvs::srv::SetMap::Request> request,
      std::shared_ptr<example_srvs::srv::SetMap::Response> response) {
    // Set the map to generate the path from
    response->success.data = set_costmap(request->map);
  }

  void generate_path_service(
      const std::shared_ptr<example_srvs::srv::GetPath::Request> request,
      std::shared_ptr<example_srvs::srv::GetPath::Response> response) {
    if (map_.get_data().size() == 0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "MAP IS EMPTY!!");
      response->success.data = false;
      response->path = std_msgs::msg::UInt8MultiArray();
      return;
    }
    // Check to make sure start and goal fields of the request are of size 2
    if (request->start.data.size() != 2) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "START POSITION MUST CONTAIN TWO ELEMENTS!!");
      response->success.data = false;
      response->path = std_msgs::msg::UInt8MultiArray();
      return;
    }
    if (request->goal.data.size() != 2) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "GOAL POSITION MUST CONTAIN TWO ELEMENTS!!");
      response->success.data = false;
      response->path = std_msgs::msg::UInt8MultiArray();
      return;
    }
    auto const start = Position{request->start.data[0], request->start.data[1]};
    auto const goal = Position{request->goal.data[0], request->goal.data[1]};

    // Generate the path
    auto const path = generate_global_path(start, goal);

    // Start populating the response message
    auto response_path = std_msgs::msg::UInt8MultiArray();

    response_path.layout.dim.resize(3, std_msgs::msg::MultiArrayDimension());

    response_path.layout.dim[0].label = "rows";
    response_path.layout.dim[0].size = path.size();
    response_path.layout.dim[0].stride = path.size() * 2;

    response_path.layout.dim[1].label = "columns";
    response_path.layout.dim[1].size = 2;
    response_path.layout.dim[1].stride = 1;

    response_path.layout.dim[2].label = "channel";
    response_path.layout.dim[2].size = 1;
    response_path.layout.dim[2].stride = 1;

    // Start pushing back the path only if there is one
    if (path.size() > 0) {
      for (auto const& position : path) {
        response_path.data.push_back(position.x);
        response_path.data.push_back(position.y);
      }
    }

    response->success.data = !path.empty();
    response->path = response_path;
  }

  // Function that sets costmap
  bool set_costmap(const std_msgs::msg::UInt8MultiArray& costmap) {  // Action
    // Get the costmap from a ros topic
    // Check that map layout makes sense
    if ((costmap.layout.dim[0].size * costmap.layout.dim[1].size) !=
        costmap.layout.dim[0].stride) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "COSTMAP DIMENSIONS AND STRIDE INCONSISTENT!!");
      return false;
    }
    if (costmap.layout.dim[0].stride != costmap.data.size()) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "COSTMAP LENGTH AND STRIDE INCONSISTENT!!");
      return false;
    }
    auto const begin = std::begin(costmap.data);
    // Populate the map
    for (size_t row = 0; row < costmap.layout.dim[0].size; row++) {
      map_.get_data().push_back(
          {begin + row * costmap.layout.dim[1].size,
           begin + (row + 1) * costmap.layout.dim[1].size});
    }
    return true;
  }

  // From the start and goal, generate a trajectory (Deterministic calculation)
  Path generate_global_path(Position const& start,
                            Position const& goal) {  // Calculation
    // Some cool and nifty algorithm
    // What is the delta in position
    int const del_x = goal.x - start.x;
    int const del_y = goal.y - start.y;

    // What direction to move in for each dimension
    int const del_x_sign = std::copysign(1.0, del_x);
    int const del_y_sign = std::copysign(1.0, del_y);

    // Push start onto the path
    Path path;
    path.push_back(start);

    // Fails if there is any obstacle in the way
    // Move horizontally
    // x limit is reduced by half of the robot size to prevent the
    // algorithm from checking collision outside of the map
    auto const x_limit = std::abs(del_x) - std::floor(robot_size_ / 2);
    for (size_t i = 0; i < x_limit; ++i) {
      if ((*is_occupied_)(map_, path.back().x + del_x_sign, path.back().y)) {
        std::cout << "Failed horizontally at " << path.back().x + del_x_sign
                  << " " << path.back().y << "\n";
        return {};
      }
      path.push_back({path.back().x + del_x_sign, path.back().y});
    }
    // Move vertically
    auto const y_limit = std::abs(del_y) - std::floor(robot_size_ / 2);
    for (size_t i = 0; i < y_limit; i++) {
      if ((*is_occupied_)(map_, path.back().x, path.back().y + del_y_sign)) {
        std::cout << "Failed vertically at " << path.back().x << " "
                  << path.back().y + del_y_sign << "\n";
        return {};
      }
      path.push_back({path.back().x, path.back().y + del_y_sign});
    }

    return path;
  }

  Map<unsigned char> map_;
  int robot_size_;
  std::unique_ptr<CollisionChecker<unsigned char>> is_occupied_;
  rclcpp::Service<example_srvs::srv::SetMap>::SharedPtr map_setter_service_;
  rclcpp::Service<example_srvs::srv::GetPath>::SharedPtr
      path_generator_service_;
};

std::vector<Position> parseGeneratedPath(
    const std_msgs::msg::UInt8MultiArray& msg) {
  // Check the path has an even number of elements
  if ((msg.data.size() % 2) != 0) {
    return {};
  }
  if (msg.data.size() == 0) {
    return {};
  }
  std::vector<Position> path;

  for (size_t idx = 0; idx < (msg.data.size() - 1); idx += 2) {
    path.push_back({msg.data[idx], msg.data[idx + 1]});
  }

  return path;
}

class TaskPlanningFixture : public testing::Test {
 protected:
  // Adapted from minimal_integration_test
  TaskPlanningFixture() : node_(std::make_shared<rclcpp::Node>("test_client")) {
    // Create ROS2 clients to set the map and calculate the path
    map_setter_client_ =
        node_->create_client<example_srvs::srv::SetMap>("set_costmap");
    path_generator_client_ = node_->create_client<example_srvs::srv::GetPath>(
        "generate_global_path");
  }

  rclcpp::FutureReturnCode populateAndSetMap() {
    auto const request = std::make_shared<example_srvs::srv::SetMap::Request>();

    request->map = std_msgs::msg::UInt8MultiArray();

    request->map.layout.dim.resize(3, std_msgs::msg::MultiArrayDimension());

    request->map.layout.dim[0].label = "rows";
    request->map.layout.dim[0].size = 8;
    request->map.layout.dim[0].stride = 64;

    request->map.layout.dim[1].label = "columns";
    request->map.layout.dim[1].size = 8;
    request->map.layout.dim[1].stride = 1;

    request->map.layout.dim[2].label = "channel";
    request->map.layout.dim[2].size = 1;
    request->map.layout.dim[2].stride = 1;

    request->map.data = {0, 0, 0,   0,   0,   0,   0, 0,  //
                         0, 0, 0,   0,   0,   0,   0, 0,  //
                         0, 0, 0,   255, 0,   0,   0, 0,  //
                         0, 0, 255, 255, 255, 0,   0, 0,  //
                         0, 0, 255, 0,   255, 255, 0, 0,  //
                         0, 0, 255, 0,   0,   0,   0, 0,  //
                         0, 0, 0,   0,   0,   0,   0, 0,  //
                         0, 0, 0,   0,   0,   0,   0, 0};

    while (!map_setter_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR_STREAM(
            node_->get_logger(),
            "Interrupted while waiting for map setter service. Exiting.");
        return rclcpp::FutureReturnCode::TIMEOUT;
      }
      RCLCPP_INFO_STREAM(node_->get_logger(),
                         "Map setter service not available, waiting again...");
    }

    auto const set_map_result = map_setter_client_->async_send_request(request);

    return rclcpp::spin_until_future_complete(node_, set_map_result);
  }

  std::pair<example_srvs::srv::GetPath::Response::SharedPtr,
            rclcpp::FutureReturnCode>
  sendPathRequest(
      const example_srvs::srv::GetPath::Request::SharedPtr request) {
    while (!path_generator_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR_STREAM(
            node_->get_logger(),
            "Interrupted while waiting for path generator service. Exiting.");
        return {std::make_shared<example_srvs::srv::GetPath::Response>(),
                rclcpp::FutureReturnCode::TIMEOUT};
      }
      RCLCPP_INFO_STREAM(
          node_->get_logger(),
          "Path generator service not available, waiting again...");
    }

    auto generate_path_result =
        path_generator_client_->async_send_request(request);

    return std::make_pair(
        generate_path_result.get(),
        rclcpp::spin_until_future_complete(node_, generate_path_result));
  }

  // Member variables
  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<example_srvs::srv::SetMap>::SharedPtr map_setter_client_;
  rclcpp::Client<example_srvs::srv::GetPath>::SharedPtr path_generator_client_;
};

TEST_F(TaskPlanningFixture, same_start_and_goal) {
  // The PathGenerator class to test
  auto const pg = std::make_shared<PathGenerator>();
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(pg);
  auto executor_thread = std::thread([&executor]() { executor->spin(); });

  // Starts processing requests to the PathGenerator service in a separate
  // thread
  // GIVEN a populated costmap that is set without error
  auto const return_code = populateAndSetMap();

  EXPECT_EQ(return_code, rclcpp::FutureReturnCode::SUCCESS)
      << "Setting the map failed";

  // WHEN a path with the same start and goal is requested
  auto const request = std::make_shared<example_srvs::srv::GetPath::Request>();

  request->start.data = {0, 0};
  request->goal.data = {0, 0};

  auto const result = sendPathRequest(request);

  // THEN the global path should have been successfully produced
  // AND the path should have one element, which is the
  // start/goal position
  EXPECT_EQ(result.second, rclcpp::FutureReturnCode::SUCCESS)
      << "Generating path failed";

  std::vector<Position> const expected{{0, 0}};
  EXPECT_EQ(result.first->success.data, true) << result.first->success.data;
  EXPECT_EQ(parseGeneratedPath(result.first->path), expected)
      << parseGeneratedPath(result.first->path);

  executor->cancel();
  executor_thread.join();
}

TEST_F(TaskPlanningFixture, no_path) {
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  std::thread executor_thread;

  auto const pg = std::make_shared<PathGenerator>();

  executor->add_node(pg);
  executor_thread = std::thread([&executor]() { executor->spin(); });

  // GIVEN a populated costmap that is set without error
  auto const return_code = populateAndSetMap();

  EXPECT_EQ(return_code, rclcpp::FutureReturnCode::SUCCESS)
      << "Setting the map failed";

  // WHEN a path is requested between two positions that do not have a valid
  // path between them given the algorithm

  auto const request = std::make_shared<example_srvs::srv::GetPath::Request>();

  request->start.data = {2, 2};
  request->goal.data = {5, 5};

  auto const result = sendPathRequest(request);

  EXPECT_EQ(result.second, rclcpp::FutureReturnCode::SUCCESS)
      << "Generating path failed";

  // THEN the global path produced should be empty
  std::vector<Position> const expected{};
  EXPECT_EQ(result.first->success.data, false) << result.first->success.data;
  EXPECT_EQ(parseGeneratedPath(result.first->path), expected)
      << parseGeneratedPath(result.first->path);

  executor->cancel();
  executor_thread.join();
}

TEST_F(TaskPlanningFixture, path_generated) {
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  // The PathGenerator class to test
  auto const pg = std::make_shared<PathGenerator>();
  executor->add_node(pg);

  auto executor_thread = std::thread([&executor]() { executor->spin(); });
  // GIVEN a populated costmap that is set without error
  auto const return_code = populateAndSetMap();

  EXPECT_EQ(return_code, rclcpp::FutureReturnCode::SUCCESS)
      << "Setting the map failed";

  // WHEN a path is requested between two positions that do have a valid path
  // between them given the algorithm

  auto const request = std::make_shared<example_srvs::srv::GetPath::Request>();

  request->start.data = {0, 0};
  request->goal.data = {7, 7};

  auto const result = sendPathRequest(request);

  EXPECT_EQ(result.second, rclcpp::FutureReturnCode::SUCCESS)
      << "Generating path failed";

  // THEN the global path produced should be the same as the expected set of
  // Positions
  std::vector<Position> const expected{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0},
                                       {5, 0}, {6, 0}, {7, 0}, {7, 1}, {7, 2},
                                       {7, 3}, {7, 4}, {7, 5}, {7, 6}, {7, 7}};
  EXPECT_EQ(result.first->success.data, true) << result.first->success.data;
  EXPECT_EQ(parseGeneratedPath(result.first->path), expected)
      << parseGeneratedPath(result.first->path);
  executor->cancel();
  executor_thread.join();
}

TEST_F(TaskPlanningFixture, large_footprint_no_path) {
  // Creare executor to spin the node in a separate thread
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // GIVEN a PathGenerator node with a large robot footprint
  auto options = rclcpp::NodeOptions{};
  options.parameter_overrides(
      std::vector<rclcpp::Parameter>{{"robot_size", 5}});
  auto const pg = std::make_shared<PathGenerator>(options);

  executor->add_node(pg);
  auto executor_thread = std::thread([&executor]() { executor->spin(); });

  // WHEN a path is requested between two positions that do have a valid path
  // for a smaller robot footprint
  auto const return_code = populateAndSetMap();
  EXPECT_EQ(return_code, rclcpp::FutureReturnCode::SUCCESS)
      << "Setting the map failed";

  auto const request = std::make_shared<example_srvs::srv::GetPath::Request>();
  request->start.data = {0, 0};
  request->goal.data = {7, 7};

  auto const result = sendPathRequest(request);

  EXPECT_EQ(result.second, rclcpp::FutureReturnCode::SUCCESS)
      << "Generating path failed";

  // THEN path planning should fail
  EXPECT_EQ(result.first->success.data, false) << result.first->success.data;

  executor->cancel();
  executor_thread.join();
}

TEST_F(TaskPlanningFixture, medium_footprint_with_path) {
  // Creare executor to spin the node in a separate thread
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // GIVEN a PathGenerator node with a large robot footprint
  auto options = rclcpp::NodeOptions{};
  options.parameter_overrides(
      std::vector<rclcpp::Parameter>{{"robot_size", 2}});
  auto const pg = std::make_shared<PathGenerator>(options);

  executor->add_node(pg);
  auto executor_thread = std::thread([&executor]() { executor->spin(); });

  // WHEN a path is requested between two positions that do have a valid path
  // for a smaller robot footprint
  auto const return_code = populateAndSetMap();
  EXPECT_EQ(return_code, rclcpp::FutureReturnCode::SUCCESS)
      << "Setting the map failed";

  auto const request = std::make_shared<example_srvs::srv::GetPath::Request>();
  request->start.data = {0, 0};
  request->goal.data = {7, 7};

  auto const result = sendPathRequest(request);

  EXPECT_EQ(result.second, rclcpp::FutureReturnCode::SUCCESS)
      << "Generating path failed";

  // THEN path planning should fail
  EXPECT_EQ(result.first->success.data, true) << result.first->success.data;

  executor->cancel();
  executor_thread.join();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
