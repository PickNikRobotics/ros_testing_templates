#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <example_srvs/msg/get_path_codes.hpp>
#include <example_srvs/msg/set_map_codes.hpp>
#include <example_srvs/srv/get_path.hpp>
#include <example_srvs/srv/set_map.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <std_msgs/msg/u_int8_multi_array.hpp>

using namespace std::chrono_literals;

/**
 * @brief Represents a position in the map
 */
struct Position {
  size_t x;
  size_t y;
};

// Operator overload for position comparison
bool operator==(Position const& lhs, Position const& rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

using Path = std::vector<Position>;

// Operator overload to print path
std::ostream& operator<<(std::ostream& os, Path const& path) {
  for (auto const& pose : path) {
    os << "(" << pose.x << ", " << pose.y << ")\n";
  }
  return os;
}

template <typename T>
class Map {
 public:
  Map(){};
  /**
   * @brief Construct a map from a 2D vector
   * @param data The 2D vector to construct the map from
   */
  Map(std::vector<std::vector<T>> data) : data_{data} {};
  T at(Position const& pos) const { return data_.at(pos.y).at(pos.x); }
  std::vector<std::vector<T>>& get_data() { return data_; }

 private:
  std::vector<std::vector<T>> data_;
};

struct PathGenerator {
  struct MiddlewareHandle {
    // Define map service callback type
    using SetMapServiceCallback = std::function<void(
        std::shared_ptr<example_srvs::srv::SetMap::Request> const request,
        std::shared_ptr<example_srvs::srv::SetMap::Response> response)>;

    // Define path generation service callback type
    using GeneratePathServiceCallback = std::function<void(
        std::shared_ptr<example_srvs::srv::GetPath::Request> const request,
        std::shared_ptr<example_srvs::srv::GetPath::Response> response)>;
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
    virtual void register_set_map_service(SetMapServiceCallback callback) = 0;

    /**
     * @brief Register a callback for the generate path service
     * @param callback The callback to register
     */
    virtual void register_generate_path_service(
        GeneratePathServiceCallback callback) = 0;

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
   * @brief Builds paths given a map
   * @param mw The middleware handle for interacting with services
   */
  PathGenerator(std::unique_ptr<MiddlewareHandle> mw) : mw_{std::move(mw)} {
    // Register the set map service
    mw_->register_set_map_service(
        [this](
            std::shared_ptr<example_srvs::srv::SetMap::Request> const request,
            std::shared_ptr<example_srvs::srv::SetMap::Response> response) {
          // Set the map to generate the path from
          response->result.code = costmap_setter(request->map);
        });
    // Register the generate path service
    mw_->register_generate_path_service(  // Trivial comment to enfore params on
                                          // new line
        [this](
            std::shared_ptr<example_srvs::srv::GetPath::Request> const request,
            std::shared_ptr<example_srvs::srv::GetPath::Response> response) {
          if (map_.get_data().size() == 0) {
            mw_->log_error("MAP IS EMPTY!!");
            response->result.code =
                example_srvs::msg::GetPathCodes::EMPTY_OCCUPANCY_MAP;
            response->path = std_msgs::msg::UInt8MultiArray();
            return;
          }
          // Check to make sure start and goal fields of the request are of
          // size 2
          if (request->start.data.size() != 2) {
            mw_->log_error("START POSITION MUST CONTAIN TWO ELEMENTS!!");
            response->result.code =
                example_srvs::msg::GetPathCodes::START_POSITION_INVALID_SIZE;
            response->path = std_msgs::msg::UInt8MultiArray();
            return;
          }
          if (request->goal.data.size() != 2) {
            mw_->log_error("GOAL POSITION MUST CONTAIN TWO ELEMENTS!!");
            response->result.code =
                example_srvs::msg::GetPathCodes::GOAL_POSITION_INVALID_SIZE;
            response->path = std_msgs::msg::UInt8MultiArray();
            return;
          }
          auto const start =
              Position{request->start.data[0], request->start.data[1]};
          auto const goal =
              Position{request->goal.data[0], request->goal.data[1]};

          // Generate the path
          auto const path = generate_global_path(start, goal);

          // Start populating the response message
          auto response_path = std_msgs::msg::UInt8MultiArray();

          if (path.has_value()) {
            response_path.layout.dim.resize(
                3, std_msgs::msg::MultiArrayDimension());

            response_path.layout.dim[0].label = "rows";
            response_path.layout.dim[0].size = path.value().size();
            response_path.layout.dim[0].stride = path.value().size() * 2;

            response_path.layout.dim[1].label = "columns";
            response_path.layout.dim[1].size = 2;
            response_path.layout.dim[1].stride = 1;

            response_path.layout.dim[2].label = "channel";
            response_path.layout.dim[2].size = 1;
            response_path.layout.dim[2].stride = 1;

            // Start pushing back the path only if there is one
            if (path.value().size() > 0) {
              for (auto const& position : path.value()) {
                response_path.data.push_back(position.x);
                response_path.data.push_back(position.y);
              }
            }
          }

          response->result.code =
              path.has_value() ? example_srvs::msg::GetPathCodes::SUCCESS
                               : example_srvs::msg::GetPathCodes::NO_VALID_PATH;
          response->path = response_path;
        });
  }

 private:
  /**
   * @brief Extracts costmap from message and sets it
   * @param costmap The costmap message
   * @return True if the costmap was set successfully, false otherwise
   */
  unsigned char costmap_setter(
      std_msgs::msg::UInt8MultiArray const& costmap) {  // Action
    // Get the costmap from a ros topic
    // Check that map layout makes sense
    if ((costmap.layout.dim[0].size * costmap.layout.dim[1].size) !=
        costmap.layout.dim[0].stride) {
      mw_->log_error("COSTMAP DIMENSIONS AND STRIDE INCONSISTENT!!");
      return example_srvs::msg::SetMapCodes::DIMENSION_AND_STRIDE_MISMATCH;
    }
    if (costmap.layout.dim[0].stride != costmap.data.size()) {
      mw_->log_error("COSTMAP LENGTH AND STRIDE INCONSISTENT!!");
      return example_srvs::msg::SetMapCodes::LENGTH_AND_STRIDE_MISMATCH;
    }
    auto const begin = std::begin(costmap.data);
    // Populate the map
    for (size_t row = 0; row < costmap.layout.dim[0].size; row++) {
      map_.get_data().push_back(
          {begin + row * costmap.layout.dim[1].size,
           begin + (row + 1) * costmap.layout.dim[1].size});
    }
    return example_srvs::msg::SetMapCodes::SUCCESS;
  }

  /**
   * @brief Generates a path from start to goal
   * @param start The start position
   * @param goal The goal position
   * @return The path from start to goal if it exists, std::nullopt otherwise
   */
  std::optional<Path> generate_global_path(
      Position const& start, Position const& goal) {  // Calculation
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

    auto is_occupied = [this](auto const x, auto const y) -> bool {
      return this->map_.at(Position{x, y}) == 1;
    };

    // Fails if there is any obstacle in the way
    // Move horizontally
    for (size_t i = 0; i < (std::abs(del_x)); ++i) {
      if (is_occupied(path.back().x + del_x_sign, path.back().y)) {
        return std::nullopt;
      }
      path.push_back({path.back().x + del_x_sign, path.back().y});
    }
    // Move vertically
    for (size_t i = 0; i < (std::abs(del_y)); i++) {
      // if (costmap.at(Position{path.back().x, path.back().y + del_y_sign})
      // == 1) {
      if (is_occupied(path.back().x, path.back().y + del_y_sign)) {
        return std::nullopt;
      }
      path.push_back({path.back().x, path.back().y + del_y_sign});
    }

    return path;
  }

  /**
   * @brief Handle to interact with the middleware services
   */
  std::unique_ptr<MiddlewareHandle> mw_;
  /**
   * @brief Cost map of the environment
   */
  Map<unsigned char> map_;
};

struct RosMiddleware : public PathGenerator::MiddlewareHandle {
  /**
   * @brief Construct a new Rosful Path Generator middleware handle
   * @param node to use for the ROS services
   */
  RosMiddleware(std::shared_ptr<rclcpp::Node> node) : node_{std::move(node)} {}

  void register_set_map_service(SetMapServiceCallback callback) override {
    map_setter_ = node_->create_service<example_srvs::srv::SetMap>(
        "set_costmap",
        [callback](
            std::shared_ptr<example_srvs::srv::SetMap::Request> const request,
            std::shared_ptr<example_srvs::srv::SetMap::Response> response) {
          callback(request, response);
        });
  }
  void register_generate_path_service(
      GeneratePathServiceCallback callback) override {
    path_generator_ = node_->create_service<example_srvs::srv::GetPath>(
        "generate_global_path",
        [callback](
            std::shared_ptr<example_srvs::srv::GetPath::Request> const request,
            std::shared_ptr<example_srvs::srv::GetPath::Response> response) {
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
  std::shared_ptr<rclcpp::Service<example_srvs::srv::SetMap>> map_setter_;
  std::shared_ptr<rclcpp::Service<example_srvs::srv::GetPath>> path_generator_;
};

/**
 * @brief Convert a path from ROS message to a vector of positions
 * @param msg The message to convert
 * @return The path as a vector of positions
 */
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

/**
 * @brief Create a sample cost map in a request message
 * @return A shared pointer to the request message with the cost map
 */
std::shared_ptr<example_srvs::srv::SetMap::Request> make_occupancy_map() {
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

  request->map.data = {0, 0, 0, 0, 0, 0, 0, 0,  //
                       0, 0, 0, 1, 0, 0, 0, 0,  //
                       0, 0, 0, 1, 0, 0, 0, 0,  //
                       0, 0, 1, 1, 1, 0, 0, 0,  //
                       0, 0, 1, 0, 1, 1, 0, 0,  //
                       0, 0, 1, 0, 0, 0, 0, 0,  //
                       0, 0, 0, 0, 0, 0, 0, 0,  //
                       0, 0, 0, 0, 0, 0, 0, 0};
  return request;
}

// Create mock middleware handle
struct MockMiddlewareHandle : public PathGenerator::MiddlewareHandle {
  MOCK_METHOD(void, register_set_map_service, (SetMapServiceCallback),
              (override));
  MOCK_METHOD(void, register_generate_path_service,
              (GeneratePathServiceCallback), (override));
  MOCK_METHOD(void, log_info, (std::string const&), (override));
  MOCK_METHOD(void, log_error, (std::string const&), (override));
};

TEST(PathGenerator, Ctor) {
  // GIVEN a mock middleware handle
  auto mw = std::make_unique<MockMiddlewareHandle>();

  // THEN it should register the services
  EXPECT_CALL(*mw, register_set_map_service(testing::_)).Times(1);
  EXPECT_CALL(*mw, register_generate_path_service(testing::_)).Times(1);

  // WHEN the path generator is constructed
  auto const path_generator = PathGenerator{std::move(mw)};
}

TEST(PathGenerator, SetMap) {
  // GIVEN a path generator
  auto mw = std::make_unique<MockMiddlewareHandle>();
  // Capture the callback so it can be called later
  PathGenerator::MiddlewareHandle::SetMapServiceCallback callback;
  ON_CALL(*mw, register_set_map_service(testing::_))
      .WillByDefault(testing::SaveArg<0>(&callback));

  auto const path_generator = PathGenerator{std::move(mw)};

  // WHEN the set map service is called
  auto const request = make_occupancy_map();
  auto response = std::make_shared<example_srvs::srv::SetMap::Response>();
  callback(request, response);

  // THEN the path generator should successfully set the map
  EXPECT_EQ(response->result.code, example_srvs::msg::SetMapCodes::SUCCESS);
}

TEST(PathGenerator, NoCostmap) {
  // GIVEN a path generator with a costmap
  auto mw = std::make_unique<MockMiddlewareHandle>();
  // Capture the path callback so it can be called later
  PathGenerator::MiddlewareHandle::GeneratePathServiceCallback path_callback;
  ON_CALL(*mw, register_generate_path_service(testing::_))
      .WillByDefault(testing::SaveArg<0>(&path_callback));

  auto const path_generator = PathGenerator{std::move(mw)};

  // WHEN the generate path service is called without a costmap
  auto path_request = std::make_shared<example_srvs::srv::GetPath::Request>();
  path_request->start.data = {0, 0};
  path_request->goal.data = {0, 0};

  auto path_response = std::make_shared<example_srvs::srv::GetPath::Response>();
  path_callback(path_request, path_response);

  // THEN the path generator should fail
  EXPECT_EQ(path_response->result.code,
            example_srvs::msg::GetPathCodes::EMPTY_OCCUPANCY_MAP);
}

struct PathGeneratorFixture : public testing::Test {
  /**
   * @brief Construct a fixture which will set the map and capture the path
   * callback
   */
  PathGeneratorFixture() : mw_{std::make_unique<MockMiddlewareHandle>()} {
    // When the map callback is called, set the costmap
    ON_CALL(*mw_, register_set_map_service(testing::_))
        .WillByDefault([&](auto const& map_callback) {
          auto const map_request = make_occupancy_map();
          auto map_response =
              std::make_shared<example_srvs::srv::SetMap::Response>();
          map_callback(map_request, map_response);
        });

    // Capture the path callback so it can be called later
    ON_CALL(*mw_, register_generate_path_service(testing::_))
        .WillByDefault(testing::SaveArg<0>(&path_callback_));
  }

  std::unique_ptr<MockMiddlewareHandle> mw_;
  PathGenerator::MiddlewareHandle::GeneratePathServiceCallback path_callback_;
};

TEST_F(PathGeneratorFixture, NoStartNoGoal) {
  // GIVEN a path generator with a costmap
  auto const path_generator = PathGenerator{std::move(mw_)};

  // WHEN the generate path service is called,
  auto const path_request =
      std::make_shared<example_srvs::srv::GetPath::Request>();
  auto path_response = std::make_shared<example_srvs::srv::GetPath::Response>();
  path_callback_(path_request, path_response);

  // THEN the path generator should fail
  EXPECT_EQ(path_response->result.code,
            example_srvs::msg::GetPathCodes::START_POSITION_INVALID_SIZE);
}

TEST_F(PathGeneratorFixture, SameStartGoal) {
  // GIVEN a path generator with a costmap
  auto const path_generator = PathGenerator{std::move(mw_)};

  // WHEN the generate path service is called with the same start and goal
  auto path_request = std::make_shared<example_srvs::srv::GetPath::Request>();
  path_request->start.data = {0, 0};
  path_request->goal.data = {0, 0};
  auto path_response = std::make_shared<example_srvs::srv::GetPath::Response>();
  path_callback_(path_request, path_response);

  // THEN the path generator should succeed
  EXPECT_EQ(path_response->result.code,
            example_srvs::msg::GetPathCodes::SUCCESS);
  auto const expected = std::vector<Position>{{0, 0}};
  // AND the path should be the same as the start
  EXPECT_EQ(parseGeneratedPath(path_response->path), expected);
}

TEST_F(PathGeneratorFixture, NoPath) {
  // GIVEN a path generator with a costmap
  auto const path_generator = PathGenerator{std::move(mw_)};

  // WHEN the generate path service is called with an unreachable goal
  auto path_request = std::make_shared<example_srvs::srv::GetPath::Request>();
  path_request->start.data = {2, 2};
  path_request->goal.data = {5, 5};

  auto path_response = std::make_shared<example_srvs::srv::GetPath::Response>();
  path_callback_(path_request, path_response);

  // THEN the path generator should succeed
  EXPECT_EQ(path_response->result.code,
            example_srvs::msg::GetPathCodes::NO_VALID_PATH);
  auto const expected = std::vector<Position>{};
  // AND the path should be empty
  EXPECT_EQ(parseGeneratedPath(path_response->path), expected);
}

TEST_F(PathGeneratorFixture, PathGenerated) {
  // GIVEN a path generator with a costmap
  auto const path_generator = PathGenerator{std::move(mw_)};

  // WHEN the generate path service is called with a reachable goal
  auto path_request = std::make_shared<example_srvs::srv::GetPath::Request>();
  path_request->start.data = {0, 0};
  path_request->goal.data = {7, 7};
  auto path_response = std::make_shared<example_srvs::srv::GetPath::Response>();
  path_callback_(path_request, path_response);

  // THEN the path generator should succeed
  EXPECT_EQ(path_response->result.code,
            example_srvs::msg::GetPathCodes::SUCCESS);
  auto const expected = std::vector<Position>{
      {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0},
      {7, 1}, {7, 2}, {7, 3}, {7, 4}, {7, 5}, {7, 6}, {7, 7}};
  // AND the path should be the same as the start
  EXPECT_EQ(parseGeneratedPath(path_response->path), expected);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
