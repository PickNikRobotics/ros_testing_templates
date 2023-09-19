#include "pathing/pathing.hpp"
#include "pathing/pathing_manager.hpp"
#include "pathing/utilities.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <std_msgs/msg/u_int8_multi_array.hpp>

using SetMap = example_srvs::srv::SetMap;
using GetPath = example_srvs::srv::GetPath;

/**
 * @brief      Gets the test costmap.
 *
 * @return     The test costmap.
 */
pathing::Map<unsigned char> get_test_occupancy_map() {
  return {{{0, 0, 0, 0, 0, 0, 0, 0},        //
           {0, 0, 0, 255, 0, 0, 0, 0},      //
           {0, 0, 0, 255, 0, 0, 0, 0},      //
           {0, 0, 255, 255, 255, 0, 0, 0},  //
           {0, 0, 255, 0, 255, 255, 0, 0},  //
           {0, 0, 255, 0, 0, 0, 0, 0},      //
           {0, 0, 0, 0, 0, 0, 0, 0},        //
           {0, 0, 0, 0, 0, 0, 0, 0}}};
}

TEST(PathingGenerateGlobalPath, EmptyOccupancyMap) {
  // GIVEN an empty costmap and some start and goal
  pathing::Map<unsigned char> sample_occupancy_map;

  pathing::Position const start{0, 0};
  pathing::Position const goal{1, 1};

  // WHEN the global path is produced
  auto const path =
      pathing::generate_global_path(start, goal, sample_occupancy_map);

  // THEN the path should be empty
  EXPECT_FALSE(path.has_value()) << path.value();
}

TEST(PathingGenerateGlobalPath, StartXOutOfBound) {
  // GIVEN an some costmap and a start position with the x position out of bound
  auto const sample_occupancy_map = get_test_occupancy_map();

  pathing::Position const start{10, 0};
  pathing::Position const goal{1, 1};

  // WHEN the global path is produced
  auto const path =
      pathing::generate_global_path(start, goal, sample_occupancy_map);

  // THEN the path should be empty
  EXPECT_FALSE(path.has_value()) << path.value();
}

TEST(PathingGenerateGlobalPath, StartYOutOfBound) {
  // GIVEN an some costmap and a start position with the y position out of bound
  auto const sample_occupancy_map = get_test_occupancy_map();

  pathing::Position const start{0, 10};
  pathing::Position const goal{1, 1};

  // WHEN the global path is produced
  auto const path =
      pathing::generate_global_path(start, goal, sample_occupancy_map);

  // THEN the path should be empty
  EXPECT_FALSE(path.has_value()) << path.value();
}

TEST(PathingGenerateGlobalPath, GoalXOutOfBound) {
  // GIVEN an some costmap and a goal position with the x position out of bound
  auto const sample_occupancy_map = get_test_occupancy_map();

  pathing::Position const start{0, 0};
  pathing::Position const goal{10, 1};

  // WHEN the global path is produced
  auto const path =
      pathing::generate_global_path(start, goal, sample_occupancy_map);

  // THEN the path should be empty
  EXPECT_FALSE(path.has_value()) << path.value();
}

TEST(PathingGenerateGlobalPath, GoalYOutOfBound) {
  // GIVEN an some costmap and a goal position with the y position out of bound
  auto const sample_occupancy_map = get_test_occupancy_map();

  pathing::Position const start{0, 0};
  pathing::Position const goal{1, 10};

  // WHEN the global path is produced
  auto const path =
      pathing::generate_global_path(start, goal, sample_occupancy_map);

  // THEN the path should be empty
  EXPECT_FALSE(path.has_value()) << path.value();
}

TEST(GenerateGlobalPath, SameStartAndGoal) {
  // GIVEN a costmap and the same start and goal
  auto const sample_occupancy_map = get_test_occupancy_map();

  pathing::Position const start{0, 0};
  pathing::Position const goal{0, 0};

  // WHEN the global path is produced
  auto const path =
      pathing::generate_global_path(start, goal, sample_occupancy_map);

  // THEN the path should have one element, which is the start/goal position
  pathing::Path expected{{0, 0}};
  EXPECT_EQ(path.value(), expected) << path.value();
}

TEST(GenerateGlobalPath, NoPath) {
  // GIVEN a costmap AND a start and goal position
  auto const sample_occupancy_map = get_test_occupancy_map();

  pathing::Position const start{2, 2};
  pathing::Position const goal{5, 5};

  // WHEN the global path is produced
  auto const path =
      pathing::generate_global_path(start, goal, sample_occupancy_map);

  // THEN the path should not have been generated
  EXPECT_FALSE(path.has_value()) << path.value();
}

TEST(GenerateGlobalPath, PathGenerated) {
  // GIVEN a costmap AND a start and goal position
  auto const sample_occupancy_map = get_test_occupancy_map();

  pathing::Position const start{0, 0};
  pathing::Position const goal{7, 7};

  // WHEN the global path is produced
  auto const path =
      pathing::generate_global_path(start, goal, sample_occupancy_map);

  // THEN the path should have a valid path from start to the goal
  pathing::Path expected{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0},
                         {5, 0}, {6, 0}, {7, 0}, {7, 1}, {7, 2},
                         {7, 3}, {7, 4}, {7, 5}, {7, 6}, {7, 7}};
  EXPECT_EQ(path.value(), expected) << path.value();
}

TEST(PathingUtilitiesParseSetMap, EmptyRequest) {
  // GIVEN a set map request
  auto const request = std::make_shared<SetMap::Request>();

  // WHEN a completely empty request is parsed
  auto const map = pathing::utilities::parseSetMapRequest(request);

  // THEN there should be no map generated
  EXPECT_FALSE(map.has_value());
}

std::shared_ptr<SetMap::Request> createSetMapRequest() {
  auto const request = std::make_shared<SetMap::Request>();

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

  request->map.data = {0,   0,   0,   0,   0, 0, 0,   0, 0,   0, 0,   255, 0,
                       0,   0,   0,   0,   0, 0, 255, 0, 0,   0, 0,   0,   0,
                       255, 255, 255, 0,   0, 0, 0,   0, 255, 0, 255, 255, 0,
                       0,   0,   0,   255, 0, 0, 0,   0, 0,   0, 0,   0,   0,
                       0,   0,   0,   0,   0, 0, 0,   0, 0,   0, 0,   0};
  return request;
}

TEST(PathingUtilitiesParseSetMap, MapRequestWrongStride) {
  // GIVEN a set map request with an incorrect stride
  auto const request = createSetMapRequest();

  request->map.layout.dim[0].stride = request->map.layout.dim[0].stride - 1;

  // WHEN the incorrect request is parsed
  auto const map = pathing::utilities::parseSetMapRequest(request);

  // THEN there should be no map generated
  EXPECT_FALSE(map.has_value());
}

TEST(PathingUtilitiesParseSetMap, MapRequestWrongDimSize) {
  // GIVEN a set map request with an incorrect dimension size
  auto const request = createSetMapRequest();

  request->map.layout.dim[0].size = request->map.layout.dim[0].size - 1;

  // WHEN the incorrect request is parsed
  auto const map = pathing::utilities::parseSetMapRequest(request);

  // THEN there should be no map generated
  EXPECT_FALSE(map.has_value());
}

TEST(PathingUtilitiesParseSetMap, ValidMapRequest) {
  // GIVEN a set map request
  auto const request = createSetMapRequest();

  // WHEN the request is parsed
  auto const map = pathing::utilities::parseSetMapRequest(request);

  // THEN there should be a map generated
  EXPECT_TRUE(map.has_value());
}

TEST(PathingUtilitiesCreateMessageFromPath, EmptyPath) {
  // GIVEN an empty path
  pathing::Path const path{};

  // WHEN a pathing::utilities::createUInt8MultiArrayMessage is created from the
  // path
  auto const msg =
      pathing::utilities::createUInt8MultiArrayMessageFromPath(path);

  // THEN there should be no data in the data field
  EXPECT_TRUE(msg.data.empty());
}

TEST(PathingUtilitiesCreateMessageFromPath, ValidPath) {
  // GIVEN a valid path
  pathing::Path const path{{0, 0}, {0, 1}, {0, 2}, {0, 3}};

  // WHEN a pathing::utilities::createUInt8MultiArrayMessage is created from the
  // path
  auto const msg =
      pathing::utilities::createUInt8MultiArrayMessageFromPath(path);

  // THEN the fields of the message are expected
  EXPECT_EQ(msg.layout.dim[0].size, 4u);
  EXPECT_EQ(msg.layout.dim[0].stride, 8u);
  std::vector<unsigned char> const expected{0, 0, 0, 1, 0, 2, 0, 3};
  EXPECT_EQ(msg.data, expected);
}

TEST(GeneratePath, EmptyOccupancyMap) {
  // GIVEN a GetPath request and an empty costmap
  pathing::Map<unsigned char> const sample_occupancy_map;

  auto const request = std::make_shared<GetPath::Request>();

  request->start.data = {0, 0};
  request->goal.data = {1, 1};

  // WHEN the path is requested
  auto const response = pathing::generate_path::generate_path(
      request, sample_occupancy_map, pathing::generate_global_path);

  // THEN there should be an error with the error::EMPTY_OCCUPANCY_MAP type
  EXPECT_EQ(response.error(),
            pathing::generate_path::error::EMPTY_OCCUPANCY_MAP);
}

TEST(GeneratePath, InvalidStartSize) {
  // GIVEN a GetPath request with an incorrect start field and an occupancy map
  auto const sample_occupancy_map = get_test_occupancy_map();

  auto const request = std::make_shared<GetPath::Request>();

  request->start.data = {0, 0, 0};
  request->goal.data = {1, 1};

  // WHEN the path is requested
  auto const response = pathing::generate_path::generate_path(
      request, sample_occupancy_map, pathing::generate_global_path);

  // THEN there should be an error with the error::INVALID_START_SIZE type
  EXPECT_EQ(response.error(),
            pathing::generate_path::error::INVALID_START_SIZE);
}

TEST(GeneratePath, InvalidGoalSize) {
  // GIVEN a GetPath request with an incorrect goal field and an occupancy map
  auto const sample_occupancy_map = get_test_occupancy_map();

  auto const request = std::make_shared<example_srvs::srv::GetPath::Request>();

  request->start.data = {0, 0};
  request->goal.data = {1, 1, 1};

  // WHEN the path is requested
  auto const response = pathing::generate_path::generate_path(
      request, sample_occupancy_map, pathing::generate_global_path);

  // THEN there should be an error with the error::INVALID_GOAL_SIZE type
  EXPECT_EQ(response.error(), pathing::generate_path::error::INVALID_GOAL_SIZE);
}

TEST(GeneratePath, NoValidPath) {
  // GIVEN a GetPath request and an occupancy map
  auto const sample_occupancy_map = get_test_occupancy_map();

  auto const request = std::make_shared<example_srvs::srv::GetPath::Request>();

  request->start.data = {2, 2};
  request->goal.data = {5, 5};

  // WHEN the path is requested
  auto const response = pathing::generate_path::generate_path(
      request, sample_occupancy_map, pathing::generate_global_path);

  // THEN there should be an error with the error::NO_VALID_PATH type
  EXPECT_EQ(response.error(), pathing::generate_path::error::NO_VALID_PATH);
}

TEST(GeneratePath, PathGenerated) {
  // GIVEN a GetPath request and an occupancy map
  auto const sample_occupancy_map = get_test_occupancy_map();

  auto const request = std::make_shared<example_srvs::srv::GetPath::Request>();

  request->start.data = {0, 0};
  request->goal.data = {7, 7};

  // WHEN the path is requested
  auto const response = pathing::generate_path::generate_path(
      request, sample_occupancy_map, pathing::generate_global_path);

  // THEN there should no errors
  EXPECT_TRUE(response.has_value());
}

/**
 * @brief Create a sample occupancy map in a request message
 * @return A shared pointer to the request message with the occupancy map
 */
std::shared_ptr<SetMap::Request> make_occupancy_map() {
  auto const request = std::make_shared<SetMap::Request>();

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
struct MockMiddlewareHandle : public pathing::Manager::MiddlewareHandle {
  MOCK_METHOD(void, register_set_map_service, (SetMapCallback), (override));
  MOCK_METHOD(void, register_generate_path_service, (GeneratePathCallback),
              (override));
  MOCK_METHOD(void, log_info, (std::string const&), (override));
  MOCK_METHOD(void, log_error, (std::string const&), (override));
};

TEST(PathManager, Ctor) {
  // GIVEN a mock middleware handle
  auto mw = std::make_unique<MockMiddlewareHandle>();

  // THEN it should register the services
  EXPECT_CALL(*mw, register_set_map_service(testing::_)).Times(1);
  EXPECT_CALL(*mw, register_generate_path_service(testing::_)).Times(1);

  // WHEN the path generator is constructed
  auto const path_generator = pathing::Manager{std::move(mw)};
}

TEST(PathManager, SetMap) {
  // GIVEN a path generator
  auto mw = std::make_unique<MockMiddlewareHandle>();
  // Capture the callback so it can be called later
  pathing::Manager::MiddlewareHandle::SetMapCallback callback;
  ON_CALL(*mw, register_set_map_service(testing::_))
      .WillByDefault(testing::SaveArg<0>(&callback));

  auto const path_generator = pathing::Manager{std::move(mw)};

  // WHEN the set map service is called
  auto const request = make_occupancy_map();
  auto response = std::make_shared<SetMap::Response>();
  callback(request, response);

  // THEN the path generator should successfully set the map
  EXPECT_TRUE(response->success.data);
}

TEST(PathManager, NoCostmap) {
  // GIVEN a path generator with a costmap
  auto mw = std::make_unique<MockMiddlewareHandle>();
  // Capture the path callback so it can be called later
  pathing::Manager::MiddlewareHandle::GeneratePathCallback path_callback;
  ON_CALL(*mw, register_generate_path_service(testing::_))
      .WillByDefault(testing::SaveArg<0>(&path_callback));

  auto const path_generator = pathing::Manager{std::move(mw)};

  // WHEN the generate path service is called without a costmap
  auto path_request = std::make_shared<example_srvs::srv::GetPath::Request>();
  path_request->start.data = {0, 0};
  path_request->goal.data = {0, 0};

  auto path_response = std::make_shared<example_srvs::srv::GetPath::Response>();
  path_callback(path_request, path_response);

  // THEN the path generator should fail
  EXPECT_FALSE(path_response->success.data);
}

struct PathManagerFixture : public testing::Test {
  /**
   * @brief Construct a fixture which will set the map and capture the path
   * callback
   */
  PathManagerFixture() : mw_{std::make_unique<MockMiddlewareHandle>()} {
    // When the map callback is called, set the costmap
    ON_CALL(*mw_, register_set_map_service(testing::_))
        .WillByDefault([&](auto const& map_callback) {
          auto const map_request = make_occupancy_map();
          auto map_response = std::make_shared<SetMap::Response>();
          map_callback(map_request, map_response);
        });

    // Capture the path callback so it can be called later
    ON_CALL(*mw_, register_generate_path_service(testing::_))
        .WillByDefault(testing::SaveArg<0>(&path_callback_));
  }

  std::unique_ptr<MockMiddlewareHandle> mw_;
  pathing::Manager::MiddlewareHandle::GeneratePathCallback path_callback_;
};

TEST_F(PathManagerFixture, NoStartNoGoal) {
  // GIVEN a path generator with a costmap
  auto const path_generator = pathing::Manager{std::move(mw_)};

  // WHEN the generate path service is called,
  auto const path_request = std::make_shared<GetPath::Request>();
  auto path_response = std::make_shared<GetPath::Response>();
  path_callback_(path_request, path_response);

  // THEN the path generator should fail
  EXPECT_FALSE(path_response->success.data);
}

TEST_F(PathManagerFixture, SameStartGoal) {
  // GIVEN a path generator with a costmap
  auto const path_generator = pathing::Manager{std::move(mw_)};

  // WHEN the generate path service is called with the same start and goal
  auto path_request = std::make_shared<GetPath::Request>();
  path_request->start.data = {0, 0};
  path_request->goal.data = {0, 0};
  auto path_response = std::make_shared<GetPath::Response>();
  path_callback_(path_request, path_response);

  // THEN the path generator should succeed
  EXPECT_TRUE(path_response->success.data);
  auto const expected = pathing::Path{{0, 0}};
  // AND the path should be the same as the start
  EXPECT_EQ(pathing::utilities::parseGeneratedPath(path_response->path),
            expected);
}

TEST_F(PathManagerFixture, NoPath) {
  // GIVEN a path generator with a costmap
  auto const path_generator = pathing::Manager{std::move(mw_)};

  // WHEN the generate path service is called with an unreachable goal
  auto path_request = std::make_shared<GetPath::Request>();
  path_request->start.data = {2, 2};
  path_request->goal.data = {5, 5};

  auto path_response = std::make_shared<GetPath::Response>();
  path_callback_(path_request, path_response);

  // THEN the path generator should succeed
  EXPECT_FALSE(path_response->success.data);
  auto const expected = pathing::Path{};
  // AND the path should be empty
  EXPECT_EQ(pathing::utilities::parseGeneratedPath(path_response->path),
            expected);
}

TEST_F(PathManagerFixture, PathGenerated) {
  // GIVEN a path generator with a costmap
  auto const path_generator = pathing::Manager{std::move(mw_)};

  // WHEN the generate path service is called with a reachable goal
  auto path_request = std::make_shared<GetPath::Request>();
  path_request->start.data = {0, 0};
  path_request->goal.data = {7, 7};
  auto path_response = std::make_shared<GetPath::Response>();
  path_callback_(path_request, path_response);

  // THEN the path generator should succeed
  EXPECT_TRUE(path_response->success.data);
  auto const expected = pathing::Path{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0},
                                      {5, 0}, {6, 0}, {7, 0}, {7, 1}, {7, 2},
                                      {7, 3}, {7, 4}, {7, 5}, {7, 6}, {7, 7}};
  // AND the path should be the same as the start
  EXPECT_EQ(pathing::utilities::parseGeneratedPath(path_response->path),
            expected);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
