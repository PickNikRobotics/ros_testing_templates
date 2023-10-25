#include "pathing/pathing.hpp"

#include "pathing/utilities.hpp"

#include <cmath>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include <example_srvs/msg/get_path_codes.hpp>
#include <example_srvs/srv/get_path.hpp>
#include <tl_expected/expected.hpp>

namespace pathing {

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

std::optional<Path> generate_global_path(
    Position const& start, Position const& goal,
    Map<unsigned char> const& occupancy_map, int robot_size) {
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
  // x limit is reduced by half of the robot size to prevent the
  // algorithm from checking collision outside of the map
  auto const x_limit = std::abs(del_x) - std::floor(robot_size / 2);
  repeat(x_limit, [&move, &del_x_sign]() { move(del_x_sign, 0); });
  // Move vertically
  auto const y_limit = std::abs(del_y) - std::floor(robot_size / 2);
  repeat(y_limit, [&move, &del_y_sign]() { move(0, del_y_sign); });

  return path;
}

namespace generate_path {

tl::expected<example_srvs::srv::GetPath::Response, error> generate_path(
    std::shared_ptr<example_srvs::srv::GetPath::Request> const request,
    Map<unsigned char> const& occupancy_map, int robot_size,
    PathingGenerator path_generator) {
  if (occupancy_map.get_data().size() == 0) {
    return tl::unexpected(error::EMPTY_OCCUPANCY_MAP);
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
  auto const path = path_generator(start, goal, occupancy_map, robot_size);
  if (!path.has_value()) {
    return tl::unexpected(error::NO_VALID_PATH);
  }

  auto response = example_srvs::srv::GetPath::Response{};
  response.result.code = example_srvs::msg::GetPathCodes::SUCCESS;
  response.path = utilities::createUInt8MultiArrayMessageFromPath(path.value());

  return response;
}

}  // namespace generate_path

}  // namespace pathing
