#include "pathing/pathing.hpp"

#include <memory>
#include <optional>

#include <example_srvs/srv/set_map.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace pathing::utilities {
std::optional<Map<unsigned char>> parseSetMapRequest(
    std::shared_ptr<example_srvs::srv::SetMap::Request> const request) {
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
};  // namespace pathing::utilities
