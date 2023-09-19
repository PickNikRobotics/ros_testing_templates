#pragma once
#include "pathing/pathing.hpp"

#include <memory>
#include <optional>

#include <example_srvs/srv/set_map.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace pathing::utilities {
/**
 * @brief      Converts map from message to occupancy map if possible
 *
 * @param[in]  request containing the map
 *
 * @return     std::optional containing the occupancy map
 */
std::optional<Map<unsigned char>> parseSetMapRequest(
    std::shared_ptr<example_srvs::srv::SetMap::Request> const request);

/**
 * @brief      Creates a UInt8MultiArray message from a path
 *
 * @param[in]  path to convert
 *
 * @return     The UInt8MultiArray message
 */
std_msgs::msg::UInt8MultiArray createUInt8MultiArrayMessageFromPath(
    Path const& path);

/**
 * @brief Convert a path from ROS message to a vector of positions
 * @param msg The message to convert
 * @return The path as a vector of positions
 */
std::vector<Position> parseGeneratedPath(
    const std_msgs::msg::UInt8MultiArray& msg);
}  // namespace pathing::utilities
