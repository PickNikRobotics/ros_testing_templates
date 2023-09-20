#pragma once
#include "pathing/pathing.hpp"

#include <memory>
#include <optional>

#include <tl_expected/expected.hpp>

#include <example_srvs/srv/set_map.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace pathing::utilities {
/**
 * @brief      Types of errors expected in parsing the set map request
 */
enum class parsing_set_map_error {
  EMPTY_REQUEST = 1,
  DIMENSION_AND_STRIDE_MISMATCH = 2,
  LENGTH_AND_STRIDE_MISMATCH = 3,
};
/**
 * @brief      Descriptions of the errors
 */
std::map<parsing_set_map_error, std::string> const parsing_set_map_error_description = {
    {parsing_set_map_error::EMPTY_REQUEST,
     "REQUEST DATA FIELD IS EMPTY!!"},
    {parsing_set_map_error::DIMENSION_AND_STRIDE_MISMATCH,
     "OCCUPANCY MAP DIMENSIONS AND STRIDE INCONSISTENT!!"},
    {parsing_set_map_error::LENGTH_AND_STRIDE_MISMATCH,
     "OCCUPANCY MAP LENGTH AND STRIDE INCONSISTENT!!"}};
/**
 * @brief      Converts map from message to occupancy map if possible
 *
 * @param[in]  request containing the map
 *
 * @return     std::optional containing the occupancy map
 */
tl::expected<Map<unsigned char>,parsing_set_map_error> parseSetMapRequest(
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
