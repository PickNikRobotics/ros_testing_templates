#pragma once
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <example_srvs/srv/get_path.hpp>
#include <tl_expected/expected.hpp>

namespace pathing {

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
std::ostream& operator<<(std::ostream& os, std::vector<Position> const& path);

// Operator overload for position comparison
bool operator==(Position const& lhs, Position const& rhs);

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
    Map<unsigned char> const& occupancy_map);

namespace generate_path {
/**
 * @brief      Types of errors expected in the generate path callback function
 */
enum class error {
  EMPTY_OCCUPANCY_MAP,
  INVALID_START_SIZE,
  INVALID_GOAL_SIZE,
  NO_VALID_PATH
};

/**
 * @brief      Descriptions of the errors
 */
std::map<error, std::string> const error_description = {
    {error::EMPTY_OCCUPANCY_MAP, "The Occupancy Map is empty."},
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
tl::expected<example_srvs::srv::GetPath::Response, error> generate_path(
    std::shared_ptr<example_srvs::srv::GetPath::Request> const request,
    Map<unsigned char> const& occupancy_map, PathingGenerator path_generator);

}  // namespace generate_path

}  // namespace pathing
