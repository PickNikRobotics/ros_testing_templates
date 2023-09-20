#pragma once
#include "pathing/pathing.hpp"

#include <functional>
#include <memory>
#include <string>

#include <example_srvs/srv/get_path.hpp>
#include <example_srvs/srv/set_map.hpp>

using SetMap = example_srvs::srv::SetMap;
using GetPath = example_srvs::srv::GetPath;

namespace pathing {

struct Manager {
  /**
   * @brief Parameters for the path manager
   */
  struct Parameters {
    /**
     * @brief The size of the robot in pixels
     * @note This is a toy description of robot size as a square
     *       centered at the robot's upper left hand corner
     */
    int robot_size;
  };

  /**
   * @brief Interface for interacting with ros services
   */
  struct MiddlewareHandle {
    // Define map service callback type
    using SetMapCallback = std::function<void(
        std::shared_ptr<example_srvs::srv::SetMap::Request> const request,
        std::shared_ptr<example_srvs::srv::SetMap::Response> response)>;

    // Define path generation service callback type
    using GeneratePathCallback = std::function<void(
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
   * @param params for manager configuration
   */
  Manager(std::unique_ptr<MiddlewareHandle> mw, Parameters params);

 private:
  /**
   * @brief Middleware handle for interacting with services
   */
  std::unique_ptr<MiddlewareHandle> mw_;

  /**
   * @brief Parameters for the path manager
   */
  Parameters params_;

  /**
   * @brief Occupancy map to path through
   */
  Map<unsigned char> map_;
};
}  // namespace pathing
