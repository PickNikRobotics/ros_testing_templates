#include "pathing/pathing.hpp"

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


