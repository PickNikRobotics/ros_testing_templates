cmake_minimum_required(VERSION 3.16)
project(naive CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(naive src/node.cpp src/incrementer.cpp src/ros_middleware.cpp)
target_include_directories(naive PRIVATE include)
ament_target_dependencies(naive rclcpp std_msgs)

install(TARGETS
  naive
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(incrementer_test test/incrementer_test.cpp src/incrementer.cpp src/ros_middleware.cpp TIMEOUT 5)
  target_include_directories(incrementer_test PRIVATE include)
  ament_target_dependencies(incrementer_test rclcpp std_msgs)
endif()

ament_package()
