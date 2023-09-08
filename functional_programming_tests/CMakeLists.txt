cmake_minimum_required(VERSION 3.16.3)
project(functional_programming_tests)
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gtest(with_functional_programming test/with_functional_programming.cpp TIMEOUT 5)
  ament_target_dependencies(with_functional_programming rclcpp std_msgs)

  ament_add_gtest(without_functional_programming test/without_functional_programming.cpp TIMEOUT 5)
  ament_target_dependencies(without_functional_programming rclcpp std_msgs example_srvs)

  #ament_add_gtest(with_dependency_injection test/with_dependency_injection.cpp TIMEOUT 5)
  #ament_target_dependencies(with_dependency_injection rclcpp std_msgs example_srvs)  

  ament_add_gmock(with_dependency_injection test/with_dependency_injection.cpp TIMEOUT 5)
  ament_target_dependencies(with_dependency_injection rclcpp std_msgs example_srvs)    
endif()

ament_package()
