cmake_minimum_required(VERSION 3.16.3)
project(functional_programming_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tl_expected REQUIRED)

add_library(pathing SHARED
  src/pathing.cpp
  src/pathing_manager.cpp
  src/utilities.cpp)
target_compile_features(pathing PUBLIC cxx_std_20)
target_include_directories(pathing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pathing rclcpp std_msgs example_srvs tl_expected)

add_executable(pathing_generator src/pathing_node.cpp)
target_compile_features(pathing_generator PUBLIC cxx_std_20)
ament_target_dependencies(pathing_generator PUBLIC rclcpp example_srvs)
target_link_libraries(pathing_generator
	PRIVATE
	pathing)

install(DIRECTORY include DESTINATION share/pathing)

install(
  TARGETS pathing
  EXPORT pathing_Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/pathing
	)

install(
  TARGETS pathing_generator
  RUNTIME DESTINATION lib/functional_programming_tests
	)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(with_functional_programming test/with_functional_programming.cpp TIMEOUT 5)
  target_compile_features(with_functional_programming PUBLIC cxx_std_20)
  ament_target_dependencies(with_functional_programming rclcpp std_msgs)
  target_link_libraries(with_functional_programming
	  pathing)

  ament_add_gtest(without_functional_programming test/without_functional_programming.cpp TIMEOUT 5)
  target_compile_features(without_functional_programming PUBLIC cxx_std_20)
  ament_target_dependencies(without_functional_programming rclcpp std_msgs example_srvs)

  ament_add_gmock(with_dependency_injection test/with_dependency_injection.cpp TIMEOUT 5)
  target_compile_features(with_dependency_injection PUBLIC cxx_std_20)
  ament_target_dependencies(with_dependency_injection rclcpp std_msgs example_srvs)

  ament_add_gtest(with_parameters test/with_parameters.cpp TIMEOUT 5)
  target_compile_features(with_parameters PUBLIC cxx_std_20)
  ament_target_dependencies(with_parameters rclcpp std_msgs example_srvs)

  ament_add_gtest(with_parameter_factory test/with_parameter_factory.cpp TIMEOUT 5)
  target_compile_features(with_parameter_factory PUBLIC cxx_std_20)
  ament_target_dependencies(with_parameter_factory rclcpp std_msgs example_srvs)
endif()

ament_package()
