cmake_minimum_required(VERSION 3.16.3)
project(minimal_integration_test)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)

  ament_add_gtest_executable(basic_test
    test/basic_test.cpp
  )
  ament_target_dependencies(basic_test ${THIS_PACKAGE_INCLUDE_DEPENDS})
  add_launch_test(test/launch/basic_test.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
