cmake_minimum_required(VERSION 3.16.3)
project(rcl_handle)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    rclcpp
    rclcpp_action
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} INTERFACE)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
