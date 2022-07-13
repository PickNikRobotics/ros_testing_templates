cmake_minimum_required(VERSION 3.16.3)
project(action)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    repo_cmake
    example_msgs
    rclcpp
    rclcpp_action
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

########################################################################
# Task execution library                                               #
########################################################################

add_library(${PROJECT_NAME}
  SHARED
    src/executor.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    lab0_cmake::project_warnings
    lab0_cmake::project_options
)

######################################################
# Task execution node                                #
######################################################

add_executable(${PROJECT_NAME}_node
    src/node.cpp
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_20)
ament_target_dependencies(${PROJECT_NAME}_node
  PUBLIC
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
    ${PROJECT_NAME}
    lab0_cmake::project_warnings
    lab0_cmake::project_options
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    # TODO
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
