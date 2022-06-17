cmake_minimum_required(VERSION 3.10)
project(publisher_subscriber)

# Link this 'library' to set the c++ standard / compile-time options requested
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_17)

# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
add_library(project_warnings INTERFACE)

# standard compiler warnings
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

# enable doxygen
# include(cmake/Doxygen.cmake)
# enable_doxygen()

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS
    include
	LIBRARIES
	  ${PROJECT_NAME}
  CATKIN_DEPENDS
    std_msgs
)

# Marking catkin includes as SYSTEM prevents compiler warnings on them
include_directories(include SYSTEM ${catkin_INCLUDE_DIRS})

add_library(lib${PROJECT_NAME}
  src/incrementer.cpp
  src/ros_middleware.cpp
)

target_link_libraries(lib${PROJECT_NAME}
  PUBLIC ${catkin_LIBRARIES}
  PRIVATE project_options project_warnings
)

add_executable(${PROJECT_NAME}
  src/node.cpp
)

target_link_libraries(${PROJECT_NAME}
  lib${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_gmock(${PROJECT_NAME}_test test/incrementer_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test
    lib${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
endif()

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
