cmake_minimum_required(VERSION 2.8.3)
project(service_call)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp injection_messages)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS
    include
	LIBRARIES
	  ${PROJECT_NAME}
  CATKIN_DEPENDS
    injection_messages
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(lib${PROJECT_NAME}
  src/incrementer.cpp
  src/ros_middleware.cpp
)

target_link_libraries(lib${PROJECT_NAME}
  ${catkin_LIBRARIES}
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
