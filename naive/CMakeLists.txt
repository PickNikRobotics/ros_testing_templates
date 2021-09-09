cmake_minimum_required(VERSION 2.8.3)
project(naive)

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

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(lib${PROJECT_NAME}
  src/incrementer.cpp
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
  find_package(rostest REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}_test
	  test/incrementer_test.test
    test/incrementer_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test
    lib${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
endif()

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
