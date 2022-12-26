cmake_minimum_required(VERSION 3.16.3)
project(functional_programming_tests LANGUAGES CXX)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} INTERFACE)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} 
)

ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
