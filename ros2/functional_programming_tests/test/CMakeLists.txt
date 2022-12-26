find_package(ament_cmake_gmock REQUIRED)
find_package(ament_lint_auto REQUIRED)

set(ament_cmake_cpplint_FOUND TRUE) # Conflicts with clang-format
set(ament_cmake_flake8_FOUND TRUE) # Conflicts with black
set(ament_cmake_uncrustify_FOUND TRUE) # Conflicts with clang-format
ament_lint_auto_find_test_dependencies()

file(GLOB_RECURSE test_srcs "*.cpp")

ament_add_gmock(test_${PROJECT_NAME}
    ${test_srcs}
  TIMEOUT 300
)

ament_target_dependencies(test_${PROJECT_NAME}
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

include_directories(
  SYSTEM # suppress warnings from 3rd party libs that we can't control
    ${GMOCK_INCLUDE_DIRS}
)

target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
)
