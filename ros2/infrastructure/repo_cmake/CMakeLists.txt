cmake_minimum_required(VERSION 3.16.3)
project(repo_cmake)

include(cmake/StandardProjectSettings.cmake)

# Link this 'library' to set the c++ standard / compile-time options requested
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_20)

# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
add_library(project_warnings INTERFACE)

# standard compiler warnings
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

# sanitizer options if supported by compiler
include(cmake/Sanitizers.cmake)
enable_sanitizers(project_options)

# allow for static analysis options
include(cmake/StaticAnalyzers.cmake)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  TARGETS project_warnings project_options
  EXPORT ${PROJECT_NAME}_Targets
)
ament_export_targets(${PROJECT_NAME}_Targets)

ament_package()
