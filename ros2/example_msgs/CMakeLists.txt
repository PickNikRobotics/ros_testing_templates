cmake_minimum_required(VERSION 3.16.3)
project(example_msgs)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(action_files
  "action/Simple.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES
    builtin_interfaces
    action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
