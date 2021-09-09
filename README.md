# dependency_injection_ros
Basic ROS1 example on doing dependency injection

# Things that need to be organized into something coherent
- Pure virtual handle interfaces
  - Stylistic choice, makes interfaces portable
  - Removes dependence (specifically when writing mocks) on constructor details for the ROS handle

# ROS2 Notes
- rclcpp::node_interfaces are not a viable tool for dependency injection
  - They return objects requiring complicated construction
  - While in the public namespace, they seem to be meant for internal OSRF testing https://answers.ros.org/question/343948/what-is-the-rclcppnode_interfacesnodebaseinterface-for/
