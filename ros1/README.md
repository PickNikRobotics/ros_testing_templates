# ros testing templates
Examples of ROS testing techniques using native ros test, dependency injection, and others.


# ROS2 Notes
- No ROS2 examples yet but
  - https://github.com/ros-planning/moveit2/pull/569/
  - https://github.com/ros-planning/moveit2/pull/562/
- rclcpp::node_interfaces are not a viable tool for dependency injection
  - They return objects requiring complicated construction
  - While in the public namespace, they seem to be meant for internal OSRF testing https://answers.ros.org/question/343948/what-is-the-rclcppnode_interfacesnodebaseinterface-for/
