# functional_programming_tests

This directory contains examples of tests that do and do not follow functional programming principles. When working with ROS, it is tempting to follow the established paradigm of encapsulating code and associated data in classes that inherit from or are composed of `rclcpp::Node`.

```
class MinimalPublisher : public rclcpp::Node
// ... class definition
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

This usually leads to the code being dependent on the ROS ecosystem to function, even when it is not needed. In `without_functional_programming.cpp`, there is class method that requires testing, `generate_global_path`.

```
class PathGenerator {
public:
// ... some code
    std::optional<Path> generate_global_path(Pos const& start, Pos const& goal);
// ... more code
};
```

To test the method properly, an entire ROS2 pipeline must be spun up.

```
class TaskPlanningFixture : public testing::Test {
public:
	// Adapted from minimal_integration_test
	TaskPlanningFixture() : node_(std::make_shared<rclcpp::Node>("bad_test_publisher")), executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
        // Create ROS2 publisher for the costmap
		costmap_publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>("test/costmap",1);

        // Publish the costmap every 100 ms
		timer_ = node_->create_wall_timer(100ms, std::bind(&TaskPlanningFixture::publish_costmap, this));
	}

	void SetUp() override {
		executor_->add_node(node_);
		executor_thread_ = std::thread([this]() { executor_->spin(); });
	}

    // Cleanup actions that could throw an exception
	void TearDown() override {
		executor_->cancel();
		executor_thread_.join();
	}
// ... more code
```

Writing all this extra code can be time consuming and lead to bugs in testing. In `with_functional_programming.cpp`, the `generate_global_path` method is now a free function, and the `costmap_callback` and `get_costmap` functions aren't needed anymore. This leads to less boilerplate code required to test `generate_global_path`.
