#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

using namespace std::chrono_literals;

// production_code.h/cc
template<typename T>
using Map = std::vector<std::vector<T>>;

// X, Y position
struct Position {
	size_t x;
	size_t y;
};

using Path = std::vector<Position>;

// Operator overload to print path
std::ostream& operator<<(std::ostream& os, std::vector<Position> const& path) {
	for (auto const& pose:path) {
		os << "(" << pose.x << ", " << pose.y << ")\n";
	}
	return os;
}

// Operator overload for position comparison
bool operator==(Position const& lhs, Position const& rhs) {
	return lhs.x == rhs.x && lhs.y == rhs.y;
}

class PathGenerator {
public:
	PathGenerator() : node_(std::make_shared<rclcpp::Node>("bad_test_subscriber")), executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
        // Create ROS2 subscriber for the costmap
		costmap_subscriber_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>("/test/costmap", rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::UInt8MultiArray::SharedPtr msg){costmap_callback(msg);});

		// Let the node spin to execute get_costmap callbacks
		executor_->add_node(node_);
		executor_thread_ = std::thread([this]() { executor_->spin(); });    		
	}
	~PathGenerator() {
		executor_->cancel();
		executor_thread_.join();		
	}
    std::optional<Path> generate_global_path(Position const& start, Position const& goal);
    void costmap_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

    Map<unsigned char> get_costmap() const { return map_;}
private:
	rclcpp::Node::SharedPtr node_;
	rclcpp::Executor::SharedPtr executor_;
	std::thread executor_thread_;

	Map<unsigned char> map_;

	rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr costmap_subscriber_;
};

// From the Robot, goal and costmap, generate a trajectory (Deterministic calculation)
std::optional<Path> PathGenerator::generate_global_path(Position const& start, Position const& goal) { // Calculation
    // Some cool and nifty algorithm
    // What is the delta in position
	int const del_x = goal.x - start.x;
	int const del_y = goal.y - start.y;

    // What direction to move in for each dimension
	int const del_x_sign = std::copysign(1.0,del_x);
	int const del_y_sign = std::copysign(1.0,del_y);

    // Push start onto the path
	Path path;
	path.push_back(start);

    // Fails if there is any obstacle in the way
    // Move horizontally
	for (size_t i = 0; i < (std::abs(del_x)); ++i) {
		if (map_.at(path.back().y).at(path.back().x + del_x_sign) == 1) {
			return std::nullopt;
		}        
		path.push_back({path.back().x + del_x_sign, path.back().y});
	}
    // Move vertically
	for (size_t i = 0; i < (std::abs(del_y)); i++) {
		if (map_.at(path.back().y + del_y_sign).at(path.back().x) == 1) {
			return std::nullopt;
		}            
		path.push_back({path.back().x, path.back().y + del_y_sign});
	}

	return path;
}

// Function that gets costmap
void PathGenerator::costmap_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) { // Action
    // Get the costmap from a ros topic
	// Check that map layout makes sense
	if ((msg->layout.dim[0].size * msg->layout.dim[1].size) != msg->layout.dim[0].stride) {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "COSTMAP DIMENSIONS AND STRIDE INCONSISTENT!!");
		return;
	}
	if (msg->layout.dim[0].stride != msg->data.size()) {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "COSTMAP LENGTH AND STRIDE INCONSISTENT!!");
		return;
	}
	auto const begin = std::begin(msg->data) ;
	// Populate the map
	for (auto row = 0; row < msg->layout.dim[0].size; row++) {
		map_.push_back({begin+row*msg->layout.dim[1].size, begin+(row+1)*msg->layout.dim[1].size});
	}
}

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

	void publish_costmap() { // Action
		auto msg = std_msgs::msg::UInt8MultiArray();

		msg.layout.dim.resize(3, std_msgs::msg::MultiArrayDimension());

	    msg.layout.dim[0].label = "rows";
	    msg.layout.dim[0].size = 8;
	    msg.layout.dim[0].stride = 64;

	    msg.layout.dim[1].label = "columns";
	    msg.layout.dim[1].size = 8;
	    msg.layout.dim[1].stride = 1;

	    msg.layout.dim[2].label = "channel";
	    msg.layout.dim[2].size = 1;
	    msg.layout.dim[2].stride = 1;    

		msg.data = {0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 1, 0, 0, 0, 0, 
		0, 0, 0, 1, 0, 0, 0, 0, 
		0, 0, 1, 1, 1, 0, 0, 0, 
		0, 0, 1, 0, 1, 1, 0, 0, 
		0, 0, 1, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0}; 

		costmap_publisher_->publish(msg);
	};
	rclcpp::Node::SharedPtr node_;
	rclcpp::Executor::SharedPtr executor_;
	std::thread executor_thread_;
	rclcpp::TimerBase::SharedPtr timer_;

	PathGenerator pg_;

	rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr costmap_publisher_;
};

TEST_F(TaskPlanningFixture, same_start_and_goal) {
	// Wait some time to make sure a valid costmap has been received
	// Note all the extra boilerplate code that had to be written to
	// test the generate_global_path function. There could have been
	// bugs introduced in the code. At the minimum, the test has been
	// slowed down, with the presence of std::this_thread::sleep_for()
	std::this_thread::sleep_for(std::chrono::seconds(1));

	// GIVEN a populated costmap AND the same start and goal position
	EXPECT_FALSE(pg_.get_costmap().empty());

    Position const start {0, 0};
    Position const goal {0, 0};

    // WHEN the global path is produced
    auto const& path = pg_.generate_global_path(start, goal);

    // THEN the global path produced should have one element, which is the start/goal position
    std::vector<Position> expected {{0, 0}};
    EXPECT_EQ(path.value(), expected) << path.value();
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	::testing::InitGoogleTest(&argc, argv);
	int result = RUN_ALL_TESTS();
	rclcpp::shutdown();
	return result;
}
