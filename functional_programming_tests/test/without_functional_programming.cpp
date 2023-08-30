#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <example_srvs/srv/set_map.hpp>
#include <example_srvs/srv/get_path.hpp>

using namespace std::chrono_literals;

// TODO: Add more comments

// production_code.h/cc
// X, Y position
struct Position {
	size_t x;
	size_t y;
};

template<typename T>
class Map
{
public:
	Map(){};
    Map(std::vector<std::vector<T>> data): data_{data} {};
    T at(Position const& pos) const {
        return data_.at(pos.y).at(pos.x);
    }
    std::vector<std::vector<T>>& get_data() {return data_;}
private:
    std::vector<std::vector<T>> data_;
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

class PathGenerator : public rclcpp::Node {
public:
	PathGenerator() : Node("wo_fp_server") {
		// Services for setting the map and generating the path       
		map_setter_service_ = this->create_service<example_srvs::srv::SetMap>("set_costmap", std::bind(&PathGenerator::set_map_service, this, std::placeholders::_1, std::placeholders::_2));
		path_generator_service_ = this->create_service<example_srvs::srv::GetPath>("generate_global_path", std::bind(&PathGenerator::generate_path_service, this, std::placeholders::_1, std::placeholders::_2));		
	}

	void set_map_service(const std::shared_ptr<example_srvs::srv::SetMap::Request> request,
							   std::shared_ptr<example_srvs::srv::SetMap::Response> response) {
		// Set the map to generate the path from
		response->success.data = costmap_setter(request->map);
	}

	void generate_path_service(const std::shared_ptr<example_srvs::srv::GetPath::Request> request,
							   std::shared_ptr<example_srvs::srv::GetPath::Response> response) {
		if (map_.get_data().size() == 0) {
			RCLCPP_ERROR_STREAM(this->get_logger(), "MAP IS EMPTY!!");
			response->success.data = false;
			response->path = std_msgs::msg::UInt8MultiArray();			
		}
		// Check to make sure start and goal fields of the request are of size 2
		if (request->start.data.size() != 2) {
			RCLCPP_ERROR_STREAM(this->get_logger(), "START POSITION MUST CONTAIN TWO ELEMENTS!!");
			response->success.data = false;
			response->path = std_msgs::msg::UInt8MultiArray();
			return;
		}
		if (request->goal.data.size() != 2) {
			RCLCPP_ERROR_STREAM(this->get_logger(), "GOAL POSITION MUST CONTAIN TWO ELEMENTS!!");
			response->success.data = false;
			response->path = std_msgs::msg::UInt8MultiArray();
			return;
		}
		auto const start = Position{request->start.data[0], request->start.data[1]};
		auto const goal = Position{request->goal.data[0], request->goal.data[1]};

		// Generate the path
		auto const path = generate_global_path(start, goal);

		// Start populating the response message
		auto response_path = std_msgs::msg::UInt8MultiArray();

		if (path.has_value()){

			response_path.layout.dim.resize(3, std_msgs::msg::MultiArrayDimension());

		    response_path.layout.dim[0].label = "rows";
		    response_path.layout.dim[0].size = path.value().size();
		    response_path.layout.dim[0].stride = path.value().size()*2;

		    response_path.layout.dim[1].label = "columns";
		    response_path.layout.dim[1].size = 2;
		    response_path.layout.dim[1].stride = 1;

		    response_path.layout.dim[2].label = "channel";
		    response_path.layout.dim[2].size = 1;
		    response_path.layout.dim[2].stride = 1;		

		    // Start pushing back the path only if there is one
		    if (path.value().size() > 0) {
			    for (auto const& position:path.value()) {
			    	response_path.data.push_back(position.x);
			    	response_path.data.push_back(position.y);
			    }
			}
		}

		response->success.data = path.has_value();	
		response->path = response_path;

	}

private:
	// Function that sets costmap
    bool costmap_setter(const std_msgs::msg::UInt8MultiArray& costmap)  { // Action
	    // Get the costmap from a ros topic
		// Check that map layout makes sense
		if ((costmap.layout.dim[0].size * costmap.layout.dim[1].size) != costmap.layout.dim[0].stride) {
			RCLCPP_ERROR_STREAM(this->get_logger(), "COSTMAP DIMENSIONS AND STRIDE INCONSISTENT!!");
			return false;
		}
		if (costmap.layout.dim[0].stride != costmap.data.size()) {
			RCLCPP_ERROR_STREAM(this->get_logger(), "COSTMAP LENGTH AND STRIDE INCONSISTENT!!");
			return false;
		}
		auto const begin = std::begin(costmap.data) ;
		// Populate the map
		for (size_t row = 0; row < costmap.layout.dim[0].size; row++) {
			map_.get_data().push_back({begin+row*costmap.layout.dim[1].size, begin+(row+1)*costmap.layout.dim[1].size});
		}
		return true;
	}

	// From the start and goal, generate a trajectory (Deterministic calculation)	
    std::optional<Path> generate_global_path(Position const& start, Position const& goal) { // Calculation
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

	    auto is_occupied = [this](auto const x, auto const y) -> bool {
	        return this->map_.at(Position{x, y}) == 1;
	    };

	    // Fails if there is any obstacle in the way
	    // Move horizontally
	    for (size_t i = 0; i < (std::abs(del_x)); ++i) {
	        if (is_occupied(path.back().x + del_x_sign, path.back().y)) {        
	            return std::nullopt;
	        }        
	        path.push_back({path.back().x + del_x_sign, path.back().y});
	    }
	    // Move vertically
	    for (size_t i = 0; i < (std::abs(del_y)); i++) {
	        // if (costmap.at(Position{path.back().x, path.back().y + del_y_sign}) == 1) {
	        if (is_occupied(path.back().x, path.back().y + del_y_sign)) {            
	            return std::nullopt;
	        }            
	        path.push_back({path.back().x, path.back().y + del_y_sign});
	    }

		return path;
	}	

	// rclcpp::Executor::SharedPtr executor_;
	// std::thread executor_thread_;

	Map<unsigned char> map_;

	rclcpp::Service<example_srvs::srv::SetMap>::SharedPtr map_setter_service_;
	rclcpp::Service<example_srvs::srv::GetPath>::SharedPtr path_generator_service_;
};

std::vector<Position> parseGeneratedPath(const std_msgs::msg::UInt8MultiArray& msg) {
	// Check the path has an even number of elements
	if ((msg.data.size() % 2) != 0) {
		return {};
	}
	if (msg.data.size() ==  0) {
		return {};
	}	
	std::vector<Position> path;

	for (size_t idx = 0; idx < (msg.data.size() - 1); idx+=2) {
		path.push_back({msg.data[idx], msg.data[idx + 1]});
	}

	return path;
}

// TODO: clean up/fix up the Test Fixture class
class TaskPlanningFixture : public testing::Test {
public:
	// Adapted from minimal_integration_test
	TaskPlanningFixture() : node_(std::make_shared<rclcpp::Node>("wo_fp_client")), executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()), pg_(std::make_shared<PathGenerator>()) {
        // Create ROS2 clients to set the map and calculate the path
		map_setter_client_ = node_->create_client<example_srvs::srv::SetMap>("set_costmap");
		path_generator_client_ = node_->create_client<example_srvs::srv::GetPath>("generate_global_path");
	}

	// TODO: Remove this stuff also, the executor stuff isn't explicitly needed 
	void SetUp() override {
		// Need the server to take commands for testing
		executor_->add_node(pg_);
		executor_thread_ = std::thread([this]() { executor_->spin(); });

		// pg_->add_and_spin_thread();
	}

    // Cleanup actions that could throw an exception
	void TearDown() override {
		executor_->cancel();
		executor_thread_.join();
	}

	void populateAndSetMap() {

		auto const request = std::make_shared<example_srvs::srv::SetMap::Request>();

		request->map = std_msgs::msg::UInt8MultiArray();

		request->map.layout.dim.resize(3, std_msgs::msg::MultiArrayDimension());

	    request->map.layout.dim[0].label = "rows";
	    request->map.layout.dim[0].size = 8;
	    request->map.layout.dim[0].stride = 64;

	    request->map.layout.dim[1].label = "columns";
	    request->map.layout.dim[1].size = 8;
	    request->map.layout.dim[1].stride = 1;

	    request->map.layout.dim[2].label = "channel";
	    request->map.layout.dim[2].size = 1;
	    request->map.layout.dim[2].stride = 1;    

		request->map.data = {0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 1, 0, 0, 0, 0, 
		0, 0, 0, 1, 0, 0, 0, 0, 
		0, 0, 1, 1, 1, 0, 0, 0, 
		0, 0, 1, 0, 1, 1, 0, 0, 
		0, 0, 1, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0};

		while (!map_setter_client_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR_STREAM(node_->get_logger(), "Interrupted while waiting for map setter service. Exiting.");
				return;
			}
			RCLCPP_INFO_STREAM(node_->get_logger(), "Map setter service not available, waiting again...");
		}
		
		auto set_map_result = map_setter_client_->async_send_request(request);

		EXPECT_EQ(rclcpp::spin_until_future_complete(node_, set_map_result), rclcpp::FutureReturnCode::SUCCESS) << "Setting the map failed";

	}

	example_srvs::srv::GetPath::Response::SharedPtr sendPathRequest(const example_srvs::srv::GetPath::Request::SharedPtr request) {

		while (!path_generator_client_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR_STREAM(node_->get_logger(), "Interrupted while waiting for path generator service. Exiting.");
				return std::make_shared<example_srvs::srv::GetPath::Response>();
			}
			RCLCPP_INFO_STREAM(node_->get_logger(), "Path generator service not available, waiting again...");
		}

		auto generate_path_result = path_generator_client_->async_send_request(request);

		EXPECT_EQ(rclcpp::spin_until_future_complete(node_, generate_path_result), rclcpp::FutureReturnCode::SUCCESS) << "Generating path failed";

		return generate_path_result.get();
	}

private:

	// Member variables 
	rclcpp::Node::SharedPtr node_;
	rclcpp::Executor::SharedPtr executor_;
	std::thread executor_thread_;

	rclcpp::Client<example_srvs::srv::SetMap>::SharedPtr map_setter_client_;
	rclcpp::Client<example_srvs::srv::GetPath>::SharedPtr path_generator_client_;

	std::shared_ptr<PathGenerator> pg_;
};

TEST_F(TaskPlanningFixture, same_start_and_goal) {
	// Note all the extra boilerplate code that had to be written to
	// test the generate_global_path function. There could have been
	// bugs introduced in the code.

	// GIVEN a populated costmap that is set without error
	populateAndSetMap();

	// WHEN a path with the same start and goal is requested

	auto const request = std::make_shared<example_srvs::srv::GetPath::Request>();

	request->start.data = {0, 0};
	request->goal.data = {0, 0};

	auto const result = sendPathRequest(request);

    // THEN the global path produced should have one element, which is the start/goal position
    std::vector<Position> expected {{0, 0}};
    EXPECT_EQ(result->success.data, true) << result->success.data;
    EXPECT_EQ(parseGeneratedPath(result->path), expected) << parseGeneratedPath(result->path);
}

TEST_F(TaskPlanningFixture, no_path) {

	// GIVEN a populated costmap that is set without error
	populateAndSetMap();

	// WHEN a path with the same start and goal is requested

	auto const request = std::make_shared<example_srvs::srv::GetPath::Request>();

	request->start.data = {2, 2};
	request->goal.data = {5, 5};

	auto const result = sendPathRequest(request);

    // THEN the global path produced should have one element, which is the start/goal position
    std::vector<Position> expected {};
    EXPECT_EQ(result->success.data, false) << result->success.data;
    EXPECT_EQ(parseGeneratedPath(result->path), expected) << parseGeneratedPath(result->path);
}

TEST_F(TaskPlanningFixture, path_generated) {
	
	// GIVEN a populated costmap that is set without error
	populateAndSetMap();

	// WHEN a path with the same start and goal is requested

	auto const request = std::make_shared<example_srvs::srv::GetPath::Request>();

	request->start.data = {0, 0};
	request->goal.data = {7, 7};

	auto const result = sendPathRequest(request);

    // THEN the global path produced should have one element, which is the start/goal position
    std::vector<Position> expected {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {7, 1}, {7, 2}, {7, 3}, {7, 4}, {7, 5}, {7, 6}, {7, 7}};
    EXPECT_EQ(result->success.data, true) << result->success.data;
    EXPECT_EQ(parseGeneratedPath(result->path), expected) << parseGeneratedPath(result->path);
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	::testing::InitGoogleTest(&argc, argv);
	int result = RUN_ALL_TESTS();
	rclcpp::shutdown();
	return result;
}
