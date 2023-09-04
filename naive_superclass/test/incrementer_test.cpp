#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <naive_superclass/incrementer.hpp>

#include <std_msgs/msg/int64.hpp>

#include <iostream>
#include <atomic>

using namespace std::chrono_literals;

class TaskPlanningFixture : public testing::Test {
public:
	// Adapted from minimal_integration_test
	TaskPlanningFixture() : node_(std::make_shared<rclcpp::Node>("test_service")), executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()), incrementer_(std::make_shared<Incrementer>("incrementer", INCREMENTER_TOPIC_IN, INCREMENTER_TOPIC_OUT)) {
		test_publisher_ = node_->create_publisher<std_msgs::msg::Int64>(INCREMENTER_TOPIC_IN, incrementer_->getQueueSize());
		test_subscriber_ = node_->create_subscription<std_msgs::msg::Int64>(INCREMENTER_TOPIC_OUT, incrementer_->getQueueSize(), std::bind(&TaskPlanningFixture::callback, this, std::placeholders::_1));
	}

	// TODO: Remove this stuff also, the executor stuff isn't explicitly needed 
	void SetUp() override {
		executor_->add_node(incrementer_);
		executor_->add_node(node_);
		executor_thread_ = std::thread([this]() { executor_->spin(); });
	}

    // Cleanup actions that could throw an exception
	void TearDown() override {
		executor_->cancel();
		executor_thread_.join();
	}

	void callback(const std_msgs::msg::Int64::SharedPtr msg) {
		if (msg->data == EXPECTED_INT) {
			pass_flag_.store(true);
		}
	}

protected:

	std::string INCREMENTER_TOPIC_IN = "in";
	std::string INCREMENTER_TOPIC_OUT = "out";

	// Test variables
	static constexpr int PUBLISHED_INT = 7;
	static constexpr int EXPECTED_INT = 8;

	// Member variables 
	rclcpp::Node::SharedPtr node_;
	rclcpp::Executor::SharedPtr executor_;
	std::thread executor_thread_;

	rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr test_subscriber_; 
	rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr test_publisher_;

	std::shared_ptr<Incrementer> incrementer_;

	std::atomic_bool pass_flag_ {false};

};

TEST_F(TaskPlanningFixture, no_path) {
	// Make sure that the Incrementer class is spinning
	// Publish a message to "in"
	// Receive the message from "out"
	// Make sure that the message has been incremented
	 
	std_msgs::msg::Int64 input;
	input.data = PUBLISHED_INT;

	test_publisher_->publish(input);
	
	std::this_thread::sleep_for(1s);

	EXPECT_TRUE(pass_flag_.load());
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	::testing::InitGoogleTest(&argc, argv);
	int result = RUN_ALL_TESTS();
	rclcpp::shutdown();
	return result;
}
