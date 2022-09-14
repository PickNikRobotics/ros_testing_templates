#include "naive/incrementer.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {

	rclcpp::init(argc,argv);
	auto const node = std::make_shared<rclcpp::Node>("incrementer");
	Incrementer incrementer{node, "/in", "/out"};
	rclcpp::spin(node);
	rclcpp::shutdown();

}
