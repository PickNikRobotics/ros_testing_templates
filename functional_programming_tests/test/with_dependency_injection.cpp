#include <gmock/gmock.h>
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

// Forward Declrataion
class PathGenerator;

template<typename ServiceType>
struct PathGeneratorServiceMiddlewareHandle {
public:

	PathGeneratorServiceMiddlewareHandle(const rclcpp::Node::SharedPtr node_handle, std::string service_name) : node_handle_{node_handle}, service_name_{std::move(service_name)} {};

	// Assumption is that handles derived from this type will be injected into the Path Generator class, and thus need the pointer?
	// I think there is lambda magic available such that this isn't needed?
	void acquirePathGeneratorPtr(PathGenerator* path_generator_ptr) {path_generator_ = path_generator_ptr;};

	virtual ~PathGeneratorServiceMiddlewareHandle() = default;

	virtual void createService() = 0;	

protected:

	virtual void callback(const std::shared_ptr<typename ServiceType::Request>, std::shared_ptr<typename ServiceType::Response>) = 0;	

	rclcpp::Node::SharedPtr node_handle_;

	std::string service_name_;

	rclcpp::Service<ServiceType>::SharedPtr service_;

	PathGenerator* path_generator_;

};

// Service for setting the map
class SetMapService : public PathGeneratorServiceMiddlewareHandle<example_srvs::srv::SetMap> {
public:

	SetMapService(const rclcpp::Node::SharedPtr node_handle, std::string service_name) 
	: PathGeneratorServiceMiddlewareHandle<example_srvs::srv::SetMap>(node_handle, service_name) {};

	void createService() override {
		service_ = node_handle_->create_service<example_srvs::srv::SetMap>(service_name_, 
			[this](const std::shared_ptr<example_srvs::srv::SetMap::Request> request, std::shared_ptr<example_srvs::srv::SetMap::Response> response){this->callback(request, response);});
	};

protected:

	void callback(const std::shared_ptr<example_srvs::srv::SetMap::Request> request, std::shared_ptr<example_srvs::srv::SetMap::Response> response) override {
		std::cout << "In SetMapService::callback\n";
	};
};

// Service for getting the path
class GetPathService : public PathGeneratorServiceMiddlewareHandle<example_srvs::srv::GetPath> {
public:

	GetPathService(const rclcpp::Node::SharedPtr node_handle, std::string service_name) 
	: PathGeneratorServiceMiddlewareHandle<example_srvs::srv::GetPath>(node_handle, service_name) {};

	void createService() override {
		service_ = node_handle_->create_service<example_srvs::srv::GetPath>(service_name_, 
			[this](const std::shared_ptr<example_srvs::srv::GetPath::Request> request, std::shared_ptr<example_srvs::srv::GetPath::Response> response){this->callback(request, response);});	
	};

protected:

	void callback(const std::shared_ptr<example_srvs::srv::GetPath::Request> request, std::shared_ptr<example_srvs::srv::GetPath::Response> response) override {
		std::cout << "In GetPathService::callback\n";
	};
};

class PathGeneratorServices {
public:
	PathGeneratorServices(SetMapService& set_map_service, GetPathService& get_path_service) : set_map_service_{set_map_service}, get_path_service_{get_path_service}{};

	void acquirePathGeneratorPtrForServices(PathGenerator* path_generator_ptr) {
		set_map_service_.acquirePathGeneratorPtr(path_generator_ptr);
		get_path_service_.acquirePathGeneratorPtr(path_generator_ptr);
	}

	virtual void createServices() {
		set_map_service_.createService();
		get_path_service_.createService();		
	}

private:
	SetMapService& set_map_service_;
	GetPathService& get_path_service_;
};

class PathGenerator {
public:

	PathGenerator(PathGeneratorServices& pg_services) : pg_services_{pg_services} {
		pg_services_.acquirePathGeneratorPtrForServices(this);
		pg_services_.createServices();
	}

private:
	// TODO: Fill this in again when the structure of all of this has been figured out

	PathGeneratorServices& pg_services_;
	
	Map<unsigned char> map_;

};

// test.cpp

using ::testing::_;
using ::testing::Invoke;
using ::testing::Mock;

struct MockSetMapService : public SetMapService {
	MockSetMapService(const rclcpp::Node::SharedPtr node_handle, std::string service_name) :  SetMapService(node_handle, service_name){};
};

struct MockGetPathService : public GetPathService {
	MockGetPathService(const rclcpp::Node::SharedPtr node_handle, std::string service_name) :  GetPathService(node_handle, service_name){};
};

struct MockPathGeneratorServices : public PathGeneratorServices {
	MockPathGeneratorServices(SetMapService& set_map_service, GetPathService& get_path_service) : PathGeneratorServices(set_map_service,get_path_service){};
	MOCK_METHOD0(createServices, void());
};

TEST(ServiceTests, CreateServices) {

	// TODO: No idea how to mock a node, so I'm starting a node that will be used by the services? Griz will say something lol
	auto node = std::make_shared<rclcpp::Node>("path_generator_services");

	MockSetMapService sm_serv(node, "MockSetMapService");
	MockGetPathService gp_serv(node, "MockGetPathService");

	MockPathGeneratorServices all_servs(sm_serv, gp_serv);

	EXPECT_CALL(all_servs, createServices).Times(1);

	PathGenerator pg{all_servs};
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	::testing::InitGoogleTest(&argc, argv);
	int result = RUN_ALL_TESTS();
	rclcpp::shutdown();
	return result;
}
