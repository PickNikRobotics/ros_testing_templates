#include "pathing/pathing_manager.hpp"

#include "pathing/pathing.hpp"
#include "pathing/utilities.hpp"

#include <memory>
#include <string>
#include <iostream>

#include <example_srvs/srv/get_path.hpp>
#include <example_srvs/srv/set_map.hpp>
#include <tl_expected/expected.hpp>

using SetMap = example_srvs::srv::SetMap;
using GetPath = example_srvs::srv::GetPath;

namespace pathing {
Manager::Manager(std::unique_ptr<Manager::MiddlewareHandle> mw,
                 Parameters params)
    : mw_{std::move(mw)}, params_{std::move(params)} {
  mw_->register_set_map_service(
      [this](auto const request, auto response) -> void {

        auto const set_map = [this](Map<unsigned char> const map)
        -> tl::expected<Map<unsigned char>, utilities::parsing_set_map_error> {
          this->map_ = map;
          return map;
        };

        auto const return_successful_response = []([[maybe_unused]] auto const success) 
        -> tl::expected<SetMap::Response, utilities::parsing_set_map_error> {
          auto response = SetMap::Response{};
          response.code.code = 0;
          return response;
        };

        auto const print_error = [this](utilities::parsing_set_map_error const error)
            -> tl::expected<SetMap::Response, utilities::parsing_set_map_error> {
          mw_->log_error(std::string{utilities::parsing_set_map_error_description.at(error)});
          return tl::make_unexpected(error);
        };

        auto const return_error_response = [](utilities::parsing_set_map_error const error)
            -> tl::expected<SetMap::Response, utilities::parsing_set_map_error> {
          auto response = SetMap::Response{};
          response.code.code = static_cast<int>(error);
          return response;
        };

        *response = utilities::parseSetMapRequest(request)
                    .and_then(set_map)
                    .and_then(return_successful_response)
                    .or_else(print_error)
                    .or_else(return_error_response)
                    .value();
      });
  mw_->register_generate_path_service([this](auto const request,
                                             auto response) {
    auto const print_error = [this](auto const error)
        -> tl::expected<GetPath::Response, pathing::generate_path::error> {
      mw_->log_error(std::string{pathing::generate_path::error_description.at(error)});
      return tl::make_unexpected(error);
    };

    auto const return_error_response = [](auto const error)
        -> tl::expected<GetPath::Response, pathing::generate_path::error> {
      auto response = GetPath::Response{};
      response.code.code = static_cast<int>(error);
      response.path = std_msgs::msg::UInt8MultiArray();
      return response;
    };

    std::cerr << "In path callback, map size is : " << this->map_.get_data().size() << "\n";

    *response =
        generate_path::generate_path(
            request, this->map_, this->params_.robot_size, generate_global_path)
            .or_else(print_error)
            .or_else(return_error_response)
            .value();
  });
}
}  // namespace pathing
