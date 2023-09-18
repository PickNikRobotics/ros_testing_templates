#include "pathing/pathing_manager.hpp"

#include "pathing/pathing.hpp"
#include "pathing/utilities.hpp"

#include <memory>
#include <string>

#include <example_srvs/srv/get_path.hpp>
#include <example_srvs/srv/set_map.hpp>
#include <tl_expected/expected.hpp>

using SetMap = example_srvs::srv::SetMap;
using GetPath = example_srvs::srv::GetPath;

namespace pathing {
Manager::Manager(std::unique_ptr<Manager::MiddlewareHandle> mw)
    : mw_{std::move(mw)} {
  mw_->register_set_map_service(
      [this](auto const request, auto response) -> void {
        auto const path = utilities::parseSetMapRequest(request);
        if (path) this->map_ = path.value();
        response->success.data = path.has_value();
      });
  mw_->register_generate_path_service([this](auto const request,
                                             auto response) {
    auto const print_error = [this](std::string_view error)
        -> tl::expected<GetPath::Response, std::string> {
      mw_->log_error(std::string{error});
      return tl::make_unexpected("");
    };

    auto const return_empty_response = []([[maybe_unused]] auto const)
        -> tl::expected<GetPath::Response, std::string> {
      auto response = GetPath::Response{};
      response.success.data = false;
      response.path = std_msgs::msg::UInt8MultiArray();
      return response;
    };
    auto const stringify_error = [](auto const error) {
      return generate_path::error_description.at(error);
    };
    *response =
        generate_path::generate_path(request, this->map_, generate_global_path)
            .map_error(stringify_error)
            .or_else(print_error)
            .or_else(return_empty_response)
            .value();
  });
}
}  // namespace pathing
