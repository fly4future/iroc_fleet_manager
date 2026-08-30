#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <mrs_msgs/srv/string.hpp>

#include "iroc_fleet_manager/utils/ros_helpers.h"

using namespace std::chrono_literals;

namespace iroc_fleet_manager
{
namespace utils
{

// Call the UAV's constraint manager to switch constraint/profile.
// This returns true on success; it waits briefly for the service reply.
bool switchProfile(const std::shared_ptr<rclcpp::Node>& node,
                   const std::string& uav_name,
                   const std::string& profile_name)
{
    std::string srv_name = "/" + uav_name + "/constraint_manager/set_constraints";
    auto client = node->create_client<mrs_msgs::srv::String>(srv_name);

    if (!client->wait_for_service(2s)) {
        RCLCPP_ERROR(node->get_logger(), "Service %s is unavailable!", srv_name.c_str());
        return false;
    }

    auto request = std::make_shared<mrs_msgs::srv::String::Request>();
    request->value = profile_name;

    auto response_future = client->async_send_request(request);

    if (response_future.wait_for(2s) == std::future_status::ready) {
        return response_future.get()->success;
    } else {
        RCLCPP_ERROR(node->get_logger(), "Calling service %s failed", srv_name.c_str());
        return false;
    }
}

}  // namespace utils
}  // namespace iroc_fleet_manager


