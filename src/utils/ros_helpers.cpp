#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
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
        RCLCPP_ERROR(node->get_logger(), "Sluzba %s neni dostupna!", srv_name.c_str());
        return false;
    }

    auto request = std::make_shared<mrs_msgs::srv::String::Request>();
    request->value = profile_name;

    auto response_future = client->async_send_request(request);

    if (response_future.wait_for(2s) == std::future_status::ready) {
        return response_future.get()->success;
    } else {
        RCLCPP_ERROR(node->get_logger(), "Selhalo volani sluzby %s", srv_name.c_str());
        return false;
    }
}

// Set several numeric parameters on the UAV constraint manager for the
// 'medium' profile. This helper constructs parameter messages and calls
// the `set_parameters` service. It returns true if the service call
// completed successfully within the short timeout.
bool setCustomValuesForMedium(const std::shared_ptr<rclcpp::Node>& node,
                             const std::string& uav_name, 
                             double horiz_speed, double horiz_acc,
                             double vert_asc_speed, double vert_asc_acc,
                             double vert_desc_speed, double vert_desc_acc) 
{
    std::string srv_name = "/" + uav_name + "/constraint_manager/set_parameters";
    auto client = node->create_client<rcl_interfaces::srv::SetParameters>(srv_name);

    if (!client->wait_for_service(2s)) {
        RCLCPP_ERROR(node->get_logger(), "Sluzba parametrů %s neni dostupna!", srv_name.c_str());
        return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    auto addDoubleParam = [&](const std::string& name, double val) {
        rcl_interfaces::msg::Parameter p;
        p.name = name;
        p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        p.value.double_value = val;
        request->parameters.push_back(p);
    };

    addDoubleParam("constraints.medium.horizontal.speed", horiz_speed);
    addDoubleParam("constraints.medium.horizontal.acceleration", horiz_acc);
    addDoubleParam("constraints.medium.vertical.ascending.speed", vert_asc_speed);
    addDoubleParam("constraints.medium.vertical.ascending.acceleration", vert_asc_acc);
    addDoubleParam("constraints.medium.vertical.descending.speed", vert_desc_speed);
    addDoubleParam("constraints.medium.vertical.descending.acceleration", vert_desc_acc);

    auto response_future = client->async_send_request(request);

    // Wait briefly for the service response. If the node is running
    // inside a component container, callbacks will be processed by the
    // container executor; waiting on the future avoids interfering with
    // that executor.
    if (response_future.wait_for(2s) == std::future_status::ready) {
        RCLCPP_INFO(node->get_logger(), "[%s] Hodnoty byly uspesne zmeneny v ROS 2!", uav_name.c_str());
        return true;
    } else {
        RCLCPP_ERROR(node->get_logger(), "[%s] Selhalo nastaveni ROS 2 parametru.", uav_name.c_str());
        return false;
    }
}

}  // namespace utils
}  // namespace iroc_fleet_manager


