// ROS helper declarations
//
// These functions provide small convenience wrappers to call common
// MRS/ROS services used by the fleet manager. They are intentionally
// synchronous wrappers that wait a short time for the service to
// respond; callers should prefer non-blocking or higher-level
// orchestration when used in performance-sensitive code paths.

#pragma once

#include <memory>
#include <string>

#include <rclcpp/node.hpp>

namespace iroc_fleet_manager
{
namespace utils
{

// Switch the active constraint/profile on a UAV's constraint manager.
// Returns true if the service call succeeded and the profile was applied.
bool switchProfile(const std::shared_ptr<rclcpp::Node> &node,
                   const std::string &uav_name,
                   const std::string &profile_name);

// Set a small set of numeric parameters used by the 'medium' flight
// profile. This wraps the ROS 2 `set_parameters` service and returns
// true on success.
bool setCustomValuesForMedium(const std::shared_ptr<rclcpp::Node> &node,
                              const std::string &uav_name,
                              double horiz_speed,
                              double horiz_acc,
                              double vert_asc_speed,
                              double vert_asc_acc,
                              double vert_desc_speed,
                              double vert_desc_acc);

}  // namespace utils
}  // namespace iroc_fleet_manager
