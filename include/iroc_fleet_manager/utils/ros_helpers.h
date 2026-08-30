// Lightweight ROS helper declarations
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

}  // namespace utils
}  // namespace iroc_fleet_manager
