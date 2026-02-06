#pragma once

#include <memory.h>
#include <mrs_lib/mutex.h>
#include <mrs_msgs/msg/collision_avoidance_info.hpp>
#include <mrs_msgs/msg/control_info.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/state_estimation_info.hpp>
#include <mrs_msgs/msg/system_health_info.hpp>
#include <mrs_msgs/msg/uav_info.hpp>
#include <mrs_msgs/msg/safety_area_manager_diagnostics.hpp>
// #include <mrs_robot_diagnostics/enums/robot_type.h>
#include <string>
#include <unordered_map>

namespace iroc_fleet_manager {

struct CommonRobotHandler_t {
  mrs_msgs::msg::GeneralRobotInfo::ConstSharedPtr general_robot_info;
  mrs_msgs::msg::StateEstimationInfo::ConstSharedPtr state_estimation_info;
  mrs_msgs::msg::ControlInfo::ConstSharedPtr control_info;
  mrs_msgs::msg::CollisionAvoidanceInfo::ConstSharedPtr collision_avoidance_info;
  mrs_msgs::msg::UavInfo::ConstSharedPtr uav_info;
  mrs_msgs::msg::SystemHealthInfo::ConstSharedPtr system_health_info;
  mrs_msgs::msg::SafetyAreaManagerDiagnostics::ConstSharedPtr safety_area_info;
};

struct CommonRobotHandlers_t {
  std::unordered_map<std::string, CommonRobotHandler_t> robots_map;
};

struct CommonHandlers_t {
  std::shared_ptr<CommonRobotHandlers_t> handlers;
};

} // namespace iroc_fleet_manager

