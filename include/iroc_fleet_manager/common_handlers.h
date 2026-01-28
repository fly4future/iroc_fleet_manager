#pragma once

#include <memory.h>
#include <mrs_lib/mutex.h>
#include <mrs_msgs/msg/collision_avoidance_info.hpp>
#include <mrs_msgs/msg/control_info.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/state_estimation_info.hpp>
#include <mrs_msgs/msg/system_health_info.hpp>
#include <mrs_msgs/msg/uav_info.hpp>
#include <mrs_robot_diagnostics/enums/robot_type.h>
#include <mrs_msgs/msg/safety_area_manager_diagnostics.hpp>
#include <string>
#include <unordered_map>

namespace iroc_fleet_manager {

// Note: ConstPtr type is a typedef for a shared pointer,
// specifically boost::shared_ptr in ROS 1

struct CommonRobotHandler_t {
  mrs_robot_diagnostics::GeneralRobotInfo::ConstPtr general_robot_info;
  mrs_robot_diagnostics::StateEstimationInfo::ConstPtr state_estimation_info;
  mrs_robot_diagnostics::ControlInfo::ConstPtr control_info;
  mrs_robot_diagnostics::CollisionAvoidanceInfo::ConstPtr collision_avoidance_info;
  mrs_robot_diagnostics::UavInfo::ConstPtr uav_info;
  mrs_robot_diagnostics::SystemHealthInfo::ConstPtr system_health_info;
  mrs_msgs::SafetyAreaManagerDiagnostics::ConstPtr safety_area_info;
};

struct CommonRobotHandlers_t {
  std::unordered_map<std::string, CommonRobotHandler_t> robots_map;
};

struct CommonHandlers_t {
  std::shared_ptr<CommonRobotHandlers_t> handlers;
};

} // namespace iroc_fleet_manager

