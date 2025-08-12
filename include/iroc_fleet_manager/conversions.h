#pragma once

#include "iroc_fleet_manager/types.h"
#include "iroc_mission_handler/Subtask.h"
#include "iroc_mission_handler/Waypoint.h"
#include <mrs_msgs/Reference.h>

#include <vector>

namespace iroc_fleet_manager {

// Helper functions to simplify the ROS API

template <typename Msg_T>
Msg_T toRosMsg(const custom_types::Point2D& point) {
  Msg_T msg_point;
  msg_point.x = point.x;
  msg_point.y = point.y;
  return msg_point;
}

template <typename Msg_T>
Msg_T toRosMsg(const custom_types::Point3D& point) {
  Msg_T msg_point;
  msg_point.x = point.x;
  msg_point.y = point.y;
  msg_point.z = point.z;
  return msg_point;
}

template <typename Msg_T>
Msg_T toRosMsg(const custom_types::Reference& point) {
  Msg_T msg_point;
  msg_point.position.x = point.x;
  msg_point.position.y = point.y;
  msg_point.position.z = point.z;
  msg_point.heading = point.heading;
  return msg_point;
}

template <typename Msg_T>
Msg_T toRosMsg(const custom_types::Subtask& subtask) {
  Msg_T ros_subtask;
  ros_subtask.type = subtask.type;
  ros_subtask.parameters = subtask.parameters;
  return ros_subtask;
}

template <typename Msg_T>
Msg_T toRosMsg(const custom_types::Waypoint& waypoint) {
  Msg_T ros_waypoint;

  ros_waypoint.reference_point = toRosMsg<mrs_msgs::Reference>(waypoint.reference);

  ros_waypoint.subtasks.reserve(waypoint.getSubtasks().size());
  for (const auto& subtask : waypoint.getSubtasks()) {
    ros_waypoint.subtasks.emplace_back(toRosMsg<iroc_mission_handler::Subtask>(subtask));
  }
  // ros_waypoint.parallel_execution = false;
  return ros_waypoint;
}

template <typename Msg_T, typename Point_T>
std::vector<Msg_T> toRosMsg(const std::vector<Point_T>& points) {
  std::vector<Msg_T> result;
  result.reserve(points.size());

  for (const auto& point : points) {
    result.emplace_back(toRosMsg<Msg_T>(point));
  }

  return result;
}
} // namespace iroc_fleet_manager
