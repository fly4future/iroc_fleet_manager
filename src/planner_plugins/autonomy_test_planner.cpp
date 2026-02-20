#include <iroc_fleet_manager/iroc_plugins/autonomy_test_planner.h>

namespace iroc_fleet_manager {

namespace planners {

namespace autonomy_test_planner {

bool AutonomyTestPlanner::initialize(const rclcpp::Node::SharedPtr node, const std::string &name, const std::string &name_space,
                                     std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers) {
  node_            = node;
  name_            = name;
  common_handlers_ = common_handlers;

  RCLCPP_INFO(node_->get_logger(), "[%s]: initialized under the name '%s', namespace '%s'", name_.c_str(), name.c_str(), name_space.c_str());

  is_initialized_ = true;
  return true;
}

bool AutonomyTestPlanner::activate(void) {
  RCLCPP_INFO(node_->get_logger(), "[%s]: activated", name_.c_str());

  is_active_ = true;

  return true;
}

void AutonomyTestPlanner::deactivate(void) {
  is_active_ = false;

  RCLCPP_INFO(node_->get_logger(), "[%s]: deactivated", name_.c_str());
}

std::tuple<result_t, std::vector<iroc_mission_handler::msg::MissionGoal>> AutonomyTestPlanner::createGoal(const std::string &goal) const {
  RCLCPP_INFO(node_->get_logger(), "[%s]: creating goal from the received request", name_.c_str());

  std::vector<iroc_mission_handler::msg::MissionGoal> mission_robots;
  result_t result;

  json json_msg, robots;

  // Parsing JSON and creating robots JSON for post processing
  result = parseJson(goal, json_msg);

  if (!result.success) {
    result.success = false;
    result.message = "Failed to parse JSON msg";
    return std::make_tuple(result, mission_robots);
  }

  bool success = utils::parseVars(json_msg, {{"robots", &robots}});

  if (!success) {
    result.success = false;
    result.message = "Failure while parsing robot data, bad JSON request";
    return std::make_tuple(result, mission_robots);
  }

  mission_robots.reserve(robots.size());
  for (auto &robot : robots) {
    std::string name;
    int segment_length;

    const auto succ = utils::parseVars(robot, {{"name", &name}, {"segment_length", &segment_length}});

    if (!succ) {
      result.success = false;
      result.message = "Failed to parse parameters";
      return std::make_tuple(result, mission_robots);
    }

    bool isRobotInFleet = common_handlers_->handlers->robots_map.count(name);

    if (!isRobotInFleet) {
      RCLCPP_WARN(node_->get_logger(), "[AutonomyTestPlanner] Robot %s not within the fleet", name.c_str());
      std::stringstream ss;
      ss << name << " not found in the fleet!";
      result.message = ss.str();
      result.success = false;
      return std::make_tuple(result, mission_robots);
    }

    iroc_mission_handler::msg::MissionGoal robot_goal;
    robot_goal.name            = name;
    robot_goal.frame_id        = iroc_mission_handler::msg::MissionGoal::FRAME_ID_FCU;
    robot_goal.height_id       = iroc_mission_handler::msg::MissionGoal::HEIGHT_ID_FCU;
    robot_goal.terminal_action = 0;
    robot_goal.points          = getAutonomyPoints(segment_length);
    // Save the individual robot goal
    mission_robots.push_back(robot_goal);
  } // robots iteration

  RCLCPP_INFO(node_->get_logger(), "[AutonomyTestPlanner] Goal created successfully!");
  result.success = true;
  result.message = "Goal created successfully";
  return std::make_tuple(result, mission_robots);
}

/*!
 * Helper method to obtain the reference points of the pre-defined autonomy test
 * trajectory, intended to be a simple cross-like movement based on the position
 * of the UAV.
 *
 */
std::vector<iroc_mission_handler::msg::Waypoint> AutonomyTestPlanner::getAutonomyPoints(double segment_length) const {

  std::vector<mrs_msgs::msg::Reference> points;
  mrs_msgs::msg::Reference point;

  // Center point
  point.position.x = 0.0;
  point.position.y = 0.0;
  point.position.z = 0.0;
  point.heading    = 0.0;

  // Right
  points.push_back(point);
  point.position.y = -segment_length;
  points.push_back(point);

  // Back to center
  point.position.y = 0.0;
  points.push_back(point);

  // Front
  point.position.x = segment_length;
  points.push_back(point);

  // Back to center
  point.position.x = 0.0;
  points.push_back(point);

  // Left
  point.position.y = segment_length;
  points.push_back(point);

  // Back to center
  point.position.y = 0.0;
  points.push_back(point);

  // Back
  point.position.x = -segment_length;
  points.push_back(point);

  // Back to center
  point.position.x = 0.0;
  points.push_back(point);

  // 360-degree pirouette (3 points)
  point.heading = (2 * M_PI) / 3.0;
  points.push_back(point);

  point.heading = (4 * M_PI) / 3.0;
  points.push_back(point);

  point.heading = (2 * M_PI);
  points.push_back(point);

  // 360-degree pirouette (3 points)
  point.heading = (2 * M_PI) / 3.0;
  points.push_back(point);

  point.heading = (4 * M_PI) / 3.0;
  points.push_back(point);

  point.heading = (2 * M_PI);
  points.push_back(point);

  // Convert mrs_msgs::msg::Reference to iroc_mission_handler::msg::Waypoint
  std::vector<iroc_mission_handler::msg::Waypoint> autonomy_points;
  autonomy_points.reserve(points.size());
  for (const auto &p : points) {
    iroc_mission_handler::msg::Waypoint wp;
    wp.reference.position.x = p.position.x;
    wp.reference.position.y = p.position.y;
    wp.reference.position.z = p.position.z;
    wp.reference.heading     = p.heading;

    autonomy_points.push_back(wp);
  }

  return autonomy_points;
}

} // namespace autonomy_test_planner
} // namespace planners
} // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::planners::autonomy_test_planner::AutonomyTestPlanner, iroc_fleet_manager::planners::Planner);
