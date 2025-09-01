#include <iroc_fleet_manager/iroc_plugins/waypoint_planner.h>

namespace iroc_fleet_manager
{

namespace planners
{

namespace waypoint_planner
{

bool WaypointPlanner::initialize(const ros::NodeHandle &parent_nh, const std::string &name, const std::string &name_space,
                                 std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers) {

  // nh_ will behave just like normal NodeHandle
  ros::NodeHandle nh_(parent_nh, name_space);

  _name_           = name;
  common_handlers_ = common_handlers;

  ros::Time::waitForValid();

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[%s]: initialized under the name '%s', namespace '%s' and action ", _name_.c_str(), name.c_str(), name_space.c_str());

  is_initialized_ = true;
  return true;
}

bool WaypointPlanner::activate(void) {

  int some_number = 0;
  ROS_INFO("[%s]: activated with some_number=%d", _name_.c_str(), some_number);

  is_active_ = true;

  return true;
}

void WaypointPlanner::deactivate(void) {

  is_active_ = false;

  ROS_INFO("[%s]: deactivated", _name_.c_str());
}

std::tuple<result_t, std::vector<iroc_mission_handler::MissionGoal>> WaypointPlanner::createGoal(const std::string &goal) const {
  // Goal to be filled
  std::vector<iroc_mission_handler::MissionGoal> mission_robots;
  ROS_INFO("Received goal :%s ",
           goal.c_str()); // to remove

  result_t result;

  json json_msg;

  // Parsing JSON and creating robots JSON for post processing
  result = parseJson(goal, json_msg);

  if (!result.success) {
    result.success = false;
    result.message = "Faile to parse JSON msg";
    return std::make_tuple(result, mission_robots);
  }

  json robots;
  bool success = utils::parseVars(json_msg, {{"robots", &robots}});

  if (!result.success) {
    result.success = false;
    result.message = "Faile to parse robots field from JSON";
    return std::make_tuple(result, mission_robots);
  }

  mission_robots.reserve(robots.size());

  for (auto &robot : robots) {
    std::string name;
    std::vector<custom_types::Waypoint> points;
    int frame_id;
    int height;
    int height_id;
    int terminal_action;

    bool success = utils::parseVars(robot, {
                                               {"name", &name},
                                               {"points", &points},
                                               {"frame_id", &frame_id},
                                               {"height_id", &height_id},
                                               {"terminal_action", &terminal_action},
                                           });

    if (!success) {
      result.success = false;
      result.message = "Failure while parsing robot data, bad JSON request";
      return std::make_tuple(result, mission_robots);
    }

    bool isRobotInFleet = common_handlers_->handlers->robots_map.count(name);

    if (!isRobotInFleet) {
      ROS_WARN("[WaypointPlanner] Robot %s not within the fleet", name.c_str());
      std::stringstream ss;
      ss << name << " not found in the fleet!";
      result.message = ss.str();
      result.success = false;
      return std::make_tuple(result, mission_robots);
    }

    auto waypoints = toRosMsg<iroc_mission_handler::Waypoint>(points);

    iroc_mission_handler::MissionGoal robot_goal;
    robot_goal.name            = name;
    robot_goal.frame_id        = frame_id;
    robot_goal.height_id       = height_id;
    robot_goal.terminal_action = terminal_action;
    robot_goal.points          = waypoints;
    // Save the individual robot goal
    mission_robots.push_back(robot_goal);
  } // robots iteration

  ROS_INFO("[WaypointPlanner] Goal created successfully!");
  result.success = true;
  result.message = "Goal created successfully";
  return std::make_tuple(result, mission_robots);
}

} // namespace waypoint_planner

} // namespace planners

} // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::planners::waypoint_planner::WaypointPlanner, iroc_fleet_manager::planners::Planner);
