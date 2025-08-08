#include <iroc_fleet_manager/WaypointFleetManagerAction.h>
#include <iroc_fleet_manager/planner.h>
#include <string>

namespace iroc_fleet_manager {

namespace waypoint_planner {

class WaypointPlanner : public iroc_fleet_manager::Planner {
public:
  bool initialize(const ros::NodeHandle &parent_nh, const std::string &name,
                  const std::string &name_space,
                  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t>
                      common_handlers) override;

  bool activate(void) override;
  void deactivate(void) override;
  std::tuple<result_t, std::vector<iroc_mission_handler::MissionGoal>>
  createGoal(const std::string &goal) const override;

  std::string _name_;

private:
  bool is_initialized_ = false;
  bool is_active_ = false;
  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers_;
};

bool WaypointPlanner::initialize(
    const ros::NodeHandle &parent_nh, const std::string &name,
    const std::string &name_space,
    std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers) {

  // nh_ will behave just like normal NodeHandle
  ros::NodeHandle nh_(parent_nh, name_space);

  _name_ = name;
  common_handlers_ = common_handlers;

  ros::Time::waitForValid();

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[%s]: initialized under the name '%s', namespace '%s' and action ",
           _name_.c_str(), name.c_str(), name_space.c_str());

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

std::tuple<result_t, std::vector<iroc_mission_handler::MissionGoal>>
WaypointPlanner::createGoal(const std::string &goal) const {
  ROS_INFO("Received goal :%s ",
           goal.c_str()); // to remove

  std::vector<iroc_mission_handler::MissionGoal> mission_robots;
  result_t result;

  json json_msg, robots;

  // Parsing JSON and creating robots JSON for post processing
  result = parseJsonAndExtractRobots(goal, json_msg, robots);

  if (!result.success) {
    return std::make_tuple(result, mission_robots);
  }

  mission_robots.reserve(robots.size());
  for (auto &robot : robots) {
    std::string name;
    int frame_id;
    int height_id;
    int terminal_action;
    result = parseRobotBase(robot, name, frame_id, height_id, terminal_action);

    if (!result.success) {
      return std::make_tuple(result, mission_robots);
    }

    // Parse points
    json points;
    result = extractJsonArray(robot, "points", points);

    if (!result.success) {
      return std::make_tuple(result, mission_robots);
    }
    std::vector<iroc_mission_handler::Waypoint> waypoints;
    waypoints.reserve(points.size());

    for (const auto &point : points) {

      iroc_mission_handler::Waypoint waypoint;
      mrs_msgs::Reference ref;
      const auto succ = parse_vars(point, {{"x", &ref.position.x},
                                           {"y", &ref.position.y},
                                           {"z", &ref.position.z},
                                           {"heading", &ref.heading}});
      if (!succ) {
        ROS_WARN("Failed to parsed expected format of reference msg");
        result.success = false;
        result.message = "Failed to parsed expected format of reference msg";
        return std::make_tuple(result, mission_robots);
      }

      waypoint.reference_point = ref;

      if (point.contains("subtasks")) {
        std::vector<iroc_mission_handler::Subtask> subtasks;
        // Process subtask
        for (const auto &subtask : point) {
          iroc_mission_handler::Subtask subtask_obj;
          int type; // this will be changed to string in the future
          std::string parameters;
          const auto succ = parse_vars(
              subtask, {{"type", &type}, {"parameters", &parameters}});

          if (!succ) {
            ROS_WARN("Failed to parsed expected format of subtasks: 'type' and "
                     "'parameters'");
            result.success = false;
            result.message = "Failed to parsed expected format of subtasks: "
                             "'type' and 'parameters'";
            return std::make_tuple(result, mission_robots);
          }
          subtask_obj.type = type;
          subtask_obj.parameters = parameters;
          subtasks.push_back(subtask_obj);
        }

        waypoint.subtasks = subtasks;
      }
      // Saving the waypoint
      waypoints.push_back(waypoint);
    } // points iteration
    iroc_mission_handler::MissionGoal robot_goal;
    robot_goal.name = name;
    robot_goal.frame_id = frame_id;
    robot_goal.height_id = height_id;
    robot_goal.terminal_action = terminal_action;
    robot_goal.points = waypoints;
    // Save the individual robot goal
    mission_robots.push_back(robot_goal);
  } // robots iteration

  ROS_INFO("[WaypointPlanner] Goal created successfully!");
  result.success = true;
  result.message = "Goal created successfully";
  return std::make_tuple(result, mission_robots);
}

} // namespace waypoint_planner

} // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::waypoint_planner::WaypointPlanner,
                       iroc_fleet_manager::Planner);
