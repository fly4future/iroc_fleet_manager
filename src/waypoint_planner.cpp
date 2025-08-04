#include <iroc_fleet_manager/WaypointFleetManagerAction.h>
#include <iroc_fleet_manager/planner.h>
#include <string>

namespace iroc_fleet_manager {

namespace waypoint_planner {

// using ActionServer_T =
// actionlib::SimpleActionServer<WaypointFleetManagerAction>;
/* class WaypointPlanner //{ */

class WaypointPlanner : public iroc_fleet_manager::Planner {
public:
  bool initialize(const ros::NodeHandle &parent_nh, const std::string &name,
                  const std::string &name_space,
                  const std::string &action_type) override;

  bool activate(void) override;
  void deactivate(void) override;
  std::vector<iroc_mission_handler::MissionGoal>
  createGoal(const std::string &goal) const override;

  std::string _name_;
  std::string _action_type_;

private:
  bool is_initialized_ = false;
  bool is_active_ = false;
};

bool WaypointPlanner::initialize(const ros::NodeHandle &parent_nh,
                                 const std::string &name,
                                 const std::string &name_space,
                                 const std::string &action_type) {

  // nh_ will behave just like normal NodeHandle
  ros::NodeHandle nh_(parent_nh, name_space);

  _name_ = name;
  _action_type_ = action_type;

  ros::Time::waitForValid();

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[%s]: initialized under the name '%s', namespace '%s' and action "
           "type '%s'",
           _name_.c_str(), name.c_str(), name_space.c_str(),
           action_type.c_str());

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

std::vector<iroc_mission_handler::MissionGoal>
WaypointPlanner::createGoal(const std::string &goal) const {
  ROS_INFO("Received goal :%s , (WIP) to implement function", goal.c_str()); // to remove

  json json_msg;
  try {
    json_msg = json::parse(goal);
  } catch (const json::exception &e) {
    ROS_ERROR_STREAM_THROTTLE(
        1.0, "[WaypointPlanner]: Bad json input: " << e.what());
  }

  json robots;
  const auto succ = parse_vars(json_msg, {{"robots", &robots}});

  // if (!succ)
  // return;

  if (!robots.is_array()) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[WaypointPlanner]: Bad mission input: Expected an array.");
    // TODO: return a proper response
  }

  // Process the robots assigned into the mission
  std::vector<iroc_mission_handler::MissionGoal> mission_robots;
  mission_robots.reserve(robots.size());

  for (const auto &robot : robots) {

    int frame_id;
    int height_id;
    int terminal_action;
    std::string name;
    json points;
    const auto succ =
        parse_vars(robot, {{"name", &name},
                           {"frame_id", &frame_id},
                           {"height_id", &height_id},
                           {"points", &points},
                           {"terminal_action", &terminal_action}});

    if (!succ)
      ROS_WARN("(WIP) Something failed, to be implemented");
    // return; // TODO: return a proper response

    if (!points.is_array()) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "[WaypointPlanner]: Bad points input: Expected an array.");
      // return; // TODO: return a proper response
    }

    // TODO: How will validate the available robots?
    {
    // std::stringstream ss;
    // std::scoped_lock lck(robot_handlers_.mtx);
    // auto *rh_ptr = findRobotHandler(name, robot_handlers_);
    //
    // if (!rh_ptr) {
    //   ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot \""
    //                                      << name
    //                                      << "\" not found. Ignoring.");
    //   res.status = httplib::StatusCode::BadRequest_400;
    //   ss << "robot \"" << name << "\" not found, ignoring";
    //   json json_response_msg = {{"message", ss.str()}};
    //   res.set_content(json_response_msg.dump(), "application/json");
    //   return;
    // }
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
      if (!succ)
        // return; // TODO: return proper response
        ROS_INFO("(WIP), TODO, we need to return a proper response here");

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

          if (!succ)
            // return; // TODO: return proper response
            ROS_INFO("(WIP), TODO, we need to return a proper response here");
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

  return mission_robots;
}

} // namespace waypoint_planner

} // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::waypoint_planner::WaypointPlanner,
                       iroc_fleet_manager::Planner);
