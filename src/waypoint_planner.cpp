#include <iroc_fleet_manager/WaypointFleetManagerAction.h>
#include <iroc_fleet_manager/planner.h>

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
  std::vector<iroc_mission_handler::MissionGoal> mission_robots;
  ROS_INFO("Received goal :%s , (WIP) to implement function", goal.c_str());

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
    ROS_WARN_STREAM_THROTTLE(1.0, "[WaypointPlanner]: Bad mission input: Expected an array.");

    // res.status = httplib::StatusCode::BadRequest_400;
    // ss << "Bad mission input: Expected an array";
    // json json_response_msg = {{"message", ss.str()}};
    // res.set_content(json_response_msg.dump(), "application/json");
    // return;
  }

  return mission_robots;
}

} // namespace waypoint_planner

} // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::waypoint_planner::WaypointPlanner,
                       iroc_fleet_manager::Planner);
