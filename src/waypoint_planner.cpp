#include <memory>
#include <ros/ros.h>

/* includes //{ */
#include <any>
#include <iroc_fleet_manager/WaypointFleetManagerAction.h>
#include <iroc_fleet_manager/planner.h>

//}

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
createGoal(const std::string &goal) {
  ROS_INFO("Received goal :%s , to implement function", goal.c_str());
}

} // namespace waypoint_planner

} // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::waypoint_planner::WaypointPlanner,
                       iroc_fleet_manager::Planner);
