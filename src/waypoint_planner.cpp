#include <ros/ros.h>

/* includes //{ */
// #include <iroc_fleet_manager/WaypointFleetManagerAction.h>
#include <iroc_fleet_manager/planner.h>
//}

namespace iroc_fleet_manager {

namespace waypoint_planner {

/* class WaypointPlanner //{ */

class WaypointPlanner : public iroc_fleet_manager::Planner {
public:
  bool initialize(const ros::NodeHandle &parent_nh, const std::string &name,
                  const std::string &name_space);

  bool activate(void);
  void deactivate(void);

  std::string _name_;

private:
  bool is_initialized_ = false;
  bool is_active_ = false;
};

bool WaypointPlanner::initialize(const ros::NodeHandle &parent_nh,
                                 const std::string &name,
                                 const std::string &name_space) {

  // nh_ will behave just like normal NodeHandle
  ros::NodeHandle nh_(parent_nh, name_space);

  _name_ = name;

  ros::Time::waitForValid();

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[%s]: initialized under the name '%s', and namespace '%s'",
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

// std::vector<iroc_mission_handler::MissionGoal> processGoal(
//     const iroc_fleet_manager::WaypointFleetManagerGoal &goal) const override;
// };
//}

/* processGoal //{ */

// std::vector<iroc_mission_handler::MissionGoal>
// WaypointPlanner::processGoal(
//     const iroc_fleet_manager::WaypointFleetManagerGoal &goal) const {
//
//   return goal.robots;
// }
//}

} // namespace waypoint_planner

} // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::waypoint_planner::WaypointPlanner,
                       iroc_fleet_manager::Planner);
