#include <iroc_fleet_manager/planner.h>
#include <string>

namespace iroc_fleet_manager {

namespace autonomy_test_planner {

class AutonomyTestPlanner : public iroc_fleet_manager::Planner {
 public:
  bool initialize(const ros::NodeHandle& parent_nh, const std::string& name, const std::string& name_space,
                  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers) override;

  bool activate(void) override;
  void deactivate(void) override;
  std::tuple<result_t, std::vector<iroc_mission_handler::MissionGoal>> createGoal(const std::string& goal) const override;
  std::vector<iroc_mission_handler::Waypoint> getAutonomyPoints(double segment_length) const;

  std::string _name_;

 private:
  bool is_initialized_ = false;
  bool is_active_ = false;
  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers_;
};

bool AutonomyTestPlanner::initialize(const ros::NodeHandle& parent_nh, const std::string& name, const std::string& name_space,
                                     std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers) {

  // nh_ will behave just like normal NodeHandle
  ros::NodeHandle nh_(parent_nh, name_space);

  _name_ = name;
  common_handlers_ = common_handlers;

  ros::Time::waitForValid();

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[%s]: initialized under the name '%s', namespace '%s' and action ", _name_.c_str(), name.c_str(), name_space.c_str());

  is_initialized_ = true;
  return true;
}

bool AutonomyTestPlanner::activate(void) {

  int some_number = 0;
  ROS_INFO("[%s]: activated with some_number=%d", _name_.c_str(), some_number);

  is_active_ = true;

  return true;
}

void AutonomyTestPlanner::deactivate(void) {

  is_active_ = false;

  ROS_INFO("[%s]: deactivated", _name_.c_str());
}

std::tuple<result_t, std::vector<iroc_mission_handler::MissionGoal>> AutonomyTestPlanner::createGoal(const std::string& goal) const {
  ROS_INFO("Received goal :%s ",
           goal.c_str()); // to remove

  std::vector<iroc_mission_handler::MissionGoal> mission_robots;
  result_t result;

  json json_msg, robots;

  // Parsing JSON and creating robots JSON for post processing
  result = parseJson(goal, json_msg);

  if (!result.success) {
    return std::make_tuple(result, mission_robots);
  }

  bool success = parseVars(json_msg, {{"robots", &robots}});

  mission_robots.reserve(robots.size());
  for (auto& robot : robots) {
    std::string name;
    int segment_length;

    if (!result.success) {
      return std::make_tuple(result, mission_robots);
    }

    const auto succ = parseVars(robot, {{"name", &name}, {"segment_length", &segment_length}});

    if (!succ) {
      result.success = false;
      result.message = "Failed to parse parameters";
      return std::make_tuple(result, mission_robots);
    }

    bool isRobotInFleet = common_handlers_->handlers->robots_map.count(name); 

    if (!isRobotInFleet) {
      ROS_WARN("[AutonomyTestPlanner] Robot %s not within the fleet", name.c_str());
      std::stringstream ss;
      ss << name << " not found in the fleet!";
      result.message = ss.str(); 
      result.success = false;
      return std::make_tuple(result, mission_robots);
    }

    iroc_mission_handler::MissionGoal robot_goal;
    robot_goal.name = name;
    robot_goal.frame_id = iroc_mission_handler::MissionGoal::FRAME_ID_FCU;
    robot_goal.height_id = iroc_mission_handler::MissionGoal::HEIGHT_ID_FCU;
    robot_goal.terminal_action = 0;
    robot_goal.points = getAutonomyPoints(segment_length);
    // Save the individual robot goal
    mission_robots.push_back(robot_goal);
  } // robots iteration

  ROS_INFO("[AutonomyTestPlanner] Goal created successfully!");
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
std::vector<iroc_mission_handler::Waypoint> AutonomyTestPlanner::getAutonomyPoints(double segment_length) const {

  std::vector<mrs_msgs::Reference> points;
  mrs_msgs::Reference point;

  // Center point
  point.position.x = 0.0;
  point.position.y = 0.0;
  point.position.z = 0.0;
  point.heading = 0.0;

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
  point.heading = 0.0; // 90 degrees
  points.push_back(point);

  point.heading = (2 * M_PI) / 3.0;
  points.push_back(point);

  point.heading = (4 * M_PI) / 3.0;
  points.push_back(point);

  point.heading = (2 * M_PI);
  points.push_back(point);

  // // 360-degree pirouette (3 points)
  point.heading = 0.0; // 90 degrees
  points.push_back(point);

  point.heading = (2 * M_PI) / 3.0;
  points.push_back(point);

  point.heading = (4 * M_PI) / 3.0;
  points.push_back(point);

  point.heading = (2 * M_PI);
  points.push_back(point);

  // Convert mrs_msgs::Reference to iroc_mission_handler::Waypoint
  std::vector<iroc_mission_handler::Waypoint> autonomy_points;
  autonomy_points.reserve(points.size());
  for (const auto& p : points) {
    iroc_mission_handler::Waypoint wp;
    wp.reference.position.x = p.position.x;
    wp.reference.position.y = p.position.y;
    wp.reference.position.z = p.position.z;
    wp.reference.heading = p.heading;

    autonomy_points.push_back(wp);
  }

  return autonomy_points;
}
} // namespace autonomy_test_planner

} // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::autonomy_test_planner::AutonomyTestPlanner, iroc_fleet_manager::Planner);
