/* includes //{ */
#include <iroc_fleet_manager/base_fleet_manager.h>
#include <iroc_fleet_manager/AutonomyTestAction.h>
//}

namespace iroc_fleet_manager
{

/* class IROCAutonomyTestManager //{ */

class IROCAutonomyTestManager : public iroc_fleet_manager::BaseFleetManager<iroc_fleet_manager::AutonomyTestAction> {
public:
  std::vector<iroc_fleet_manager::WaypointMissionRobot> processGoal(const iroc_fleet_manager::AutonomyTestGoal& goal) const override;

private:
  std::vector<mrs_msgs::Reference> getAutonomyPoints(double segment_length) const;
};
//}

/* processGoal //{ */

std::vector<iroc_fleet_manager::WaypointMissionRobot>
IROCAutonomyTestManager::processGoal(const iroc_fleet_manager::AutonomyTestGoal& goal) const {

  std::vector<iroc_fleet_manager::WaypointMissionRobot> mission_robots; 

  iroc_fleet_manager::WaypointMissionRobot waypoint_robot;
  // Fill in the waypoint mission robots
  for (const auto& robot: goal.robots) {
    waypoint_robot.name      = robot.name;
    waypoint_robot.frame_id  = MissionHandlerActionServerGoal::FRAME_ID_FCU; //Using current local _frame
    waypoint_robot.height_id = MissionHandlerActionServerGoal::HEIGHT_ID_FCU; //Defining FCU height
    waypoint_robot.points    = getAutonomyPoints(robot.segment_length);
    mission_robots.push_back(waypoint_robot);
  }

  return mission_robots;
}
//}

// | -------------------- support functions ------------------- |

/* getAutonomyPoints() //{ */
std::vector<mrs_msgs::Reference> IROCAutonomyTestManager::getAutonomyPoints(double segment_length) const {

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
  point.heading = 0.0;  // 90 degrees
  points.push_back(point);

  point.heading = (2* M_PI) / 3.0;  
  points.push_back(point);

  point.heading = (4* M_PI) / 3.0;  
  points.push_back(point);

  point.heading = (2* M_PI);  
  points.push_back(point);

  // // 360-degree pirouette (3 points)
  point.heading = 0.0;  // 90 degrees
  points.push_back(point);

  point.heading = (2* M_PI) / 3.0;  
  points.push_back(point);

  point.heading = (4* M_PI) / 3.0;  
  points.push_back(point);

  point.heading = (2* M_PI);  
  points.push_back(point);

  return points;

}

//}

}  // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::IROCAutonomyTestManager, nodelet::Nodelet);
