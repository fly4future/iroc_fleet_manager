#ifndef IROC_FLEET_MANAGER_PLANNER
#define IROC_FLEET_MANAGER_PLANNER

#include <iroc_mission_handler/MissionAction.h>
#include <ros/ros.h>

namespace iroc_fleet_manager {

class Planner {
public:
  /**
   * @brief Initializes the planner. It is called once for every planner. The
   * runtime is not limited.
   *
   * @param nh the node handle of the FleetManager
   * @param name of the planner for distinguishing multiple running instances of
   * the same code
   * @param name_space the parameter namespace of the planner, can be used
   * during initialization of the private node handle
   *
   * @return true if success
   */
  virtual bool initialize(const ros::NodeHandle &nh, const std::string &name,
                          const std::string &name_space,
                          const std::string &action_type) = 0;

  /**
   * @brief It is called before the planner will be required and used. Should
   * not take much time (within miliseconds).
   *
   * @return true if success
   */
  virtual bool activate(void) = 0;

  /**
   * @brief is called when this planner is no longer needed. However, it can be
   * activated later.
   */
  virtual void deactivate(void) = 0;

  /**
   * @brief Request for planner to process an incoming goal
   * @param incoming goal for the planner, string with JSON format type
   * @return the goals of the robots in the fleet.
   */
  virtual std::vector<iroc_mission_handler::MissionGoal>
  createGoal(const std::string &goal) const = 0;

  virtual ~Planner() = default;
};

} // namespace iroc_fleet_manager

#endif
