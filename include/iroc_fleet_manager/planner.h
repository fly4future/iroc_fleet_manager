#ifndef IROC_FLEET_MANAGER_PLANNER
#define IROC_FLEET_MANAGER_PLANNER

#include <iroc_fleet_manager/common_handlers.h>
#include <iroc_fleet_manager/json_var_parser.h>
#include <iroc_mission_handler/MissionAction.h>
#include <ros/ros.h>

namespace iroc_fleet_manager {

struct result_t {
  bool success;
  std::string message;
};

using json = nlohmann::json;

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
                          std::shared_ptr<iroc_fleet_manager::CommonHandlers_t>
                              common_handlers) = 0;

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
  virtual std::tuple<result_t, std::vector<iroc_mission_handler::MissionGoal>>
  createGoal(const std::string &goal) const = 0;

  virtual ~Planner() = default;

protected:
  result_t parseJsonAndExtractRobots(const std::string &goal, json &json_msg,
                                     json &robots) const;
  result_t extractJsonArray(json &input_json, const std::string &key,
                            json &output_array) const;
  result_t parseRobotBase(const json &robot, std::string &name, int &frame_id,
                          int &height_id, int &terminal_action) const;
};

result_t Planner::parseJsonAndExtractRobots(const std::string &goal,
                                            json &json_msg,
                                            json &robots) const {

  result_t result;
  try {
    json_msg = json::parse(goal);
  } catch (const json::exception &e) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[Planner]: Bad json input: " << e.what());
    result.success = false;
    result.message = "BadRequest_400: Bad JSON input";
    return result;
  }

  // Extract robots
  const auto response = extractJsonArray(json_msg, "robots", robots);

  if (!response.success) {
    result.success = response.success;
    result.message = response.message;
    return result;
  }

  result.success = true;
  return result;
}

result_t Planner::extractJsonArray(json &input_json, const std::string &key,
                                   json &output_array) const {

  result_t result;
  // Extract key array
  const auto succ = parse_vars(input_json, {{key, &output_array}});
  std::stringstream ss;
  if (!succ) {
    ss << "Missing " << key << "key";
    result.success = false;
    result.message = ss.str();
    return result;
  }

  if (!output_array.is_array()) {
    result.success = false;
    ss << "Bad mission input" << key << "is not an array";
    result.message = ss.str();
    return result;
  }
  result.success = true;
  result.message = "Success";
  return result;
}

result_t Planner ::parseRobotBase(const json &robot, std::string &name,
                                  int &frame_id, int &height_id,
                                  int &terminal_action) const {
  result_t result;

  const auto succ = parse_vars(robot, {{"name", &name},
                                       {"frame_id", &frame_id},
                                       {"height_id", &height_id},
                                       {"terminal_action", &terminal_action}});

  if (!succ) {
    result.success = false;
    result.message = "Failure parsing the expected base keys for robot.";
  } else {
    result.success = true;
  }

  return result;
}

} // namespace iroc_fleet_manager

#endif
