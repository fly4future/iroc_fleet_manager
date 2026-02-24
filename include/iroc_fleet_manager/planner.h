#pragma once

#include <iroc_common/result.h>
#include <iroc_fleet_manager/common_handlers.h>
#include <iroc_common/json_var_parser.h>
#include <iroc_mission_handler/action/mission.hpp>
#include <rclcpp/rclcpp.hpp>

namespace iroc_fleet_manager {

namespace planners {

// Re-export iroc_common::result_t under the planners namespace for convenience.
using result_t = iroc_common::result_t;

using json = nlohmann::json;

class Planner {
 public:
  /**
   * @brief Initializes the planner. Called once at startup.
   */
  virtual bool initialize(const rclcpp::Node::SharedPtr node, const std::string &name, const std::string &name_space,
                          std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers) = 0;

  /**
   * @brief Called before the planner is used. Should return quickly (milliseconds).
   */
  virtual bool activate(void) = 0;

  /**
   * @brief Called when this planner is no longer needed.
   */
  virtual void deactivate(void) = 0;

  /**
   * @brief Process an incoming JSON goal and return per-robot mission goals.
   */
  virtual std::tuple<result_t, std::vector<iroc_mission_handler::msg::MissionGoal>> createGoal(const std::string &goal) const = 0;

  virtual ~Planner() = default;

 protected:
  result_t parseJson(const std::string &goal, json &json_msg) const;
};

result_t Planner::parseJson(const std::string &goal, json &json_msg) const {
  result_t result;
  try {
    json_msg = json::parse(goal);
  }
  catch (const json::exception &e) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("IROCFleetManager"), "Bad json input: " << e.what());
    result.success = false;
    result.message = "BadRequest_400: Bad JSON input";
    return result;
  }
  result.success = true;
  return result;
}

} // namespace planners
} // namespace iroc_fleet_manager
