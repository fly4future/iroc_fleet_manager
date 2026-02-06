#include <iroc_fleet_manager/utils/conversions.h>
#include <iroc_fleet_manager/planner.h>
#include <string>

namespace iroc_fleet_manager
{

namespace planners
{

namespace waypoint_planner
{

class WaypointPlanner : public iroc_fleet_manager::planners::Planner {
public:
  bool initialize(const rclcpp::Node::SharedPtr node, const std::string &name, const std::string &name_space,
                  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers) override;

  bool activate(void) override;
  void deactivate(void) override;
  std::tuple<result_t, std::vector<iroc_mission_handler::msg::MissionGoal>> createGoal(const std::string &goal) const override;

  std::string _name_;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;
  bool is_initialized_ = false;
  bool is_active_      = false;
  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers_;
};

} // namespace waypoint_planner
} // namespace planners
} // namespace iroc_fleet_manager
