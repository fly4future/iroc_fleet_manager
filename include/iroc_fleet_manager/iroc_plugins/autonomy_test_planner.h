#pragma once

#include <iroc_fleet_manager/planner.h>
#include <string>

namespace iroc_fleet_manager
{

namespace planners
{

namespace autonomy_test_planner
{

class AutonomyTestPlanner : public iroc_fleet_manager::planners::Planner {
public:
  bool initialize(const rclcpp::Node::SharedPtr node, const std::string &name, const std::string &name_space,
                  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers) override;

  bool activate(void) override;
  void deactivate(void) override;
  std::tuple<result_t, std::vector<iroc_mission_handler::msg::MissionGoal>> createGoal(const std::string &goal) const override;
  std::vector<iroc_mission_handler::msg::Waypoint> getAutonomyPoints(double segment_length) const;

  std::string name_;

private:
  rclcpp::Node::SharedPtr node_;

  bool is_initialized_ = false;
  bool is_active_      = false;
  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers_;
};

} // namespace autonomy_test_planner
} // namespace planners
} // namespace iroc_fleet_manager
