#include <iroc_fleet_manager/planner.h>

#include <mrs_lib/param_loader.h>
#include <string>

// Energy aware coverage planner library includes
#include <EnergyAwareMCPP/EnergyCalculator.h>
#include <EnergyAwareMCPP/MapPolygon.hpp>
#include <EnergyAwareMCPP/ShortestPathCalculator.hpp>
#include <EnergyAwareMCPP/SimpleLogger.h>
#include <EnergyAwareMCPP/algorithms.hpp>
#include <EnergyAwareMCPP/coverage_planner.hpp>
#include <EnergyAwareMCPP/mstsp_solver/MstspSolver.h>
#include <EnergyAwareMCPP/mstsp_solver/SolverConfig.h>
#include <EnergyAwareMCPP/utils.hpp>

#include <iroc_fleet_manager/msg/coverage_mission.hpp>
#include <iroc_fleet_manager/utils/conversions.h>
#include <mrs_msgs/msg/point2_d.hpp>

namespace iroc_fleet_manager
{

namespace planners
{

namespace coverage_planner
{

class CoveragePlanner : public iroc_fleet_manager::planners::Planner {
public:
  bool initialize(const rclcpp::Node::SharedPtr node, const std::string &name, const std::string &name_space,
                  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers) override;
  bool activate(void) override;
  void deactivate(void) override;
  std::tuple<result_t, std::vector<iroc_mission_handler::msg::MissionGoal>> createGoal(const std::string &goal) const override;

  std::string name_;

private:
  // Additional type for coverage planner
  typedef std::vector<std::vector<iroc_mission_handler::msg::Waypoint>> coverage_paths_t;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  mutable algorithm_config_t planner_config_;
  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers_;

  algorithm_config_t parse_algorithm_config(mrs_lib::ParamLoader &param_loader) const;

  coverage_paths_t getCoveragePaths(const iroc_fleet_manager::msg::CoverageMission &mission) const;
};

} // namespace coverage_planner
} // namespace planners
} // namespace iroc_fleet_manager
