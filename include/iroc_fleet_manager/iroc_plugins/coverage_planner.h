#include <iroc_fleet_manager/planner.h>
#include <ros/package.h>

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

#include <iroc_fleet_manager/CoverageMission.h>
#include <iroc_fleet_manager/CoverageMissionRobot.h>
#include <iroc_fleet_manager/utils/conversions.h>
#include <mrs_msgs/Point2D.h>

namespace iroc_fleet_manager
{

namespace planners
{

namespace coverage_planner
{

class CoveragePlanner : public iroc_fleet_manager::planners::Planner {
public:
  bool initialize(const ros::NodeHandle &parent_nh, const std::string &name, const std::string &name_space,
                  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers) override;

  bool activate(void) override;
  void deactivate(void) override;
  std::tuple<result_t, std::vector<iroc_mission_handler::MissionGoal>> createGoal(const std::string &goal) const override;

  std::string name_;

private:
  // Additional type for coverage planner
  typedef std::vector<std::vector<iroc_mission_handler::Waypoint>> coverage_paths_t;

  bool is_initialized_ = false;
  bool is_active_      = false;

  mutable algorithm_config_t planner_config_;
  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers_;

  algorithm_config_t parse_algorithm_config(mrs_lib::ParamLoader &param_loader) const;

  coverage_paths_t getCoveragePaths(const iroc_fleet_manager::CoverageMission &mission) const;
};

} // namespace coverage_planner
} // namespace planners
} // namespace iroc_fleet_manager
