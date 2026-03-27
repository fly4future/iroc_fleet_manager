#include <iroc_fleet_manager/planner.h>
#include <ros/package.h>

#include <mrs_lib/param_loader.h>
#include <string>

#include <typeinfo>
#include <cmath>

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

#include <CzAltitude/DMR5GElevationGrid.hpp>

#include <iroc_fleet_manager/CoverageMission.h>
#include <iroc_fleet_manager/CoverageMissionRobot.h>
#include <iroc_fleet_manager/utils/conversions.h>
#include <mrs_msgs/Point2D.h>

namespace iroc_fleet_manager
{

namespace planners
{

namespace cz_altitude_coverage_planner
{

class CzAltitudeCoveragePlanner : public iroc_fleet_manager::planners::Planner {
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

  std::string dmr5g_files_path;

  mutable algorithm_config_t planner_config_;
  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers_;

  algorithm_config_t parse_algorithm_config(mrs_lib::ParamLoader &param_loader) const;

  coverage_paths_t getCoveragePaths(const iroc_fleet_manager::CoverageMission &mission) const;

  std::tuple<result_t, std::vector<iroc_mission_handler::MissionGoal>> endWithError(result_t result, const std::string& error_msg, std::vector<iroc_mission_handler::MissionGoal> mission_robots) const;
  double toRadians(double degree) const;
  double distanceLatLon(double lat1, double lon1, double lat2, double lon2) const;
};

} // namespace cz_altitude_coverage_planner
} // namespace planners
} // namespace iroc_fleet_manager
