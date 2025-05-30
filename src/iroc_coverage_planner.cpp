/* includes //{ */
#include <iroc_fleet_manager/base_fleet_manager.h>
#include <iroc_fleet_manager/CoverageMissionAction.h>

// Coverage planner library includes
#include <CoveragePlannerLib/coverage_planner.hpp>
#include <CoveragePlannerLib/MapPolygon.hpp>
#include <CoveragePlannerLib/EnergyCalculator.h>
#include <CoveragePlannerLib/algorithms.hpp>
#include <CoveragePlannerLib/ShortestPathCalculator.hpp>
#include <CoveragePlannerLib/mstsp_solver/SolverConfig.h>
#include <CoveragePlannerLib/mstsp_solver/MstspSolver.h>
#include <CoveragePlannerLib/SimpleLogger.h>
#include <CoveragePlannerLib/utils.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

//}

namespace iroc_fleet_manager
{

/* class IROC_CoverageManager //{ */

class IROC_CoverageManager : public iroc_fleet_manager::BaseFleetManager<iroc_fleet_manager::CoverageMissionAction> {
public:
  std::vector<iroc_mission_handler::MissionRobotGoal> processGoal(const iroc_fleet_manager::CoverageMissionGoal& goal) const override;

private:

  mutable algorithm_config_t planner_config_;

  // Additional type for coverage planner
  typedef std::vector<std::vector<mrs_msgs::Reference>> coverage_paths_t;

  // | ------------------ Additional functions ------------------ |
  algorithm_config_t parse_algorithm_config(mrs_lib::ParamLoader& param_loader) const;
  coverage_paths_t getCoveragePaths(const iroc_fleet_manager::CoverageMission& mission) const; 
};
//}

// | -------------------- support functions ------------------- |

/* processGoal //{ */

std::vector<iroc_mission_handler::MissionRobotGoal>
IROC_CoverageManager::processGoal(const iroc_fleet_manager::CoverageMissionGoal& goal) const {

  /* load parameters */
  mrs_lib::ParamLoader param_loader(nh_, "IROC_CoverageManager");

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");

  planner_config_ = parse_algorithm_config(param_loader); 

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[IROC_CoverageManager]: Could not load all parameters!");
    ros::shutdown();
  }

  std::vector<iroc_mission_handler::MissionRobotGoal> mission_robots; 
  auto paths = getCoveragePaths(goal.mission);  
  ROS_INFO("[IROC_CoverageManager:]: Coverage paths size: %zu", paths.size());

  // Filling the mission_robots vector with the generated paths
  for (int it = 0; it < goal.mission.robots.size() ; it++) {
    iroc_mission_handler::MissionRobotGoal robot;
    robot.name = goal.mission.robots[it].name;
    robot.points = paths[it];
    robot.terminal_action = goal.mission.robots[it].terminal_action;
    robot.height_id = goal.mission.robots[it].height_id;
    robot.frame_id = goal.mission.robots[it].frame_id;
    mission_robots.push_back(robot);
  }

  return mission_robots;
}

//}

/* parse_algorithm_config() //{ */

algorithm_config_t IROC_CoverageManager::parse_algorithm_config(mrs_lib::ParamLoader& param_loader ) const {
  const std::string yaml_prefix = "coverage_planner/";
  algorithm_config_t algorithm_config;

  // Load basic drone parameters
  param_loader.loadParam(yaml_prefix + "drone_mass", algorithm_config.energy_calculator_config.drone_mass);
  param_loader.loadParam(yaml_prefix + "drone_area", algorithm_config.energy_calculator_config.drone_area);
  param_loader.loadParam(yaml_prefix + "average_acceleration", algorithm_config.energy_calculator_config.average_acceleration);
  param_loader.loadParam(yaml_prefix + "propeller_radius", algorithm_config.energy_calculator_config.propeller_radius);
  param_loader.loadParam(yaml_prefix + "number_of_propellers", algorithm_config.energy_calculator_config.number_of_propellers);
  param_loader.loadParam(yaml_prefix + "allowed_path_deviation", algorithm_config.energy_calculator_config.allowed_path_deviation);
  param_loader.loadParam(yaml_prefix + "number_of_rotations", algorithm_config.number_of_rotations);

  // Load battery model parameters
  const std::string battery_prefix = yaml_prefix + "battery_model/";
  param_loader.loadParam(battery_prefix + "cell_capacity", algorithm_config.energy_calculator_config.battery_model.cell_capacity);
  param_loader.loadParam(battery_prefix + "number_of_cells", algorithm_config.energy_calculator_config.battery_model.number_of_cells);
  param_loader.loadParam(battery_prefix + "d0", algorithm_config.energy_calculator_config.battery_model.d0);
  param_loader.loadParam(battery_prefix + "d1", algorithm_config.energy_calculator_config.battery_model.d1);
  param_loader.loadParam(battery_prefix + "d2", algorithm_config.energy_calculator_config.battery_model.d2);
  param_loader.loadParam(battery_prefix + "d3", algorithm_config.energy_calculator_config.battery_model.d3);

  // Load speed model parameters
  const std::string speed_prefix = yaml_prefix + "best_speed_model/";
  param_loader.loadParam(speed_prefix + "c0", algorithm_config.energy_calculator_config.best_speed_model.c0);
  param_loader.loadParam(speed_prefix + "c1", algorithm_config.energy_calculator_config.best_speed_model.c1);
  param_loader.loadParam(speed_prefix + "c2", algorithm_config.energy_calculator_config.best_speed_model.c2);

  // Load coordinate system parameters
  param_loader.loadParam(yaml_prefix + "points_in_lat_lon", algorithm_config.points_in_lat_lon);
  if (algorithm_config.points_in_lat_lon) {
    param_loader.loadParam(yaml_prefix + "latitude_origin", algorithm_config.lat_lon_origin.first);
    param_loader.loadParam(yaml_prefix + "longitude_origin", algorithm_config.lat_lon_origin.second);
  }

  param_loader.loadParam(yaml_prefix + "sweeping_step", algorithm_config.sweeping_step);

  int decomposition_method;
  param_loader.loadParam(yaml_prefix + "decomposition_method", decomposition_method);
  algorithm_config.decomposition_type = static_cast<decomposition_type_t>(decomposition_method);

  param_loader.loadParam(yaml_prefix + "min_sub_polygons_per_uav", algorithm_config.min_sub_polygons_per_uav);

  // Load optimization parameters
  param_loader.loadParam(yaml_prefix + "rotations_per_cell", algorithm_config.rotations_per_cell);
  param_loader.loadParam(yaml_prefix + "no_improvement_cycles_before_stop", algorithm_config.no_improvement_cycles_before_stop);
  param_loader.loadParam(yaml_prefix + "max_single_path_energy", algorithm_config.max_single_path_energy);

  return algorithm_config;
}
//}

/* calculate_centroid() //{ */
// Calculate centroid of a polygon
point_t calculate_centroid(const MapPolygon& polygon) {
  auto points = polygon.get_all_points();
  double sum_x = 0, sum_y = 0;
  for (const auto& point : points) {
    sum_x += point.first;
    sum_y += point.second;
  }
  return {sum_x / points.size(), sum_y / points.size()};
}
//}

/* calculate_distance() //{ */
// Calculate Euclidean distance between two points
double calculate_distance(const point_t& p1, const point_t& p2) {
  return std::sqrt(std::pow(p1.first - p2.first, 2) + std::pow(p1.second - p2.second, 2));
}
//}

/* assign_closest_polygons() //{ */
// Assign each UAV to its closest polygon and return the assigned polygons
std::vector<MapPolygon> assign_closest_polygons(const std::vector<point_t>& uav_positions, 
                                            std::vector<MapPolygon> polygons) {
  std::vector<MapPolygon> assigned_polygons(uav_positions.size());
  std::vector<point_t> polygon_centroids;
  
  // Calculate centroids for all polygons
  for (const auto& poly : polygons) {
    polygon_centroids.push_back(calculate_centroid(poly));
  }
  
  // For each UAV, find the closest unassigned polygon
  for (size_t i = 0; i < uav_positions.size(); i++) {
    double min_distance = std::numeric_limits<double>::max();
    size_t closest_polygon_idx = 0;
    
    for (size_t j = 0; j < polygon_centroids.size(); j++) {
      double dist = calculate_distance(uav_positions[i], polygon_centroids[j]);
      if (dist < min_distance) {
        min_distance = dist;
        closest_polygon_idx = j;
      }
    }
    
    // Assign the closest polygon to this UAV
    assigned_polygons[i] = polygons[closest_polygon_idx];
    
    // Remove the assigned polygon and its centroid from consideration
    polygons.erase(polygons.begin() + closest_polygon_idx);
    polygon_centroids.erase(polygon_centroids.begin() + closest_polygon_idx);
  }
  
  return assigned_polygons;
}
//}

/* getCoveragePaths() //{ */
IROC_CoverageManager::coverage_paths_t IROC_CoverageManager::getCoveragePaths(const iroc_fleet_manager::CoverageMission& mission) const { 

  // Fly zone and no fly zones
  std::vector<point_t> fly_zone; 
  std::vector<std::vector<point_t>> no_fly_zones;

  // Fill the search area 
  for (const auto& point : mission.search_area) 
    fly_zone.emplace_back(point.x, point.y);

  // Add the first point to close the polygon
  fly_zone.emplace_back(mission.search_area[0].x, mission.search_area[0].y);

  planner_config_.lat_lon_origin.first = mission.latlon_origin.x;
  planner_config_.lat_lon_origin.second = mission.latlon_origin.y;

  // Initialize polygon and transform all the point into meters
  MapPolygon polygon = MapPolygon(fly_zone, no_fly_zones, planner_config_.lat_lon_origin);

  // Decompose the polygon
  ShortestPathCalculator shortest_path_calculator(polygon);

  // Create a logger to log everything directly into stdout
  auto shared_logger = std::make_shared<loggers::SimpleLogger>();
  EnergyCalculator energy_calculator{planner_config_.energy_calculator_config, shared_logger};
  ROS_INFO_STREAM("[IROC_CoverageManager:]: Energy calculator created. Optimal speed: " << energy_calculator.get_optimal_speed());

  //Decompose polygon for each UAV
  auto decomposed_polygon = decompose_polygon(mission.robots.size(),planner_config_, polygon);

  std::vector<point_t> polygon_centroids;
  for (const auto& polygon : decomposed_polygon) {
    auto points_tmp = polygon.get_all_points();
    auto m_polygon_points = std::vector<point_t>{points_tmp.begin(), points_tmp.end()};
    polygon_centroids.emplace_back(calculate_centroid(polygon));
  }

  // Create a vector of UAV positions
  std::vector<point_t> uav_positions;
  for (const auto& robot : mission.robots) {
      uav_positions.push_back({robot.local_position.x, robot.local_position.y});
  }

  // Assign each UAV to its closest polygon
  auto assigned_polygons = assign_closest_polygons(uav_positions, decomposed_polygon);

  ROS_INFO("[IROC_CoverageManager:]: Size of decomposed polygon: %zu", decomposed_polygon.size());

  // For saving the paths for each UAV
  coverage_paths_t coverage_paths;
  // For accessing the robots information from mission goal
  int uav_index = 0;
  for (const auto& polygon : decomposed_polygon) {
    mstsp_solver::final_solution_t best_solution;
    try
    {

      const auto& assigned_polygon = assigned_polygons[uav_index];
      // Setting the start position for each UAV
      planner_config_.start_pos.first = mission.robots.at(uav_index).global_position.x;
      planner_config_.start_pos.second= mission.robots.at(uav_index).global_position.y;
      auto f = [&](int n) { return solve_for_uavs(n, planner_config_, assigned_polygon, energy_calculator, shortest_path_calculator, shared_logger); };
      best_solution = generate_with_constraints(planner_config_.max_single_path_energy * 3600, 1, f);
      uav_index++;
    }
    catch (const polygon_decomposition_error& e)
    {
      ROS_WARN_STREAM("[IROC_CoverageManager:]: Error while decomposing the polygon");  
      return coverage_paths_t();
    }

    auto best_paths = best_solution.paths;
    for (auto& path : best_paths)
    {
      std::vector<mrs_msgs::Reference> coverage_path;
      mrs_msgs::Reference point; 
      for (auto& p : path)
      {
        // TODO Replace with mrs_lib transformer?
        auto lat_lon_p = meters_to_gps_coordinates({p.x, p.y}, planner_config_.lat_lon_origin);
        p.x = lat_lon_p.first;
        p.y = lat_lon_p.second;
        // Fill the reference point
        point.position.x = p.x;
        point.position.y = p.y;
        point.position.z = mission.robots[0].height; // We are using same height for all robots 
        point.heading = 0.0;
        coverage_path.push_back(point);
      }
      coverage_paths.push_back(coverage_path);
    }  
  }

  return coverage_paths;
}
//}

}  // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::IROC_CoverageManager, nodelet::Nodelet);
