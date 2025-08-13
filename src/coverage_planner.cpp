#include <iroc_fleet_manager/WaypointFleetManagerAction.h>
#include <iroc_fleet_manager/planner.h>
#include <ros/package.h>

#include <mrs_lib/param_loader.h>
#include <string>

// Coverage planner library includes
#include <CoveragePlannerLib/EnergyCalculator.h>
#include <CoveragePlannerLib/MapPolygon.hpp>
#include <CoveragePlannerLib/ShortestPathCalculator.hpp>
#include <CoveragePlannerLib/SimpleLogger.h>
#include <CoveragePlannerLib/algorithms.hpp>
#include <CoveragePlannerLib/coverage_planner.hpp>
#include <CoveragePlannerLib/mstsp_solver/MstspSolver.h>
#include <CoveragePlannerLib/mstsp_solver/SolverConfig.h>
#include <CoveragePlannerLib/utils.hpp>
#include <iroc_fleet_manager/CoverageMission.h>
#include <iroc_fleet_manager/CoverageMissionRobot.h>
#include <iroc_fleet_manager/conversions.h>
#include <mrs_msgs/Point2D.h>

namespace iroc_fleet_manager {

namespace coverage_planner {

class CoveragePlanner : public iroc_fleet_manager::Planner {
 public:
  bool initialize(const ros::NodeHandle& parent_nh, const std::string& name, const std::string& name_space,
                  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers) override;

  bool activate(void) override;
  void deactivate(void) override;
  std::tuple<result_t, std::vector<iroc_mission_handler::MissionGoal>> createGoal(const std::string& goal) const override;

  std::string _name_;

 private:
  // Additional type for coverage planner
  typedef std::vector<std::vector<iroc_mission_handler::Waypoint>> coverage_paths_t;

  bool is_initialized_ = false;
  bool is_active_ = false;

  mutable algorithm_config_t planner_config_;
  std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers_;

  algorithm_config_t parse_algorithm_config(mrs_lib::ParamLoader& param_loader) const;

  coverage_paths_t getCoveragePaths(const iroc_fleet_manager::CoverageMission& mission) const;
};

bool CoveragePlanner::initialize(const ros::NodeHandle& parent_nh, const std::string& name, const std::string& name_space,
                                 std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers) {

  // nh_ will behave just like normal NodeHandle
  ros::NodeHandle nh_(parent_nh, name_space);

  _name_ = name;
  common_handlers_ = common_handlers;
  ros::Time::waitForValid();

  /* load parameters */
  mrs_lib::ParamLoader param_loader(nh_, "CoveragePlanner");

  param_loader.addYamlFile(ros::package::getPath("iroc_fleet_manager") + "/config/coverage_config.yaml");

  planner_config_ = parse_algorithm_config(param_loader);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: could not load all parameters!", _name_.c_str());
    is_initialized_ = false;
    return true;
  }

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[%s]: initialized under the name '%s', namespace '%s' and action ", _name_.c_str(), name.c_str(), name_space.c_str());

  is_initialized_ = true;
  return true;
}

bool CoveragePlanner::activate(void) {

  int some_number = 0;
  ROS_INFO("[%s]: activated with some_number=%d", _name_.c_str(), some_number);

  is_active_ = true;

  return true;
}

void CoveragePlanner::deactivate(void) {

  is_active_ = false;

  ROS_INFO("[%s]: deactivated", _name_.c_str());
}

std::tuple<result_t, std::vector<iroc_mission_handler::MissionGoal>> CoveragePlanner::createGoal(const std::string& goal) const {
  // Goal to be filled
  std::vector<iroc_mission_handler::MissionGoal> mission_robots;
  ROS_INFO("[CoveragePlanner] Received goal :%s ",
           goal.c_str()); // to remove

  // Custom messages used in the coverage planner
  std::vector<iroc_fleet_manager::CoverageMissionRobot> robots_msg;
  std::vector<mrs_msgs::Point2D> search_area_msg;
  mrs_msgs::Point2D latlon_origin_msg;

  result_t result;
  json json_msg;

  // Parsing JSON and creating robots JSON for post processing
  result = parseJson(goal, json_msg);

  if (!result.success) {
    return std::make_tuple(result, mission_robots);
  }

  std::vector<custom_types::Point2D> search_area;
  json robots;
  int frame_id;
  int height;
  int height_id;
  int terminal_action;

  bool success = parseVars(json_msg, {
                                         {"search_area", &search_area},
                                         {"robots", &robots},
                                         {"frame_id", &frame_id},
                                         {"height", &height},
                                         {"height_id", &height_id},
                                         {"terminal_action", &terminal_action},
                                     });

  search_area_msg = toRosMsg<mrs_msgs::Point2D>(search_area);

  // Extract robots
  robots_msg.reserve(robots.size());
  for (const auto& robot : robots) {
    iroc_fleet_manager::CoverageMissionRobot robot_msg;
    std::string name;

    auto succ = parseVars(robot, {{"name", &name}});

    if (!result.success) {
      return std::make_tuple(result, mission_robots);
    }

    robot_msg.name = name;
    robot_msg.frame_id = frame_id;
    robot_msg.height_id = height_id;
    robot_msg.height = height;
    robot_msg.terminal_action = terminal_action;
    auto global_pose = common_handlers_->handlers->robots_map[name].state_estimation_info->global_pose.position;
    auto local_pose = common_handlers_->handlers->robots_map[name].state_estimation_info->local_pose.position;

    robot_msg.global_position = global_pose;
    robot_msg.local_position = local_pose;
    robots_msg.push_back(robot_msg);
  }

  // Extracting the latlon origin
  // For simplicity taking the first origin, but we could also validate if all
  // of the origins are consistent

  latlon_origin_msg.x = common_handlers_->handlers->robots_map[robots_msg.at(0).name].safety_area_info->safety_area.origin_x;
  latlon_origin_msg.y = common_handlers_->handlers->robots_map[robots_msg.at(0).name].safety_area_info->safety_area.origin_y;

  iroc_fleet_manager::CoverageMission mission;
  mission.robots = robots_msg;
  mission.search_area = search_area_msg;
  mission.latlon_origin = latlon_origin_msg;

  auto paths = getCoveragePaths(mission);

  // Filling the mission_robots vector with the generated paths
  for (int it = 0; it < mission.robots.size(); it++) {
    iroc_mission_handler::MissionGoal robot;
    robot.name = mission.robots[it].name;
    robot.points = paths[it];
    robot.terminal_action = mission.robots[it].terminal_action;
    robot.height_id = mission.robots[it].height_id;
    robot.frame_id = mission.robots[it].frame_id;
    mission_robots.push_back(robot);
  }

  ROS_INFO("[CoveragePlanner] Goal created successfully!");
  result.success = true;
  result.message = "Goal created successfully";
  return std::make_tuple(result, mission_robots);
}

algorithm_config_t CoveragePlanner::parse_algorithm_config(mrs_lib::ParamLoader& param_loader) const {
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

point_t calculate_centroid(const MapPolygon& polygon) {
  auto points = polygon.get_all_points();
  double sum_x = 0, sum_y = 0;
  for (const auto& point : points) {
    sum_x += point.first;
    sum_y += point.second;
  }
  return {sum_x / points.size(), sum_y / points.size()};
}

double calculate_distance(const point_t& p1, const point_t& p2) {
  return std::sqrt(std::pow(p1.first - p2.first, 2) + std::pow(p1.second - p2.second, 2));
}

std::vector<MapPolygon> assign_closest_polygons(const std::vector<point_t>& uav_positions, std::vector<MapPolygon> polygons) {
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

CoveragePlanner::coverage_paths_t CoveragePlanner::getCoveragePaths(const iroc_fleet_manager::CoverageMission& mission) const {

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
  ROS_INFO_STREAM("[CoveragePlanner:]: Energy calculator created. Optimal speed: " << energy_calculator.get_optimal_speed());

  // Decompose polygon for each UAV
  auto decomposed_polygon = decompose_polygon(mission.robots.size(), planner_config_, polygon);

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

  ROS_INFO("[CoveragePlanner:]: Size of decomposed polygon: %zu", decomposed_polygon.size());

  // For saving the paths for each UAV
  coverage_paths_t coverage_paths;
  // For accessing the robots information from mission goal
  int uav_index = 0;
  for (const auto& polygon : decomposed_polygon) {
    mstsp_solver::final_solution_t best_solution;
    try {

      const auto& assigned_polygon = assigned_polygons[uav_index];
      // Setting the start position for each UAV
      planner_config_.start_pos.first = mission.robots.at(uav_index).global_position.x;
      planner_config_.start_pos.second = mission.robots.at(uav_index).global_position.y;
      auto f = [&](int n) { return solve_for_uavs(n, planner_config_, assigned_polygon, energy_calculator, shortest_path_calculator, shared_logger); };
      best_solution = generate_with_constraints(planner_config_.max_single_path_energy * 3600, 1, f);
      uav_index++;
    } catch (const polygon_decomposition_error& e) {
      ROS_WARN_STREAM("[CoveragePlanner:]: Error while decomposing the polygon");
      return coverage_paths_t();
    }

    auto best_paths = best_solution.paths;
    for (auto& path : best_paths) {
      std::vector<iroc_mission_handler::Waypoint> coverage_path;
      mrs_msgs::Reference point;
      for (auto& p : path) {
        // TODO Replace with mrs_lib transformer?
        auto lat_lon_p = meters_to_gps_coordinates({p.x, p.y}, planner_config_.lat_lon_origin);
        p.x = lat_lon_p.first;
        p.y = lat_lon_p.second;
        // Fill the reference point
        point.position.x = p.x;
        point.position.y = p.y;
        point.position.z = mission.robots[0].height; // We are using same height for all robots
        point.heading = 0.0;

        iroc_mission_handler::Waypoint waypoint;
        waypoint.reference = point;
        coverage_path.push_back(waypoint);
      }
      coverage_paths.push_back(coverage_path);
    }
  }

  return coverage_paths;
}
} // namespace coverage_planner

} // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::coverage_planner::CoveragePlanner, iroc_fleet_manager::Planner);
