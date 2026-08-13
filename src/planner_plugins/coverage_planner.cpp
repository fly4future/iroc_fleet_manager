#include <iroc_fleet_manager/iroc_plugins/coverage_planner.h>
#include <iroc_fleet_manager/utils/json_var_parser.h>
#include <iroc_fleet_manager/utils/ros_helpers.h>
#include <mrs_msgs/msg/reference.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace iroc_fleet_manager
{

namespace planners
{

namespace coverage_planner
{

bool CoveragePlanner::initialize(const rclcpp::Node::SharedPtr node, const std::string &name, const std::string &name_space,
                                 std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers) {
  node_  = node;
  clock_ = node->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  name_            = name;
  common_handlers_ = common_handlers;

  std::string package_path = ament_index_cpp::get_package_share_directory("iroc_fleet_manager");
  YAML::Node algorithm_config_node = YAML::LoadFile(package_path + "/config/coverage_planner_config.yaml");
  if (!algorithm_config_is_valid(algorithm_config_node)) {
    RCLCPP_ERROR(node_->get_logger(), "Algorithm config is not complete. Exiting...");
    return false;
  }

  /* load parameters */
  mrs_lib::ParamLoader param_loader(node_, "CoveragePlanner");

  param_loader.addYamlFile(package_path + "/config/coverage_planner_config.yaml");

  planner_config_ = parse_algorithm_config(param_loader);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    is_initialized_ = false;
    return true;
  }

  // | ----------------------- finish init ---------------------- |

  RCLCPP_INFO(node_->get_logger(), "[%s]: initialized under the name '%s', namespace '%s'", name_.c_str(), name.c_str(), name_space.c_str());

  is_initialized_ = true;
  return true;
}

bool CoveragePlanner::activate(void) {

  RCLCPP_INFO(node_->get_logger(), "[%s]: activated", name_.c_str());

  is_active_ = true;

  return true;
}

void CoveragePlanner::deactivate(void) {

  is_active_ = false;

  RCLCPP_INFO(node_->get_logger(), "[%s]: deactivated", name_.c_str());
}

std::tuple<result_t, std::vector<iroc_mission_handler::msg::MissionGoal>> CoveragePlanner::createGoal(const std::string &goal) const {
  // Goal to be filled
  std::vector<iroc_mission_handler::msg::MissionGoal> mission_robots;
  RCLCPP_INFO(node_->get_logger(), "[%s]: creating goal from the received request", name_.c_str());

  // Custom messages used in the coverage planner
  std::vector<iroc_fleet_manager::msg::CoverageMissionRobot> robots_msg;
  mrs_msgs::msg::Point2D                                     latlon_origin_msg;

  result_t result;
  json     json_msg;

  // Parsing JSON and creating robots JSON for post processing
  result = parseJson(goal, json_msg);

  if (!result.success) {
    result.success = false;
    result.message = "Faile to parse JSON msg";
    return std::make_tuple(result, mission_robots);
  }

  RCLCPP_INFO(node_->get_logger(), "[%s]: received goal: %s", name_.c_str(), goal.c_str());

  using HRNoFlyZone = std::pair<std::vector<iroc_fleet_manager::custom_types::Point2DLatLon>, double>;
  std::vector<std::vector<iroc_fleet_manager::custom_types::Point2DLatLon>> search_areas;
  std::vector<std::vector<iroc_fleet_manager::custom_types::Point2DLatLon>> no_fly_zones;
  std::vector<HRNoFlyZone> hr_no_fly_zones;
  std::vector<double> min_horizontal_distances;
  std::vector<double> min_vertical_distances;
  json robots;
  // int frame_id;
  int height;
  int height_id;
  int terminal_action;

  bool success = iroc_fleet_manager::utils::parseVars(json_msg, {
                                                {"search_areas", &search_areas},
                                                {"min_horizontal_distances", &min_horizontal_distances},
                                                {"min_vertical_distances", &min_vertical_distances},
                                                {"robots", &robots},
                                                {"height", &height},
                                                {"height_id", &height_id},
                                                {"terminal_action", &terminal_action}
                                            });
  if (!success) {
    RCLCPP_ERROR(node_->get_logger(), "Failure while parsing robot data, bad JSON request");
    result.success = false;
    result.message = "Failure while parsing robot data, bad JSON request";
    return std::make_tuple(result, mission_robots);
  }

  // Check if not empty
  if (robots.empty()) {
    result.success = false;
    result.message = "Received empty robots list, aborting mission.";
    RCLCPP_WARN(node_->get_logger(), " Received empty robots list, aborting mission.");
    return std::make_tuple(result, mission_robots);
  }

  if (search_areas.empty()) {
    result.success = false;
    result.message = "Received empty search area, aborting mission.";
    RCLCPP_WARN(node_->get_logger(), " Received empty search area, aborting mission.");
    return std::make_tuple(result, mission_robots);
  }

  if (robots.size() != min_horizontal_distances.size() || robots.size() != min_vertical_distances.size()) {
    RCLCPP_ERROR(node_->get_logger(), "The number of values in 'robots' differs from 'min_horizontal_distances' or 'min_vertical_distances'. Each robot should have its own specified minimum horizontal and vertical distances from other robots.");
    result.success = false;
    result.message = "The number of values in 'robots' differs from 'min_horizontal_distances' or 'min_vertical_distances'. Each robot should have its own specified minimum horizontal and vertical distances from other robots.";
    return std::make_tuple(result, mission_robots);
  }

  if (robots.size() != planner_config_.drones.size()) {
    RCLCPP_ERROR(node_->get_logger(), "The number of drones in the mission JSON (coverage.json) does not match the number of drone definitions in the coverage planner config file (coverage_planner_config.yaml).");
    result.success = false;
    result.message = "The number of drones in the mission JSON does not match the number of drone definitions in the coverage planner configuration.";
    return std::make_tuple(result, mission_robots);
  }

  // parsing optional parameters
  success = iroc_fleet_manager::utils::parseVars(json_msg, {{"no_fly_zones", &no_fly_zones}});
  success = iroc_fleet_manager::utils::parseVars(json_msg, {{"hr_no_fly_zones", &hr_no_fly_zones}});
  
  // Extract robots
  robots_msg.reserve(robots.size());
  for (const auto &robot : robots) {
    iroc_fleet_manager::msg::CoverageMissionRobot robot_msg;
    std::string                                   name;

    name = robot.get<std::string>();

    bool isRobotInFleet = common_handlers_->handlers->robots_map.count(name);

    if (!isRobotInFleet) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Robot " << name << " not within the fleet");
      std::stringstream ss;
      ss << name << " not found in the fleet!";
      result.message = ss.str();
      result.success = false;
      return std::make_tuple(result, mission_robots);
    }

    robot_msg.name            = name;
    robot_msg.frame_id        = iroc_mission_handler::msg::MissionGoal::FRAME_ID_LATLON;
    robot_msg.height_id       = height_id;
    robot_msg.height          = height;
    robot_msg.terminal_action = terminal_action;
    auto global_pose          = common_handlers_->handlers->robots_map[name].state_estimation_info->global_pose.position;
    auto local_pose           = common_handlers_->handlers->robots_map[name].state_estimation_info->local_pose.position;

    robot_msg.global_position = global_pose;
    robot_msg.local_position  = local_pose;
    robots_msg.push_back(robot_msg);
  }

  // Extracting the latlon origin
  // For simplicity taking the first origin, but we could also validate if all
  // of the origins are consistent

  latlon_origin_msg.x = common_handlers_->handlers->robots_map[robots_msg.at(0).name].safety_area_info->world_origin.x;
  latlon_origin_msg.y = common_handlers_->handlers->robots_map[robots_msg.at(0).name].safety_area_info->world_origin.y;

  iroc_fleet_manager::msg::CoverageMission mission;
  mission.robots        = robots_msg;
  mission.latlon_origin = latlon_origin_msg;

  auto paths = getCoveragePaths(mission, search_areas, no_fly_zones, hr_no_fly_zones, min_horizontal_distances, min_vertical_distances);

  if (paths.empty()) {
    result.success = false;
    result.message = "Coverage path planning failed.";
    RCLCPP_ERROR(node_->get_logger(), "Coverage path planning failed, aborting mission creation.");
    return std::make_tuple(result, mission_robots);
  }

  // If the trajectory was optimized by time, set the speed and acceleration parameters from config file to drones
  if (!planner_config_.drones.empty() && planner_config_.drones.at(0).optimization_type == "time") {
    for (size_t i = 0; i < mission.robots.size(); ++i) {
      const auto &robot = mission.robots.at(i);
      const auto &time_cfg = planner_config_.drones.at(i).time_config.value();

      // Try to switch the UAV to the 'medium' profile (best-effort).
      if (!iroc_fleet_manager::utils::switchProfile(node_, robot.name, "medium")) {
        RCLCPP_WARN(node_->get_logger(),
                    "Switch to profile 'medium' failed for drone %s; continuing with current constraints.",
                    robot.name.c_str());
      }

      const bool ok = iroc_fleet_manager::utils::setCustomValuesForMedium(
          node_,
          robot.name,
          time_cfg.max_horizontal_speed,
          time_cfg.horizontal_acceleration,
          time_cfg.max_vertical_speed,
          time_cfg.vertical_acceleration,
          time_cfg.max_vertical_speed,
          time_cfg.vertical_acceleration);

      if (!ok) {
        RCLCPP_WARN(node_->get_logger(),
                    "Failed to set time-optimized constraints for drone %s.",
                    robot.name.c_str());
      } else {
        RCLCPP_INFO(node_->get_logger(),
                    "Time-optimized constraints set for drone %s.",
                    robot.name.c_str());
      }
    }
  }

  // Filling the mission_robots vector with the generated paths
  for (size_t it = 0; it < mission.robots.size(); it++) {
    iroc_mission_handler::msg::MissionGoal robot;
    robot.name            = mission.robots[it].name;
    robot.points          = paths[it];
    robot.terminal_action = mission.robots[it].terminal_action;
    robot.height_id       = mission.robots[it].height_id;
    robot.frame_id        = mission.robots[it].frame_id;
    mission_robots.push_back(robot);
  }

  RCLCPP_INFO(node_->get_logger(), "[%s]: Goal created successfully!", name_.c_str());
  result.success = true;
  result.message = "Goal created successfully";
  return std::make_tuple(result, mission_robots);
}

bool CoveragePlanner::algorithm_config_is_valid(const YAML::Node &root_node) {
    // Navigate to the 'coverage_planner' sub-node
    if (!root_node["fleet_manager"] || !root_node["fleet_manager"]["planners"] || !root_node["fleet_manager"]["planners"]["coverage_planner"]) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "The configuration file must contain the path 'fleet_manager.planners.coverage_planner'.");
      return false;
    }
    const YAML::Node& config = root_node["fleet_manager"]["planners"]["coverage_planner"];

    // Helper lambda for checking if a key exists
    auto check_key = [&](const YAML::Node& node, const std::string& key) {
      if (!node[key]) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Missing required key '" << key << "' in the config file.");
        return false;
      }
      return true;
    };

    // --- Global mission parameters ---
    const std::vector<std::string> global_keys = {
        "number_of_rotations", "points_in_lat_lon",
        "sweeping_step", "decomposition_method", "min_sub_polygons_per_uav",
        "rotations_per_cell", "no_improvement_cycles_before_stop", "drones"
    };

    for (const auto& key : global_keys) {
        if (!check_key(config, key)) return false;
    }

    if (config["points_in_lat_lon"].as<bool>()) {
        if (!check_key(config, "latitude_origin") || !check_key(config, "longitude_origin")) {
          RCLCPP_ERROR_STREAM(node_->get_logger(), "'latitude_origin' and 'longitude_origin' are required when 'points_in_lat_lon' is true.");
          return false;
        }
    }

    // --- Parameters for individual drones ---
    if (!config["drones"] || !config["drones"].IsSequence() || config["drones"].size() == 0) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "'drones' array is missing, not a sequence, or is empty in the config file.");
      return false;
    }

    const std::string first_optimization_type = config["drones"][0]["optimization_type"].as<std::string>();

    for (const auto& drone_node : config["drones"]) {
        if (!check_key(drone_node, "optimization_type") || !check_key(drone_node, "max_single_path_cost")) {
          RCLCPP_ERROR_STREAM(node_->get_logger(), "Each drone in 'drones' must have 'optimization_type' and 'max_single_path_cost'.");
          return false;
        }

        const std::string current_optimization_type = drone_node["optimization_type"].as<std::string>();
        if (current_optimization_type != first_optimization_type) {
          RCLCPP_ERROR_STREAM(node_->get_logger(), "All drones must have the same 'optimization_type'. Found '" << current_optimization_type
                   << "' which is different from the first drone's type '" << first_optimization_type << "'.");
          return false;
        }


        if (current_optimization_type == "energy") {
            const std::vector<std::string> energy_keys = {
                "drone_mass", "drone_area", "average_acceleration", "propeller_radius",
                "number_of_propellers", "allowed_path_deviation", "battery_model", "best_speed_model"
            };
            for (const auto& key : energy_keys) {
                if (!check_key(drone_node, key)) {
                  RCLCPP_ERROR_STREAM(node_->get_logger(), "Drone with 'energy' optimization is missing key '" << key << "'.");
                  return false;
                }
            }
            if (!check_key(drone_node["battery_model"], "cell_capacity") || !check_key(drone_node["battery_model"], "number_of_cells") ||
            !check_key(drone_node["battery_model"], "d0") || !check_key(drone_node["battery_model"], "d1") ||
            !check_key(drone_node["battery_model"], "d2") || !check_key(drone_node["battery_model"], "d3")) {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "'battery_model' node is missing one or more required parameters (cell_capacity, number_of_cells, d0-d3).");
            return false;
          }
            if (!check_key(drone_node["best_speed_model"], "c0") || !check_key(drone_node["best_speed_model"], "c1") ||
              !check_key(drone_node["best_speed_model"], "c2")) {
              RCLCPP_ERROR_STREAM(node_->get_logger(), "'best_speed_model' node is missing one or more required parameters (c0-c2).");
              return false;
            }

            // Check the global physical parameters required for the energy model
            if (!check_key(config, "air_density") || !check_key(config, "earth_gravity") || !check_key(config, "propeller_efficiency")) {
                 return false;
            }

        } else if (current_optimization_type == "time") {
            const std::vector<std::string> time_keys = {
                "max_horizontal_speed", "max_vertical_speed", "horizontal_acceleration",
                "vertical_acceleration", "allowed_path_deviation"
            };
            for (const auto& key : time_keys) {
                if (!check_key(drone_node, key)) {
                  RCLCPP_ERROR_STREAM(node_->get_logger(), "Drone with 'time' optimization is missing key '" << key << "'.");
                  return false;
                }
            }
        } else {
          RCLCPP_ERROR_STREAM(node_->get_logger(), "Unknown 'optimization_type': " << current_optimization_type << ". Use 'energy' or 'time'.");
          return false;
        }
    }

    return true;
}

algorithm_config_t CoveragePlanner::parse_algorithm_config(mrs_lib::ParamLoader &param_loader) const {
  const std::string  yaml_prefix = "fleet_manager/planners/coverage_planner/";
  algorithm_config_t algorithm_config;

  double air_density = -1;
  double earth_gravity = -1;
  double propeller_efficiency = -1;

  std::string package_path = ament_index_cpp::get_package_share_directory("iroc_fleet_manager");
  YAML::Node algorithm_config_node = YAML::LoadFile(package_path + "/config/coverage_planner_config.yaml");
  YAML::Node drones_node = algorithm_config_node["fleet_manager"]["planners"]["coverage_planner"]["drones"];

  algorithm_config.drones.reserve(drones_node.size());

  algorithm_config.number_of_drones = drones_node.size();

  for (size_t i = 0; i < drones_node.size(); ++i) {
    YAML::Node drone = drones_node[i];
    // DroneConfig drone_config;
    // if (i == 0) optimization_type = drone["optimization_type"];
    drone_spec_t drone_config;
    drone_config.optimization_type = drone["optimization_type"].as<std::string>();

    if (drone["optimization_type"].as<std::string>() == "energy") {
      energy_calculator_config_t energy_params;

      energy_params.drone_mass = drone["drone_mass"].as<double>();
      energy_params.drone_area = drone["drone_area"].as<double>();
      energy_params.propeller_radius = drone["propeller_radius"].as<double>();
      energy_params.number_of_propellers = drone["number_of_propellers"].as<int>();

      energy_params.average_acceleration = drone["average_acceleration"].as<double>();
      energy_params.allowed_path_deviation = drone["allowed_path_deviation"].as<double>();

      YAML::Node battery = drone["battery_model"];
      energy_params.battery_model.cell_capacity = battery["cell_capacity"].as<double>();
      energy_params.battery_model.number_of_cells = battery["number_of_cells"].as<int>();
      energy_params.battery_model.d0 = battery["d0"].as<double>();
      energy_params.battery_model.d1 = battery["d1"].as<double>();
      energy_params.battery_model.d2 = battery["d2"].as<double>();
      energy_params.battery_model.d3 = battery["d3"].as<double>();

      YAML::Node speed = drone["best_speed_model"];
      energy_params.best_speed_model.c0 = speed["c0"].as<double>();
      energy_params.best_speed_model.c1 = speed["c1"].as<double>();
      energy_params.best_speed_model.c2 = speed["c2"].as<double>();

      if (air_density == -1 && earth_gravity == -1 && propeller_efficiency == -1) {
        param_loader.loadParam(yaml_prefix + "air_density", air_density);
        param_loader.loadParam(yaml_prefix + "earth_gravity", earth_gravity);
        param_loader.loadParam(yaml_prefix + "propeller_efficiency", propeller_efficiency);
      }
      energy_params.air_density = air_density;
      energy_params.earth_gravity = earth_gravity;
      energy_params.propeller_efficiency = propeller_efficiency;
      
      drone_config.energy_config = energy_params;

    } else if (drone["optimization_type"].as<std::string>() == "time") {
      time_calculator_config_t time_params;
      time_params.max_horizontal_speed = drone["max_horizontal_speed"].as<double>();
      time_params.max_vertical_speed = drone["max_vertical_speed"].as<double>();
      time_params.horizontal_acceleration = drone["horizontal_acceleration"].as<double>();
      time_params.vertical_acceleration = drone["vertical_acceleration"].as<double>();
      time_params.allowed_path_deviation = drone["allowed_path_deviation"].as<double>();
      drone_config.time_config = time_params;
    }

    drone_config.max_single_path_cost = drone["max_single_path_cost"].as<double>();

    algorithm_config.drones.push_back(drone_config);
  }
 
  param_loader.loadParam(yaml_prefix + "number_of_rotations", algorithm_config.number_of_rotations);

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

  return algorithm_config;
}

struct TransitPath
{
  double x1, y1, x2, y2;
  
  TransitPath(double x1, double y1, double x2, double y2) : x1(x1), y1(y1), x2(x2), y2(y2) {}
};

class TransitPathGroup
{
private:
  std::vector<std::unique_ptr<TransitPath>> transit_path_group;
  std::vector<double*> z_ptrs;
  
public:
  int drone_idx;

  double min_horizontal_distance;
  double min_vertical_distance;
  double drone_height;  // this variable has value of sweeping height if TPG is not going above height restricted no-fly-zone. If it is going above hr no-fly zone, then it has value of the height the drone needs to travel

  TransitPathGroup(int drone_idx, double min_horizontal_distance, double min_vertical_distance) : drone_idx(drone_idx), min_horizontal_distance(min_horizontal_distance), min_vertical_distance(min_vertical_distance) {
  }

  // transit_path_group should be read only for user
  const std::vector<std::unique_ptr<TransitPath>>& get() const {
    return transit_path_group;
  }

  void setHeight(double height, double sweeping_height, double transit_height) {
    if (height == transit_height) {
      drone_height = sweeping_height;
    } else {
      drone_height = height + min_vertical_distance;
    }
  }

  void writeTransitPathHeights(double height) {
    for (double* &z_ptr : z_ptrs) {
      if (z_ptr) *(z_ptr) = height;
    }
  }

  void addTransitPath(double x1, double y1, double x2, double y2, double *z1, double *z2) {
    std::unique_ptr<TransitPath> tp(new TransitPath(x1, y1, x2, y2));
    transit_path_group.push_back(std::move(tp));
    for (double* z_ptr : {z1, z2}) {
      if (count(z_ptrs.begin(), z_ptrs.end(), z_ptr) == 0) {
        z_ptrs.push_back(z_ptr);
      }
    }
  }
};

struct TransitPathGroupsStruct
{
  std::vector<std::unique_ptr<TransitPathGroup>> transit_path_groups;
  // Stores the information of which TransiPathGroups should be under which TransitPathGroup
  std::vector<std::vector<int>> transit_paths_under;
};



// Graph stores the information of transit paths overlaping. It is used in resolveTransitHeights function.
struct Graph {
    int V; // number of vertexes
    std::vector<std::vector<int>> adj; // List of neighbours

    Graph(int V) : V(V), adj(V) {}

    void addEdge(int u, int v) {
        adj[u].push_back(v);
        adj[v].push_back(u);
    }
};

// Auxiliary structure for deciding which vertex in Graph has more priority
struct NodePriority {
    int id;
    int degree;
    double best_available_height;

    // Logic of deciding which vertex has more priority:
    // 1. Higher degree (number of overlaps) has more priority
    // 2. If degrees are equal, more priority has a vertex which can be moved to lower level 
    bool operator>(const NodePriority& other) const {
        if (degree != other.degree) {
            return degree > other.degree;
        }
        return best_available_height < other.best_available_height;
    }
};

std::vector<int> hungarianAlgorithm(const std::vector<std::vector<double>>& matrix);
bool pointCloseToLineSegment(const point_t& point, const TransitPath& path, double min_dist);
bool segmentsIntersect(TransitPath tp1, TransitPath tp2);
int horizontalAndVerticalTPGIntersection(TransitPathGroup &tpg1, TransitPathGroup &tpg2, double drone_distance);
bool checkForPotentialCycle(std::vector<std::vector<int>> &transit_paths_under, int starting_idx, int search_idx);
bool checkOverlap2(TransitPathGroup &tpg1, TransitPathGroup &tpg2, double min_distance);
bool checkOverlap(TransitPath tp1, TransitPath tp2, double min_distance);
double pointToSegmentDistance(custom_types::Point2D p, custom_types::Point2D s1, custom_types::Point2D s2);
void resolveTransitHeights(TransitPathGroupsStruct& tpgs, CoveragePlanner::coverage_paths_t& coverage_paths, const Graph& graph, double sweeping_height, std::vector<double> min_horizontal_distances, std::vector<double> min_vertical_distances);
std::vector<iroc_mission_handler::msg::Waypoint> pointVecToWaypointVec(std::vector<point_t> &points, double transit_path_height);
std::vector<point_heading_t<double>> convertWaypointsToPointHeading(const std::vector<iroc_mission_handler::msg::Waypoint>& iroc_waypoints);


// Calculates the distance between a drone position and start and end of sweeping trajectory
double droneToSweepingDistance(point_t drone_pos, point_t start, point_t end, ShortestPathCalculator shortest_path_calculator)
{
  double distance = 0;
  std::vector<point_t> path_to_start = shortest_path_calculator.shortest_path_between_points({drone_pos.first, drone_pos.second}, {start.first, start.second}).first;
  for (unsigned int i = 1; i < path_to_start.size(); i++) {
    distance += std::sqrt(pow(path_to_start.at(i-1).first - path_to_start.at(i).first, 2) + pow(path_to_start.at(i-1).second - path_to_start.at(i).second, 2));
  }
  std::vector<point_t> path_to_end = shortest_path_calculator.shortest_path_between_points({drone_pos.first, drone_pos.second}, {end.first, end.second}).first;
  for (unsigned int i = 1; i < path_to_end.size(); i++) {
    distance += std::sqrt(pow(path_to_end.at(i-1).first - path_to_end.at(i).first, 2) + pow(path_to_end.at(i-1).second - path_to_end.at(i).second, 2));
  }
  return distance;
}

// Check if a point is inside a polygon using the ray casting algorithm
bool is_inside(const point_t& p, const std::vector<point_t>& polygon) {
    if (polygon.empty()) return false;
    int intersections = 0;
    for (size_t i = 0; i < polygon.size() - 1; ++i) {
        const auto& p1 = polygon[i];
        const auto& p2 = polygon[i+1];

        if (p.second > std::min(p1.second, p2.second) &&
            p.second <= std::max(p1.second, p2.second) &&
            p.first <= std::max(p1.first, p2.first) &&
            p1.second != p2.second) {
            double x_intersection = (p.second - p1.second) * (p2.first - p1.first) / (p2.second - p1.second) + p1.first;
            if (p1.first == p2.first || p.first <= x_intersection) {
                intersections++;
            }
        }
    }
    return (intersections % 2) == 1;
}

CoveragePlanner::coverage_paths_t CoveragePlanner::getCoveragePaths(const iroc_fleet_manager::msg::CoverageMission &mission, const std::vector<std::vector<custom_types::Point2DLatLon>> &search_areas_arg, const std::vector<std::vector<custom_types::Point2DLatLon>> &no_fly_zones_arg, const std::vector<std::pair<std::vector<custom_types::Point2DLatLon>, double>> &hr_no_fly_zones_arg, std::vector<double> min_horizontal_distances, std::vector<double> min_vertical_distances) const {

  std::vector<polygon_t> fly_zones;
  std::vector<polygon_t> no_fly_zones;
  std::vector<std::pair<polygon_t, double>> hr_no_fly_zones;

  // Fill the fly zones
  fly_zones.reserve(search_areas_arg.size());
  for (const auto &search_area : search_areas_arg) {
    std::vector<point_t> fly_zone;
    for (const auto &point : search_area) {
      fly_zone.emplace_back(point.lat, point.lon);
    }
    // Add the first point to close the polygon
    if (!fly_zone.empty()) {
      fly_zone.emplace_back(search_area[0].lat, search_area[0].lon);
    }
    fly_zones.push_back(fly_zone);
  }

  // Fill the no-fly zones
  no_fly_zones.reserve(no_fly_zones_arg.size());
  for (const auto &zone : no_fly_zones_arg) {
    std::vector<point_t> no_fly_zone;
    for (const auto &point : zone) {
      no_fly_zone.emplace_back(point.lat, point.lon);
    }
    // Add the first point to close the polygon
    if (!no_fly_zone.empty()) {
      no_fly_zone.emplace_back(zone[0].lat, zone[0].lon);
    }
    no_fly_zones.push_back(no_fly_zone);
  }

  // Fill the height restricted no-fly zones
  hr_no_fly_zones.reserve(hr_no_fly_zones_arg.size());
  for (const auto &zone : hr_no_fly_zones_arg) {
    polygon_t hr_no_fly_zone;
    hr_no_fly_zone.reserve(zone.first.size() + 1);
    for (const auto &point : zone.first) {
      hr_no_fly_zone.emplace_back(point.lat, point.lon);
    }
    // Add the first point to close the polygon
    if (!hr_no_fly_zone.empty()) {
      hr_no_fly_zone.emplace_back(zone.first[0].lat, zone.first[0].lon);
    }
    hr_no_fly_zones.emplace_back(std::move(hr_no_fly_zone), zone.second);
  }

  // Validate: all zones must be either fully disjoint or fully contained.
  coverage_paths_t coverage_paths_empty;
  std::vector<polygon_t> all_polygons;
  for (const auto &fz : fly_zones) all_polygons.push_back(fz);
  for (const auto &nfz : no_fly_zones) all_polygons.push_back(nfz);
  for (const auto &hr : hr_no_fly_zones) all_polygons.push_back(hr.first);

  for (size_t i = 0; i < all_polygons.size(); ++i) {
    for (size_t j = i + 1; j < all_polygons.size(); ++j) {
      bool overlap = false;

      int inside_i_j = 0;
      for (size_t a = 0; a + 1 < all_polygons[i].size(); ++a) {
        if (is_inside(all_polygons[i][a], all_polygons[j])) inside_i_j++;
      }

      int inside_j_i = 0;
      for (size_t b = 0; b + 1 < all_polygons[j].size(); ++b) {
        if (is_inside(all_polygons[j][b], all_polygons[i])) inside_j_i++;
      }

      // If some but not all vertices are inside, this is a partial overlap.
      if ((inside_i_j > 0 && inside_i_j < static_cast<int>(all_polygons[i].size() - 1)) ||
          (inside_j_i > 0 && inside_j_i < static_cast<int>(all_polygons[j].size() - 1))) {
        overlap = true;
      }

      // edge intersection check (covers crossings without vertex containment)
      for (size_t a = 0; a + 1 < all_polygons[i].size() && !overlap; ++a) {
        TransitPath s1(all_polygons[i][a].first, all_polygons[i][a].second, all_polygons[i][a+1].first, all_polygons[i][a+1].second);
        for (size_t b = 0; b + 1 < all_polygons[j].size() && !overlap; ++b) {
          TransitPath s2(all_polygons[j][b].first, all_polygons[j][b].second, all_polygons[j][b+1].first, all_polygons[j][b+1].second);
          if (segmentsIntersect(s1, s2)) overlap = true;
        }
      }

      if (overlap) {
        RCLCPP_ERROR(node_->get_logger(), "Polygon edges of Fly zones, No-fly zones and Height restricted no-fly zones must not overlap.");
        return coverage_paths_empty;
      }
    }
  }

  double transit_path_height = mission.robots[0].height + 1.0;
  double sweeping_height = mission.robots[0].height;

  planner_config_.lat_lon_origin.first  = mission.latlon_origin.x;
  planner_config_.lat_lon_origin.second = mission.latlon_origin.y;
  planner_config_.number_of_drones = mission.robots.size();
  planner_config_.sweeping_alt = sweeping_height;

  // Create a logger to log everything directly into stdout
  auto shared_logger = std::make_shared<loggers::SimpleLogger>();

  std::vector<std::shared_ptr<PathCostCalculator>> cost_calculators;
  for (const auto& spec : planner_config_.drones) {
    if (spec.optimization_type == "energy" && spec.energy_config.has_value()) {
      cost_calculators.push_back(std::make_shared<EnergyCalculator>(spec.energy_config.value(), shared_logger));
    } else if (spec.optimization_type == "time" && spec.time_config.has_value()) {
      cost_calculators.push_back(std::make_shared<TimeCalculator>(spec.time_config.value()));
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Unknown drone specification for optimization_type: %s", spec.optimization_type.c_str());
      coverage_paths_t empty_path;
      return empty_path;
    }
  }


  // Create one master polygon that contains ALL obstacles. This will be used for pathfinding between areas.
  // The fly-zone part is left empty, as the ShortestPathCalculator will ignore it anyway.
  MapPolygon master_obstacle_polygon;
  polygon_t empty_fly_zone;
  master_obstacle_polygon = MapPolygon(empty_fly_zone, no_fly_zones, planner_config_.lat_lon_origin, hr_no_fly_zones);

  ShortestPathCalculator shortest_path_calculator(master_obstacle_polygon, true, sweeping_height);

  // Now, create a vector of MapPolygon objects, one for each search area.
  // Each of these will contain its own fly zone boundary, but also ALL no-fly zones.
  // The trapezoidal decomposition will correctly handle only the NFZs inside the FZ.
  std::vector<MapPolygon> search_areas;
  
  for (polygon_t fly_zone : fly_zones) {

    MapPolygon area;
    area = MapPolygon(fly_zone, no_fly_zones, planner_config_.lat_lon_origin, hr_no_fly_zones);    

    // Remove outer no-fly zones only for decomposition and sweeping purposes for this specific area.
    // ShortestPathCalculator already loaded the original polygon with all zones for safe transit paths.
    std::vector<polygon_t> internal_nfz;
    for (const auto& nfz : area.no_fly_zone_polygons) {
      if (!nfz.empty() && is_point_in_polygon(nfz[0], area.fly_zone_polygon_points)) {
        internal_nfz.push_back(nfz);
      } else {
        RCLCPP_WARN(node_->get_logger(), "A no-fly zone is outside the fly zone. It will be ignored for sweeping but kept for transit paths.");
      }
    }
    area.no_fly_zone_polygons = internal_nfz;

    // Do the same for HR NFZs - for decomposition, keep only the internal ones.
    std::vector<HeightRestrictedNoFlyZone> internal_hr_nfz;
    for (const auto& hr_nfz : area.height_restricted_no_fly_zone_polygons) {
      if (!hr_nfz.polygon.empty() && is_point_in_polygon(hr_nfz.polygon[0], area.fly_zone_polygon_points)) {
        internal_hr_nfz.push_back(hr_nfz);
      } else {
        RCLCPP_WARN(node_->get_logger(), "A height restricted no-fly zone is outside the fly zone. It will be ignored for sweeping but kept for transit paths.");
      }
    }
    area.height_restricted_no_fly_zone_polygons = internal_hr_nfz;

    search_areas.push_back(area);
  }

  // For saving the paths for each UAV
  coverage_paths_t coverage_paths_tmp;

  mstsp_solver::final_solution_t best_solution;
  try {
    planner_config_.start_pos.first  = mission.robots.at(0).global_position.x;
    planner_config_.start_pos.second = mission.robots.at(0).global_position.y;

    best_solution = solve_for_uavs(planner_config_.number_of_drones, planner_config_, search_areas, cost_calculators, shortest_path_calculator, shared_logger);

  } catch (const polygon_decomposition_error &e) {
    RCLCPP_ERROR(node_->get_logger(), "Error while decomposing the polygon");
    return coverage_paths_tmp;
  } catch (const std::runtime_error &e) {
    RCLCPP_ERROR(node_->get_logger(), "Error while decomposing the polygon: %s", e.what());
    return coverage_paths_tmp;
  }


  // Save genrated path to coverage_paths_tmp excluding some points
  for (unsigned int d = 0; d < best_solution.paths.size(); d++) {

    best_solution.paths.at(d).erase(best_solution.paths.at(d).begin());
    best_solution.paths.at(d).pop_back();
    std::vector<iroc_mission_handler::msg::Waypoint> coverage_path;

    for (auto &p : best_solution.paths.at(d)) {   //  drone_path

      mrs_msgs::msg::Reference point;
      // Fill the reference point
      point.position.x = p.x;
      point.position.y = p.y;
      point.position.z = p.z;
      point.heading    = 0.0;

      iroc_mission_handler::msg::Waypoint waypoint;
      waypoint.reference = point;
      coverage_path.push_back(waypoint);
    }
    coverage_paths_tmp.push_back(coverage_path);
  }

  // Get drone positions and start and end position of each sweeping path
  int drone_num = planner_config_.number_of_drones;
  std::vector<point_t> drone_positions(drone_num);
  std::vector<std::tuple<point_t, point_t>> path_start_end_pos(drone_num);
  for (int i = 0; i < drone_num; i++) {
    drone_positions.at(i) = gps_coordinates_to_meters({mission.robots.at(i).global_position.x, mission.robots.at(i).global_position.y}, planner_config_.lat_lon_origin);
    std::get<0>(path_start_end_pos.at(i)) = {coverage_paths_tmp.at(i).at(1).reference.position.x, coverage_paths_tmp.at(i).at(1).reference.position.y};
    std::get<1>(path_start_end_pos.at(i)) = {coverage_paths_tmp.at(i).at(coverage_paths_tmp.at(i).size() - 2).reference.position.x,coverage_paths_tmp.at(i).at(coverage_paths_tmp.at(i).size() - 2).reference.position.y};
  }
  
  // Create a matrix used for the hungarian algorithm
  std::vector<std::vector<double>> matrix(drone_num, std::vector<double>(drone_num, 0));
  for (int i = 0; i < drone_num; i++) {
    for (int j = 0; j < drone_num; j++) {
      matrix[i][j] = droneToSweepingDistance(drone_positions.at(i), std::get<0>(path_start_end_pos.at(j)), std::get<1>(path_start_end_pos.at(j)), shortest_path_calculator);
    }
  }

  // Hungarian algorithm assigns each drone the nearest sweeping path
  std::vector<int> assignment = hungarianAlgorithm(matrix);

  // coverage_paths is used to change order of the paths from coverage_paths_tmp. By changing the order of paths we assign each path to a different drone.
  coverage_paths_t coverage_paths(coverage_paths_tmp.size());
  TransitPathGroupsStruct tpgs;
  std::vector<double> min_horizontal_distances_tmp(min_horizontal_distances.size());
  std::vector<double> min_vertical_distances_tmp(min_vertical_distances.size());

  for (int i = 0; i < drone_num; i++) {
    // Saving coverage_paths_tmp into coverage_paths in different order. This way the path is assigned to a specific drone. We do this because we want to assign a coverage path to the nearest drone.
    coverage_paths.at(i) = coverage_paths_tmp.at(assignment[i]);
    min_horizontal_distances_tmp.at(i) = min_horizontal_distances.at(assignment[i]);
    min_vertical_distances_tmp.at(i) = min_vertical_distances.at(assignment[i]);
    
    // Calculates the path from the drone's starting position to the start of the sweeping path. If the direct route is obstructed by no-fly zones, shortest_path_calculator() finds a route around them.
    // std::vector<point_t> path_from_start = shortest_path_calculator.shortest_path_between_points({drone_positions.at(i).first, drone_positions.at(i).second}, {coverage_paths.at(i).at(1).reference.position.x, coverage_paths.at(i).at(1).reference.position.y});
    auto path_res = shortest_path_calculator.shortest_path_between_points({drone_positions.at(i).first, drone_positions.at(i).second}, {coverage_paths.at(i).at(1).reference.position.x, coverage_paths.at(i).at(1).reference.position.y});
    std::vector<point_t> path_from_start = path_res.first;
    double current_transit_path_height = path_res.second;
    path_from_start.pop_back();

    std::vector<iroc_mission_handler::msg::Waypoint> path_from_start_waypoints = pointVecToWaypointVec(path_from_start, current_transit_path_height);
    coverage_paths.at(i).insert(coverage_paths.at(i).begin(), path_from_start_waypoints.begin(), path_from_start_waypoints.end());    

    // Calculates the path from the end of sweeping path to drone's end position. If the direct route is obstructed by no-fly zones, shortest_path_calculator() finds a route around them.
    // std::vector<point_t> path_to_end = shortest_path_calculator.shortest_path_between_points({coverage_paths.at(i).back().reference.position.x, coverage_paths.at(i).back().reference.position.y}, {drone_positions.at(i).first, drone_positions.at(i).second});
    path_res = shortest_path_calculator.shortest_path_between_points({coverage_paths.at(i).back().reference.position.x, coverage_paths.at(i).back().reference.position.y}, {drone_positions.at(i).first, drone_positions.at(i).second});
    std::vector<point_t> path_to_end = path_res.first;
    current_transit_path_height = path_res.second;
    path_to_end.erase(path_to_end.begin());
    std::vector<iroc_mission_handler::msg::Waypoint> path_to_end_waypoints = pointVecToWaypointVec(path_to_end, current_transit_path_height);
    coverage_paths.at(i).insert(coverage_paths.at(i).end(), path_to_end_waypoints.begin(), path_to_end_waypoints.end());
    

    // Fill the TransitPathGroupStruct
    for (unsigned int j = 1; j < coverage_paths.at(i).size(); j++) {
      custom_types::Point2D current_point = custom_types::Point2D(coverage_paths.at(i).at(j).reference.position.x, coverage_paths.at(i).at(j).reference.position.y);
      custom_types::Point2D prev_point = custom_types::Point2D(coverage_paths.at(i).at(j-1).reference.position.x, coverage_paths.at(i).at(j-1).reference.position.y);

      if (coverage_paths.at(i).at(j).reference.position.z > sweeping_height && coverage_paths.at(i).at(j-1).reference.position.z > sweeping_height &&
            (current_point.x != prev_point.x || current_point.y != prev_point.y)) { // TODO: this last condition probably isn't needed

        // Checking if the current TransitPath (composed of current_point and prev_point) connects to the last added TransitPath.
        // If it does, the last TransitPathGroup is extended. If not, a new TransitPathGroup is created.
        if (!tpgs.transit_path_groups.empty() && tpgs.transit_path_groups.back()->drone_idx == i && tpgs.transit_path_groups.back()->get().back()->x2 == prev_point.x && tpgs.transit_path_groups.back()->get().back()->y2 == prev_point.y) {
          tpgs.transit_path_groups.back()->addTransitPath(prev_point.x, prev_point.y, current_point.x, current_point.y, &coverage_paths.at(i).at(j-1).reference.position.z, &coverage_paths.at(i).at(j).reference.position.z);
        } else {
          std::unique_ptr<TransitPathGroup> tpg(new TransitPathGroup(i, min_horizontal_distances_tmp.at(i), min_vertical_distances_tmp.at(i)));
          tpg->addTransitPath(prev_point.x, prev_point.y, current_point.x, current_point.y, &coverage_paths.at(i).at(j-1).reference.position.z, &coverage_paths.at(i).at(j).reference.position.z);
          tpg->setHeight(coverage_paths.at(i).at(j).reference.position.z, sweeping_height, transit_path_height);
          tpgs.transit_path_groups.push_back(std::move(tpg));
        }
      }
    }
  }

  min_horizontal_distances = min_horizontal_distances_tmp;
  min_vertical_distances = min_vertical_distances_tmp;

  // Fill the transit_paths_under vector.
  tpgs.transit_paths_under.insert(tpgs.transit_paths_under.end(), tpgs.transit_path_groups.size(), std::vector<int>());
  for (unsigned int i = 0; i < tpgs.transit_path_groups.size(); i++) {
    for (unsigned int j = i+1; j < tpgs.transit_path_groups.size(); j++) {
      if (tpgs.transit_path_groups.at(i)->drone_idx == tpgs.transit_path_groups.at(j)->drone_idx) continue; 

      int r = horizontalAndVerticalTPGIntersection(*tpgs.transit_path_groups.at(i), *tpgs.transit_path_groups.at(j), std::max(min_horizontal_distances.at(tpgs.transit_path_groups.at(i)->drone_idx), min_horizontal_distances.at(tpgs.transit_path_groups.at(j)->drone_idx)));
      if (r == -1 && !checkForPotentialCycle(tpgs.transit_paths_under, j, i)) {
        tpgs.transit_paths_under.at(i).push_back(j);
      } else if (r == 1 && !checkForPotentialCycle(tpgs.transit_paths_under, i, j)) {
        tpgs.transit_paths_under.at(j).push_back(i);
      }
    }
  }
  
  // Graph is created. In this graph vertexes are transit path groups and edges mean that two transit path groups overlap
  Graph graph = Graph(tpgs.transit_path_groups.size());
  for (unsigned int i = 0; i < tpgs.transit_path_groups.size(); i++) {
    for (unsigned int j = i+1; j < tpgs.transit_path_groups.size(); j++) {
      if (tpgs.transit_path_groups.at(i)->drone_idx != tpgs.transit_path_groups.at(j)->drone_idx && checkOverlap2(*tpgs.transit_path_groups.at(i), *tpgs.transit_path_groups.at(j), std::max(min_horizontal_distances.at(tpgs.transit_path_groups.at(i)->drone_idx), min_horizontal_distances.at(tpgs.transit_path_groups.at(j)->drone_idx)))) {
        graph.addEdge(i, j);
      }
    }
  }
  
  resolveTransitHeights(tpgs, coverage_paths, graph, sweeping_height, min_horizontal_distances, min_vertical_distances);

  // Check if path is within cost constraints
  for (int i = 0; i < planner_config_.number_of_drones; i++) {
    double path_cost = cost_calculators.at(i)->calculate_path_cost(convertWaypointsToPointHeading(coverage_paths.at(i)));
    double cost_limit = planner_config_.drones.at(i).max_single_path_cost;

    if (path_cost > cost_limit) {
      RCLCPP_ERROR(node_->get_logger(), "Found solution for drone %d costs %.2f %s which exceeds the limit of %.2f %s. Try to increase the number of drones.", i, path_cost, (planner_config_.drones.at(0).optimization_type == "energy" ? "Joules" : "seconds"), cost_limit,
        (planner_config_.drones.at(0).optimization_type == "energy" ? "Joules" : "seconds"));
      coverage_paths_t empty_path;
      return empty_path;
    }
  }

  // Covert coverage_paths to gps coordinates
  for (int i = 0; i < drone_num; i++) {
    for (unsigned int j = 0; j < coverage_paths.at(i).size(); j++) {
      point_t d2 = meters_to_gps_coordinates({coverage_paths.at(i).at(j).reference.position.x, coverage_paths.at(i).at(j).reference.position.y}, planner_config_.lat_lon_origin);
      coverage_paths.at(i).at(j).reference.position.x = d2.first;
      coverage_paths.at(i).at(j).reference.position.y = d2.second;
    }
  }

  return coverage_paths;
}





std::vector<iroc_mission_handler::msg::Waypoint> pointVecToWaypointVec(std::vector<point_t> &points, double transit_path_height)
{
  std::vector<iroc_mission_handler::msg::Waypoint> waypoint_vec;
  for (point_t &p : points) {
    iroc_mission_handler::msg::Waypoint waypoint;
    waypoint.reference.position.x = p.first;
    waypoint.reference.position.y = p.second;
    waypoint.reference.position.z = transit_path_height;
    waypoint.reference.heading    = 0.0;
    waypoint_vec.push_back(waypoint);
  }
  return waypoint_vec;
}

bool checkForPotentialCycle(std::vector<std::vector<int>> &transit_paths_under, int starting_idx, int search_idx)
{
  for (int index : transit_paths_under.at(starting_idx)) {
    if (index == search_idx || checkForPotentialCycle(transit_paths_under, index, search_idx)) return true;
  }
  return false;
}

// return -1 => tpg1 should be above tpg2, return 0 => there should not be any constrain, return 1 => tpg2 should be above tpg1
int horizontalAndVerticalTPGIntersection(TransitPathGroup &tpg1, TransitPathGroup &tpg2, double drone_distance)
{
  bool tpg1_above_tpg2 = false;

  point_t start = {tpg2.get().front()->x1, tpg2.get().front()->y1};
  point_t end = {tpg2.get().back()->x2, tpg2.get().back()->y2};
  for (auto &tp : tpg1.get()) {
    // If the starting point or ending point of tpg2 is close to any of the tpg1 line segments, then tpg1 should be above tpg2
    custom_types::Point2D start_pt = {start.first, start.second};
    custom_types::Point2D end_pt = {end.first, end.second};
    custom_types::Point2D tp_start = {tp->x1, tp->y1};
    custom_types::Point2D tp_end = {tp->x2, tp->y2};
    if (pointToSegmentDistance(start_pt, tp_start, tp_end) < drone_distance || pointToSegmentDistance(end_pt, tp_start, tp_end) < drone_distance) {
      tpg1_above_tpg2 = true;
      break;
    }
  }

  bool tpg2_above_tpg1 = false;
  start = {tpg1.get().front()->x1, tpg1.get().front()->y1};
  end = {tpg1.get().back()->x2, tpg1.get().back()->y2};
  for (auto &tp : tpg2.get()) {
    // If the starting point or ending point of tpg2 is close to any of the tpg1 line segments, then tpg1 should be above tpg2
    custom_types::Point2D start_pt = {start.first, start.second};
    custom_types::Point2D end_pt = {end.first, end.second};
    custom_types::Point2D tp_start = {tp->x1, tp->y1};
    custom_types::Point2D tp_end = {tp->x2, tp->y2};
    if (pointToSegmentDistance(start_pt, tp_start, tp_end) < drone_distance || pointToSegmentDistance(end_pt, tp_start, tp_end) < drone_distance) {
      tpg2_above_tpg1 = true;
      break;
    }
  }

  if (tpg1_above_tpg2 == tpg2_above_tpg1) {
    return 0;
  } else if (tpg1_above_tpg2) {
    return -1;
  } else {
    return 1;
  }
}

// Helper function to calculate squared distance between two points
double distSq(custom_types::Point2D p1, custom_types::Point2D p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

// Helper function to find the minimum distance from a point to a line segment
double pointToSegmentDistance(custom_types::Point2D p, custom_types::Point2D s1, custom_types::Point2D s2) {
    double l2 = distSq(s1, s2);
    if (l2 == 0.0) return std::sqrt(distSq(p, s1)); // Segment is just a point

    // Consider the line extending the segment, parameterized as s1 + t (s2 - s1).
    // We find projection of point p onto the line. 
    // It falls where t = [(p-s1) . (s2-s1)] / |s2-s1|^2
    double t = ((p.x - s1.x) * (s2.x - s1.x) + (p.y - s1.y) * (s2.y - s1.y)) / l2;
    
    // Clamp t to the range [0, 1] to stay on the segment
    t = std::max(0.0, std::min(1.0, t));
    
    custom_types::Point2D projection = { s1.x + t * (s2.x - s1.x), s1.y + t * (s2.y - s1.y) };
    return std::sqrt(distSq(p, projection));
}

// Function to check if two segments intersect
bool segmentsIntersect(TransitPath tp1, TransitPath tp2) {
    auto ccw = [](double ax, double ay, double bx, double by, double cx, double cy) {
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax);
    };
    
    bool intersect = (ccw(tp1.x1, tp1.y1, tp2.x1, tp2.y1, tp2.x2, tp2.y2) != ccw(tp1.x2, tp1.y2, tp2.x1, tp2.y1, tp2.x2, tp2.y2)) &&
                     (ccw(tp1.x1, tp1.y1, tp1.x2, tp1.y2, tp2.x1, tp2.y1) != ccw(tp1.x1, tp1.y1, tp1.x2, tp1.y2, tp2.x2, tp2.y2));
    return intersect;
}

// This function checks whether two line segments overlap or if they are closer than min_distance.
bool checkOverlap(TransitPath tp1, TransitPath tp2, double min_distance)
{
    // 1. If segments intersect, the distance is 0, which is always < min_distance
    if (segmentsIntersect(tp1, tp2)) {
        return true;
    }

    // 2. Calculate minimum distance between segments.
    // The minimum distance between two non-intersecting segments 
    // is always the distance from one of the endpoints to the other segment.
    custom_types::Point2D p1_1 = custom_types::Point2D(tp1.x1, tp1.y1);
    custom_types::Point2D p1_2 = custom_types::Point2D(tp1.x2, tp1.y2);
    custom_types::Point2D p2_1 = custom_types::Point2D(tp2.x1, tp2.y1);
    custom_types::Point2D p2_2 = custom_types::Point2D(tp2.x2, tp2.y2);

    double d1 = pointToSegmentDistance(p1_1, p2_1, p2_2);
    double d2 = pointToSegmentDistance(p1_2, p2_1, p2_2);
    double d3 = pointToSegmentDistance(p2_1, p1_1, p1_2);
    double d4 = pointToSegmentDistance(p2_2, p1_1, p1_2);

    double min_actual_dist = std::min({d1, d2, d3, d4});

    return min_actual_dist < min_distance;
}

bool checkOverlap2(TransitPathGroup &tpg1, TransitPathGroup &tpg2, double min_distance)
{
  for (const std::unique_ptr<TransitPath> &tp1 : tpg1.get()) {
    for (const std::unique_ptr<TransitPath> &tp2 : tpg2.get()) {
      if (checkOverlap(*tp1, *tp2, min_distance)) {
        return true;
      }
    }
  }
  return false;
}

bool checkOverlap3(TransitPathGroup &tpg, TransitPath &tp, double min_distance)
{
  for (const std::unique_ptr<TransitPath> &tp_ : tpg.get()) {
    if (checkOverlap(*tp_, tp, min_distance)) {
      return true;
    }
  }
  return false;
}

// This function assigns each transit path an altitude (z coordinate) so that the transit paths don't overlap
void resolveTransitHeights(TransitPathGroupsStruct& tpgs, CoveragePlanner::coverage_paths_t& coverage_paths, const Graph& graph, double sweeping_height, std::vector<double> min_horizontal_distances, std::vector<double> min_vertical_distances)
{
  int n = graph.V;
  if (n == 0) return;

  // 1. Calculation of vertex (= transit path group) levels
  // We will be coloring graph vertexes one by one
  std::vector<double> assigned_heights(n, -1);
  std::vector<bool> processed(n, false);

  for (int i = 0; i < n; ++i) {
    int best_node = -1;
    NodePriority best_priority = {-1, -1, 1000000.0};

    // Vertex with top priority (most overlaps) is found. We only search for vertex which we didn't assign any level yet
    for (int v = 0; v < n; ++v) {
      if (processed[v]) continue;

      // skip vertex that should be higher than something not processed yet
      bool skip = false;
      for (int tpg_idxs : tpgs.transit_paths_under[v]) {
        if (!processed[tpg_idxs]) { skip = true; break; }
      }
      if (skip) continue;

      // if vertex (transit path group) is going above some height restricted no-fly zone, then possible_level should be high enough to go above the hr no-fly zone
      double possible_height = std::max(sweeping_height, tpgs.transit_path_groups.at(v)->drone_height);

      // possible height is set above transit paths that should be under current transit path
      for (int tpg_idxs : tpgs.transit_paths_under[v]) {
        possible_height = std::max(possible_height, assigned_heights[tpg_idxs] + std::max(tpgs.transit_path_groups.at(v)->min_vertical_distance, tpgs.transit_path_groups.at(tpg_idxs)->min_vertical_distance));
      }
      
      // If the transit path group ovelaps sweeping path, then the possible_height is set above it
      if (possible_height == sweeping_height) {
        for (unsigned int k = 0; k < coverage_paths.size(); k++) {
          if ((int)k == tpgs.transit_path_groups.at(v)->drone_idx) continue;

          for (unsigned int j = 1; j < coverage_paths.at(k).size(); j++) {
            iroc_mission_handler::msg::Waypoint current_point = coverage_paths.at(k).at(j);
            iroc_mission_handler::msg::Waypoint prev_point = coverage_paths.at(k).at(j-1);

            if (current_point.reference.position.z == sweeping_height && prev_point.reference.position.z == sweeping_height) {
              TransitPath tp = TransitPath(current_point.reference.position.x, current_point.reference.position.y, prev_point.reference.position.x, prev_point.reference.position.y);
              if (checkOverlap3(*tpgs.transit_path_groups.at(v), tp, std::max(tpgs.transit_path_groups.at(v)->min_horizontal_distance, min_horizontal_distances.at(k)))) {
                possible_height = std::max(possible_height, sweeping_height + std::max(tpgs.transit_path_groups.at(v)->min_vertical_distance, min_vertical_distances.at(k)));
              }
            }
          }
        }
      }

      // If vertex neighbors are closer minimum vertical distance, then possible_height is being set above it
      for (int neighbor : graph.adj[v]) {
        if (assigned_heights[neighbor] != -1 && std::abs(possible_height - assigned_heights[neighbor]) < std::max(tpgs.transit_path_groups.at(v)->min_vertical_distance, min_vertical_distances[tpgs.transit_path_groups.at(neighbor)->drone_idx])) {
          possible_height = assigned_heights[neighbor] + std::max(tpgs.transit_path_groups.at(v)->min_vertical_distance, min_vertical_distances[tpgs.transit_path_groups.at(neighbor)->drone_idx]);
        }
      }

      NodePriority current = {v, (int)graph.adj[v].size(), possible_height};
      if (best_node == -1 || current > best_priority) {
        best_priority = current;
        best_node = v;
      }
    }

    if (best_node == -1) break;

    // 2. Assigning level to a vertex
    assigned_heights[best_node] = best_priority.best_available_height;
    processed[best_node] = true;

    std::cout << "best node " << best_node << ", height " << assigned_heights[best_node] << ", drone idx " << tpgs.transit_path_groups.at(best_node)->drone_idx << std::endl;

    tpgs.transit_path_groups.at(best_node)->drone_height = assigned_heights[best_node];   // useless row
    tpgs.transit_path_groups.at(best_node)->writeTransitPathHeights(assigned_heights[best_node]);
  }
}

const double INF = std::numeric_limits<double>::max();

// Function for solving the Hungarian matching problem (min cost).
std::vector<int> hungarianAlgorithm(const std::vector<std::vector<double>>& matrix) {
    if (matrix.empty()) return {};

    int n = matrix.size();
    int m = matrix[0].size();

    std::vector<double> u(n + 1, 0), v(m + 1, 0), minv(m + 1, 0);
    std::vector<int> p(m + 1, 0), way(m + 1, 0);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::fill(minv.begin(), minv.end(), INF);
        std::vector<bool> used(m + 1, false);

        do {
            used[j0] = true;
            int i0 = p[j0], j1 = 0;
            double delta = INF;

            for (int j = 1; j <= m; ++j) {
                if (!used[j]) {
                    double cur = matrix[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    std::vector<int> result(n);
    for (int j = 1; j <= m; ++j) {
        if (p[j] != 0) {
            result[p[j] - 1] = j - 1;
        }
    }
    return result;
}

std::vector<point_heading_t<double>> convertWaypointsToPointHeading(const std::vector<iroc_mission_handler::msg::Waypoint>& iroc_waypoints)
{
    std::vector<point_heading_t<double>> path_for_calc;
    path_for_calc.reserve(iroc_waypoints.size());

    for (const auto& iroc_wp : iroc_waypoints) {
        point_heading_t<double> new_point;
        
        new_point.x = iroc_wp.reference.position.x;
        new_point.y = iroc_wp.reference.position.y;
        new_point.z = iroc_wp.reference.position.z;
        new_point.heading = iroc_wp.reference.heading;

        path_for_calc.push_back(new_point);
    }

    return path_for_calc;
}


} // namespace coverage_planner

} // namespace planners

} // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::planners::coverage_planner::CoveragePlanner, iroc_fleet_manager::planners::Planner);
