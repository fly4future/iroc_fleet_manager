#include <iroc_fleet_manager/iroc_plugins/coverage_planner.h>

namespace iroc_fleet_manager
{

namespace planners
{

namespace coverage_planner
{

bool CoveragePlanner::initialize(const ros::NodeHandle &parent_nh, const std::string &name, const std::string &name_space,
                                 std::shared_ptr<iroc_fleet_manager::CommonHandlers_t> common_handlers) {

  // nh_ will behave just like normal NodeHandle
  ros::NodeHandle nh_(parent_nh, name_space);

  name_            = name;
  common_handlers_ = common_handlers;
  ros::Time::waitForValid();

  /* load parameters */
  mrs_lib::ParamLoader param_loader(nh_, "CoveragePlanner");

  param_loader.addYamlFile(ros::package::getPath("iroc_fleet_manager") + "/config/coverage_planner_config.yaml");

  planner_config_ = parse_algorithm_config(param_loader);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: could not load all parameters!", name_.c_str());
    is_initialized_ = false;
    return true;
  }

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[%s]: initialized under the name '%s', namespace '%s' and action ", name_.c_str(), name.c_str(), name_space.c_str());

  is_initialized_ = true;
  return true;
}

bool CoveragePlanner::activate(void) {

  ROS_INFO("[%s]: activated", name_.c_str());

  is_active_ = true;

  return true;
}

void CoveragePlanner::deactivate(void) {

  is_active_ = false;

  ROS_INFO("[%s]: deactivated", name_.c_str());
}

std::tuple<result_t, std::vector<iroc_mission_handler::MissionGoal>> CoveragePlanner::createGoal(const std::string &goal) const {
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
    result.success = false;
    result.message = "Faile to parse JSON msg";
    return std::make_tuple(result, mission_robots);
  }

  using HRNoFlyZone = std::pair<std::vector<custom_types::Point2D>, double>;
  std::vector<std::vector<custom_types::Point2D>> search_areas;
  std::vector<std::vector<custom_types::Point2D>> no_fly_zones;
  std::vector<HRNoFlyZone> hr_no_fly_zones;
  std::vector<double> min_horizontal_drone_distances;
  std::vector<double> min_vertical_drone_distances;
  json robots;
  int frame_id;
  int height;
  int height_id;
  int terminal_action;
  

  bool success = utils::parseVars(json_msg, {
                                                {"search_areas", &search_areas},
                                                {"min_horizontal_drone_distances", &min_horizontal_drone_distances},
                                                {"min_vertical_drone_distances", &min_vertical_drone_distances},
                                                {"robots", &robots},
                                                {"height", &height},
                                                {"height_id", &height_id},
                                                {"terminal_action", &terminal_action}
                                            });
  if (!success) {
      result.success = false;
      result.message = "Failure while parsing robot data, bad JSON request";
      return std::make_tuple(result, mission_robots);
  }

  // parsing optional parameters
  success = utils::parseVars(json_msg, {{"no_fly_zones", &no_fly_zones}});
  success = utils::parseVars(json_msg, {{"hr_no_fly_zones", &hr_no_fly_zones}});
  
  // Extract robots
  robots_msg.reserve(robots.size());
  for (const auto &robot : robots) {
    iroc_fleet_manager::CoverageMissionRobot robot_msg;
    std::string name;

    name = robot.get<std::string>();

    bool isRobotInFleet = common_handlers_->handlers->robots_map.count(name);

    if (!isRobotInFleet) {
      ROS_WARN("[CoveragePlanner] Robot %s not within the fleet", name.c_str());
      std::stringstream ss;
      ss << name << " not found in the fleet!";
      result.message = ss.str();
      result.success = false;
      return std::make_tuple(result, mission_robots);
    }

    robot_msg.name            = name;
    robot_msg.frame_id        = iroc_mission_handler::MissionGoal::FRAME_ID_LATLON;
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

  latlon_origin_msg.x = common_handlers_->handlers->robots_map[robots_msg.at(0).name].safety_area_info->safety_area.origin_x;
  latlon_origin_msg.y = common_handlers_->handlers->robots_map[robots_msg.at(0).name].safety_area_info->safety_area.origin_y;

  iroc_fleet_manager::CoverageMission mission;
  mission.robots        = robots_msg;
  mission.latlon_origin = latlon_origin_msg;

  auto paths = getCoveragePaths(mission, search_areas, no_fly_zones, hr_no_fly_zones);

  // Filling the mission_robots vector with the generated paths
  for (int it = 0; it < mission.robots.size(); it++) {
    iroc_mission_handler::MissionGoal robot;
    robot.name            = mission.robots[it].name;
    robot.points          = paths[it];
    robot.terminal_action = mission.robots[it].terminal_action;
    robot.height_id       = mission.robots[it].height_id;
    robot.frame_id        = mission.robots[it].frame_id;
    mission_robots.push_back(robot);
  }

  ROS_INFO("[CoveragePlanner] Goal created successfully!");
  result.success = true;
  result.message = "Goal created successfully";
  return std::make_tuple(result, mission_robots);
}


algorithm_config_t CoveragePlanner::parse_algorithm_config(mrs_lib::ParamLoader &param_loader) const {
  const std::string yaml_prefix = "fleet_manager/planners/coverage_planner/";
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
  int level;
  int drone_idx;

  // double drone_safety_height;
  // double drone_safety_width;
  // double drone_height;

  TransitPathGroup(int drone_idx) : drone_idx(drone_idx) {
    level = -1;
  }

  // transit_path_group should be read only for user
  const std::vector<std::unique_ptr<TransitPath>>& get() const {
    return transit_path_group;
  }

  void setLevelFromHeight(double height, double sweeping_height, double transit_height, double level_height) {
    if (height == transit_height) {
      level = 0;
    } else {
      level = std::ceil((height - sweeping_height) / level_height);
    }
  }

  void writeTransitPathHeights(double sweeping_height, double level_height) {
    for (double* &z_ptr : z_ptrs) {
      if (z_ptr) *(z_ptr) = sweeping_height + level * level_height;
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
    int best_available_level;

    // Logic of deciding which vertex has more prioriy:
    // 1. Higher degree (number of overlaps) has more priority
    // 2. If degrees are equal, more priority has a vertex which can be moved to lower level 
    bool operator>(const NodePriority& other) const {
        if (degree != other.degree) {
            return degree > other.degree;
        }
        return best_available_level < other.best_available_level;
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
void resolveTransitHeights(TransitPathGroupsStruct& tpgs, CoveragePlanner::coverage_paths_t& coverage_paths, const Graph& graph, double level_height, double sweeping_height);
std::vector<iroc_mission_handler::Waypoint> pointVecToWaypointVec(std::vector<point_t> &points, double transit_path_height);


// Calculates the distance between a drone position and start and end of sweeping trajectory
double droneToSweepingDistance(point_t drone_pos, point_t start, point_t end, ShortestPathCalculator shortest_path_calculator)
{
  double distance = 0;
  std::vector<point_t> path_to_start = shortest_path_calculator.shortest_path_between_points({drone_pos.first, drone_pos.second}, {start.first, start.second}).first;
  for (int i = 1; i < path_to_start.size(); i++) {
    distance += std::sqrt(pow(path_to_start.at(i-1).first - path_to_start.at(i).first, 2) + pow(path_to_start.at(i-1).second - path_to_start.at(i).second, 2));
  }
  std::vector<point_t> path_to_end = shortest_path_calculator.shortest_path_between_points({drone_pos.first, drone_pos.second}, {end.first, end.second}).first;
  for (int i = 1; i < path_to_end.size(); i++) {
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

CoveragePlanner::coverage_paths_t CoveragePlanner::getCoveragePaths(const iroc_fleet_manager::CoverageMission &mission, const std::vector<std::vector<custom_types::Point2D>> &search_areas_arg, const std::vector<std::vector<custom_types::Point2D>> &no_fly_zones_arg, const std::vector<std::pair<std::vector<custom_types::Point2D>, double>> &hr_no_fly_zones_arg) const {

  std::vector<polygon_t> fly_zones;
  std::vector<polygon_t> no_fly_zones;
  std::vector<std::pair<polygon_t, double>> hr_no_fly_zones;

  // Fill the fly zones
  fly_zones.reserve(search_areas_arg.size());
  for (const auto &search_area : search_areas_arg) {
    std::vector<point_t> fly_zone;
    for (const auto &point : search_area) {
      fly_zone.emplace_back(point.x, point.y);
    }
    // Add the first point to close the polygon
    if (!fly_zone.empty()) {
      fly_zone.emplace_back(search_area[0].x, search_area[0].y);
    }
    fly_zones.push_back(fly_zone);
  }

  // Fill the no-fly zones
  no_fly_zones.reserve(no_fly_zones_arg.size());
  for (const auto &zone : no_fly_zones_arg) {
    std::vector<point_t> no_fly_zone;
    for (const auto &point : zone) {
      no_fly_zone.emplace_back(point.x, point.y);
    }
    // Add the first point to close the polygon
    if (!no_fly_zone.empty()) {
      no_fly_zone.emplace_back(zone[0].x, zone[0].y);
    }
    no_fly_zones.push_back(no_fly_zone);
  }

  // Fill the height restricted no-fly zones
  hr_no_fly_zones.reserve(hr_no_fly_zones_arg.size());
  for (const auto &zone : hr_no_fly_zones_arg) {
    polygon_t hr_no_fly_zone;
    hr_no_fly_zone.reserve(zone.first.size() + 1);
    for (const auto &point : zone.first) {
      hr_no_fly_zone.emplace_back(point.x, point.y);
    }
    // Add the first point to close the polygon
    if (!hr_no_fly_zone.empty()) {
      hr_no_fly_zone.emplace_back(zone.first[0].x, zone.first[0].y);
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
        ROS_ERROR("Polygon edges of Fly zones, No-fly zones and Height restricted no-fly zones must not overlap.");
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
  EnergyCalculator energy_calculator{planner_config_.energy_calculator_config, shared_logger};
  std::cout << "Energy calculator created. Optimal speed: " << energy_calculator.get_optimal_speed() << std::endl;


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

    // Odstranění vnějších no-fly zón pouze pro potřeby dekompozice a sweepování pro TUTO konkrétní oblast.
    // ShortestPathCalculator si již načetl původní polygon se všemi zónami pro bezpečné přelety.
    std::vector<polygon_t> internal_nfz;
    for (const auto& nfz : area.no_fly_zone_polygons) {
      if (!nfz.empty() && is_point_in_polygon(nfz[0], area.fly_zone_polygon_points)) {
        internal_nfz.push_back(nfz);
      } else {
        ROS_WARN("A no-fly zone is outside the fly zone. It will be ignored for sweeping but kept for transit paths.");
      }
    }
    area.no_fly_zone_polygons = internal_nfz;

    // To samé pro HR NFZ - pro dekompozici ponecháme jen ty vnitřní.
    std::vector<HeightRestrictedNoFlyZone> internal_hr_nfz;
    for (const auto& hr_nfz : area.height_restricted_no_fly_zone_polygons) {
      if (!hr_nfz.polygon.empty() && is_point_in_polygon(hr_nfz.polygon[0], area.fly_zone_polygon_points)) {
        internal_hr_nfz.push_back(hr_nfz);
      } else {
        ROS_WARN("A height restricted no-fly zone is outside the fly zone. It will be ignored for sweeping but kept for transit paths.");
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
    auto f = [&](int n) {
      return solve_for_uavs(n, planner_config_, search_areas, energy_calculator, shortest_path_calculator,
                            shared_logger);
    };
    best_solution = generate_with_constraints(planner_config_.max_single_path_energy * 3600,
                                              planner_config_.number_of_drones, f);

  } catch (const polygon_decomposition_error &e) {
    ROS_ERROR("Error while decomposing the polygon");
    return coverage_paths_tmp;
  } catch (const std::runtime_error &e) {
    ROS_ERROR("Error while decomposing the polygon: %s", e.what());
    return coverage_paths_tmp;
  }


  // Save genrated path to coverage_paths_tmp excluding some points
  // for (auto &drone_path : best_solution.paths) {
  for (int d = 0; d < best_solution.paths.size(); d++) {

    best_solution.paths.at(d).erase(best_solution.paths.at(d).begin());
    best_solution.paths.at(d).pop_back();
    std::vector<iroc_mission_handler::Waypoint> coverage_path;

    for (auto &p : best_solution.paths.at(d)) {   //  drone_path

      mrs_msgs::Reference point;
      // Fill the reference point
      point.position.x = p.x;
      point.position.y = p.y;
      point.position.z = p.z;
      point.heading    = 0.0;

      iroc_mission_handler::Waypoint waypoint;
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
  double drone_distance = 5;  // [m]
  double drone_width_dist = 5;
  double drone_height_dist = 5;

  for (int i = 0; i < drone_num; i++) {
    // Saving coverage_paths_tmp into coverage_paths in different order. This way the path is assigned to a specific drone. We do this because we want to assign a coverage path to the nearest drone.
    coverage_paths.at(i) = coverage_paths_tmp.at(assignment[i]);
    
    // Calculates the path from the drone's starting position to the start of the sweeping path. If the direct route is obstructed by no-fly zones, shortest_path_calculator() finds a route around them.
    // std::vector<point_t> path_from_start = shortest_path_calculator.shortest_path_between_points({drone_positions.at(i).first, drone_positions.at(i).second}, {coverage_paths.at(i).at(1).reference.position.x, coverage_paths.at(i).at(1).reference.position.y});
    auto path_res = shortest_path_calculator.shortest_path_between_points({drone_positions.at(i).first, drone_positions.at(i).second}, {coverage_paths.at(i).at(1).reference.position.x, coverage_paths.at(i).at(1).reference.position.y});
    std::vector<point_t> path_from_start = path_res.first;
    double current_transit_path_height = path_res.second;
    path_from_start.pop_back();

    std::vector<iroc_mission_handler::Waypoint> path_from_start_waypoints = pointVecToWaypointVec(path_from_start, current_transit_path_height);
    coverage_paths.at(i).insert(coverage_paths.at(i).begin(), path_from_start_waypoints.begin(), path_from_start_waypoints.end());    

    // Calculates the path from the end of sweeping path to drone's end position. If the direct route is obstructed by no-fly zones, shortest_path_calculator() finds a route around them.
    // std::vector<point_t> path_to_end = shortest_path_calculator.shortest_path_between_points({coverage_paths.at(i).back().reference.position.x, coverage_paths.at(i).back().reference.position.y}, {drone_positions.at(i).first, drone_positions.at(i).second});
    path_res = shortest_path_calculator.shortest_path_between_points({coverage_paths.at(i).back().reference.position.x, coverage_paths.at(i).back().reference.position.y}, {drone_positions.at(i).first, drone_positions.at(i).second});
    std::vector<point_t> path_to_end = path_res.first;
    current_transit_path_height = path_res.second;
    path_to_end.erase(path_to_end.begin());
    std::vector<iroc_mission_handler::Waypoint> path_to_end_waypoints = pointVecToWaypointVec(path_to_end, current_transit_path_height);
    coverage_paths.at(i).insert(coverage_paths.at(i).end(), path_to_end_waypoints.begin(), path_to_end_waypoints.end());
    

    // Fill the TransiPathGroupStruct
    for (int j = 1; j < coverage_paths.at(i).size(); j++) {
      custom_types::Point2D current_point = custom_types::Point2D(coverage_paths.at(i).at(j).reference.position.x, coverage_paths.at(i).at(j).reference.position.y);
      custom_types::Point2D prev_point = custom_types::Point2D(coverage_paths.at(i).at(j-1).reference.position.x, coverage_paths.at(i).at(j-1).reference.position.y);

      if (coverage_paths.at(i).at(j).reference.position.z > sweeping_height && coverage_paths.at(i).at(j-1).reference.position.z > sweeping_height &&
            (current_point.x != prev_point.x || current_point.y != prev_point.y)) { // TODO: this last condition probably isn't needed

        // Checking if the current TransitPath (composed of current_point and prev_point) connects to the last added TransitPath.
        // If it does, the last TransitPathGroup is extended. If not, a new TransitPathGroup is created.
        if (!tpgs.transit_path_groups.empty() && tpgs.transit_path_groups.back()->drone_idx == i && tpgs.transit_path_groups.back()->get().back()->x2 == prev_point.x && tpgs.transit_path_groups.back()->get().back()->y2 == prev_point.y) {
          tpgs.transit_path_groups.back()->addTransitPath(prev_point.x, prev_point.y, current_point.x, current_point.y, &coverage_paths.at(i).at(j-1).reference.position.z, &coverage_paths.at(i).at(j).reference.position.z);
        } else {
          std::unique_ptr<TransitPathGroup> tpg(new TransitPathGroup(i));
          tpg->addTransitPath(prev_point.x, prev_point.y, current_point.x, current_point.y, &coverage_paths.at(i).at(j-1).reference.position.z, &coverage_paths.at(i).at(j).reference.position.z);
          tpg->setLevelFromHeight(coverage_paths.at(i).at(j).reference.position.z, sweeping_height, transit_path_height, drone_distance);
          tpgs.transit_path_groups.push_back(std::move(tpg));
        }
      }
    }
  }

  // Fill the transit_paths_under vector.
  tpgs.transit_paths_under.insert(tpgs.transit_paths_under.end(), tpgs.transit_path_groups.size(), std::vector<int>());
  for (int i = 0; i < tpgs.transit_path_groups.size(); i++) {
    for (int j = i+1; j < tpgs.transit_path_groups.size(); j++) {
      if (tpgs.transit_path_groups.at(i)->drone_idx == tpgs.transit_path_groups.at(j)->drone_idx) continue; 

      int r = horizontalAndVerticalTPGIntersection(*tpgs.transit_path_groups.at(i), *tpgs.transit_path_groups.at(j), drone_distance);
      if (r == -1 && !checkForPotentialCycle(tpgs.transit_paths_under, j, i)) {
        tpgs.transit_paths_under.at(i).push_back(j);
      } else if (r == 1 && !checkForPotentialCycle(tpgs.transit_paths_under, i, j)) {
        tpgs.transit_paths_under.at(j).push_back(i);
      }
    }
  }
  
  // Graph is created. In this graph vertexes are transit path groups and edges mean that two transit path groups overlap
  Graph graph = Graph(tpgs.transit_path_groups.size());
  for (int i = 0; i < tpgs.transit_path_groups.size(); i++) {
    for (int j = i+1; j < tpgs.transit_path_groups.size(); j++) {
      if (tpgs.transit_path_groups.at(i)->drone_idx != tpgs.transit_path_groups.at(j)->drone_idx && checkOverlap2(*tpgs.transit_path_groups.at(i), *tpgs.transit_path_groups.at(j), drone_distance)) { // drone_distance
        graph.addEdge(i, j);
      }
    }
  }
  
  resolveTransitHeights(tpgs, coverage_paths, graph, drone_distance, mission.robots[0].height);

  // Covert coverage_paths to gps coordinates
  for (int i = 0; i < drone_num; i++) {
    for (int j = 0; j < coverage_paths.at(i).size(); j++) {
      point_t d2 = meters_to_gps_coordinates({coverage_paths.at(i).at(j).reference.position.x, coverage_paths.at(i).at(j).reference.position.y}, planner_config_.lat_lon_origin);
      coverage_paths.at(i).at(j).reference.position.x = d2.first;
      coverage_paths.at(i).at(j).reference.position.y = d2.second;
    }
  }

  return coverage_paths;
}





std::vector<iroc_mission_handler::Waypoint> pointVecToWaypointVec(std::vector<point_t> &points, double transit_path_height)
{
  std::vector<iroc_mission_handler::Waypoint> waypoint_vec;
  for (point_t &p : points) {
    iroc_mission_handler::Waypoint waypoint;
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
void resolveTransitHeights(TransitPathGroupsStruct& tpgs, CoveragePlanner::coverage_paths_t& coverage_paths, const Graph& graph, double level_height, double sweeping_height)
{
  int n = graph.V;
  if (n == 0) return;

  // 1. Calculation of vertex (= transit path group) levels
  // We will be coloring graph vertexes one by one
  std::vector<int> assigned_levels(n, -1);
  std::vector<bool> processed(n, false);

  for (int i = 0; i < n; ++i) {
    int best_node = -1;
    NodePriority best_priority = {-1, -1, 1000000};

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
      int possible_level = std::max(0, tpgs.transit_path_groups.at(v)->level);

      for (int tpg_idxs : tpgs.transit_paths_under[v]) {
        possible_level = std::max(possible_level, assigned_levels[tpg_idxs] + 1);
      }
      // If the transit path group ovelaps sweeping path, then the possible_level is set to 1
      if (possible_level == 0) {
        bool level_zero = true;
        for (int k = 0; k < coverage_paths.size(); k++) {
          if (k == tpgs.transit_path_groups.at(v)->drone_idx) continue;

          for (int j = 1; j < coverage_paths.at(k).size(); j++) {
            iroc_mission_handler::Waypoint current_point = coverage_paths.at(k).at(j);
            iroc_mission_handler::Waypoint prev_point = coverage_paths.at(k).at(j-1);

            if (current_point.reference.position.z == sweeping_height && prev_point.reference.position.z == sweeping_height) {
              TransitPath tp = TransitPath(current_point.reference.position.x, current_point.reference.position.y, prev_point.reference.position.x, prev_point.reference.position.y);
              if (checkOverlap3(*tpgs.transit_path_groups.at(v), tp, level_height)) {
                level_zero = false;
                break;
              }
            }
          }
          if (level_zero == false) break; 
        }
        if (level_zero == false) possible_level = 1;
      }

      // Picking the lowest possible level that this vertex can get
      std::set<int> neighbor_levels;
      for (int neighbor : graph.adj[v]) {
        if (assigned_levels[neighbor] != -1) {
          neighbor_levels.insert(assigned_levels[neighbor]);
        }
      }

      while (neighbor_levels.count(possible_level)) {
        possible_level++;
      }

      NodePriority current = {v, (int)graph.adj[v].size(), possible_level};
      if (best_node == -1 || current > best_priority) {
        best_priority = current;
        best_node = v;
      }
    }

    if (best_node == -1) break;

    // 2. Assigning level to a vertex
    assigned_levels[best_node] = best_priority.best_available_level;
    processed[best_node] = true;

    std::cout << "best node " << best_node << ", level " << assigned_levels[best_node] << ", drone idx " << tpgs.transit_path_groups.at(best_node)->drone_idx << std::endl;
    
    tpgs.transit_path_groups.at(best_node)->level = assigned_levels[best_node];
    tpgs.transit_path_groups.at(best_node)->writeTransitPathHeights(sweeping_height, level_height);
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

} // namespace coverage_planner

} // namespace planners

} // namespace iroc_fleet_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_fleet_manager::planners::coverage_planner::CoveragePlanner, iroc_fleet_manager::planners::Planner);
