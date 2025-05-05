#ifndef COVERAGE_PLANNER_HPP
#define COVERAGE_PLANNER_HPP

#include "MapPolygon.hpp"
#include "EnergyCalculator.h"
#include "algorithms.hpp"
#include "ShortestPathCalculator.hpp"
#include "mstsp_solver/SolverConfig.h"
#include "mstsp_solver/MstspSolver.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include "SimpleLogger.h"
#include "utils.hpp"
#include <iomanip>

// Struct defining full algorithm config in one place
struct algorithm_config_t
{
  energy_calculator_config_t energy_calculator_config;
  int number_of_rotations;
  bool points_in_lat_lon;
  std::pair<double, double> lat_lon_origin;
  std::string fly_zone_points_file;
  std::vector<std::string> no_fly_zone_points_files;
  int number_of_drones;
  double sweeping_step;
  decomposition_type_t decomposition_type;
  int min_sub_polygons_per_uav;
  std::pair<double, double> start_pos;

  int rotations_per_cell;
  int no_improvement_cycles_before_stop;
  double max_single_path_energy;
};

/* generate_with_constraints() //{ */
template <typename F>
[[maybe_unused]] mstsp_solver::final_solution_t generate_with_constraints(double max_energy_bound, unsigned int n_uavs, F f)
{
  // Generate the initial solution that can be optimized after
  mstsp_solver::final_solution_t solution = f(n_uavs);
  if (solution.max_path_energy < max_energy_bound)
  {
    return solution;
  }

  auto current_n_uavs = n_uavs;
  int iteration = 0;
  while (solution.max_path_energy > max_energy_bound)
  {
    // Stop if too many iterations are already done. TODO: remove the hardcoded value from here
    if (++iteration > 10)
    {
      std::cout << "Could not generate paths to satisfy the upper bound on energy consumption..." << std::endl;
      return solution;
    }
    // if the energy consumption is divided well, this should be enough
    unsigned int updated_n_uavs = std::ceil(solution.path_energies_sum / max_energy_bound);

    // if the needed number of UAVs is estimated to be smaller than on previous step, make it larger
    if (updated_n_uavs <= current_n_uavs)
    {
      updated_n_uavs = current_n_uavs + 1;
    }
    current_n_uavs = updated_n_uavs;
    solution = f(current_n_uavs);
  }
  return solution;
}
//}

mstsp_solver::final_solution_t solve_for_uavs(int n_uavs, const algorithm_config_t& algorithm_config, MapPolygon polygon,
                                              const EnergyCalculator& energy_calculator, const ShortestPathCalculator& shortest_path_calculator,
                                              std::shared_ptr<loggers::SimpleLogger>& logger);


#endif //COVERAGE_PLANNER_HPP

