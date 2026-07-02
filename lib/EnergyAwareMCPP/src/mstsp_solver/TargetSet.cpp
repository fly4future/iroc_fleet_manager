#include "EnergyAwareMCPP/mstsp_solver/TargetSet.h"
#include "EnergyAwareMCPP/algorithms.hpp"
#include <iostream>
#include <algorithm>
#include <utility>
#include "EnergyAwareMCPP/PathCostCalculator.hpp"

namespace mstsp_solver
{
  /* TargetSet Constructor //{ */


  TargetSet::TargetSet(size_t index, const MapPolygon& polygon, double sweeping_step, double wall_distance, std::shared_ptr<PathCostCalculator> cost_calculator,
                       const std::vector<double>& rotation_angles)
      : index(index), polygon(polygon), cost_calculator(std::move(cost_calculator)), sweeping_step(sweeping_step), m_wall_distance{wall_distance}
  {
    set_rotation_angles(rotation_angles); // Note: sweep_alt is not available here, assuming 0. This might need adjustment.
  }
  //}

  /* TargetSet Constructor //{ */

  TargetSet::TargetSet(size_t index, const MapPolygon& polygon, double sweeping_step, double wall_distance, std::shared_ptr<PathCostCalculator> cost_calculator,
                       size_t number_of_edges_rotations)
      : index(index), polygon(polygon), cost_calculator(std::move(cost_calculator)), sweeping_step(sweeping_step), m_wall_distance{wall_distance}
  {

    auto thin_coverage = thin_polygon_coverage(polygon, sweeping_step, 4);
    // If no thin coverage path is generated because the polygon is not thin enough, perform normal sweeping procedure
    if (thin_coverage.empty())
    {
      set_rotation_angles(polygon.get_n_longest_edges_rotation_angles(number_of_edges_rotations));
    } else
    {
      targets.push_back(Target{true, 0.0, 0.0, thin_coverage[0], thin_coverage.back(), index, targets.size()});
    }
  }
  //}

  /* TargetSet::add_one_rotation_angle() //{ */
  
  void TargetSet::add_one_rotation_angle(double angle, bool up)
  {
    auto sweeping_path = sweeping(polygon, angle, sweeping_step, m_wall_distance, up);
    // If sweeping failed (e.g. because of the polygon splitting with such a rotation angle)
    if (sweeping_path.empty()) { return; }

    std::vector<point_heading_t<double>> path_with_z;
    path_with_z.reserve(sweeping_path.size());
    for(const auto& p : sweeping_path) {
        point_heading_t<double> p_3d(p);
        path_with_z.push_back(p_3d);
    }
    double path_cost = cost_calculator->calculate_path_cost(path_with_z);

    targets.push_back(Target{up, angle, path_cost, sweeping_path[0], sweeping_path[sweeping_path.size() - 1], index, targets.size()});
  }
  //}

  /* TargetSet::set_rotation_angles() //{ */
  
  void TargetSet::set_rotation_angles(const std::vector<double>& angles)
  {
    targets.clear();
    for (auto angle : angles)
    {
      for (int i = 0; i < 2; i++)
      {
        add_one_rotation_angle(angle, static_cast<bool>(i));
      }
    }
    if (targets.empty())
    {
      // If not sweeping angle produced a valid sweeping pattern
      // Try to add the sweeping with no angle. This should work for any polygon after boustrophedon decomposition
      add_one_rotation_angle(0, true);
      add_one_rotation_angle(0, false);
    }
  }
  //}
}  // namespace mstsp_solver
