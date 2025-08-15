#pragma once

#include <nlohmann/json.hpp>
#include "iroc_fleet_manager/utils/types.h"
#include <variant>

namespace iroc_fleet_manager
{

namespace utils
{

using json   = nlohmann::json;
namespace ct = custom_types;

using parseable_t = std::variant<json *, bool *, int *, double *, std::string *, std::vector<int> *, std::vector<double> *, std::vector<std::string> *,
                                 ct::Point2D *, ct::Point3D *, ct::Reference *, ct::Waypoint *, ct::Subtask *, std::vector<ct::Point2D> *,
                                 std::vector<ct::Point3D> *, std::vector<ct::Reference> *, std::vector<ct::Waypoint> *, std::vector<ct::Subtask> *>;

bool parseVars(const json &js, std::vector<std::pair<std::string_view, parseable_t>> &&vars);

} // namespace utils
} // namespace iroc_fleet_manager
