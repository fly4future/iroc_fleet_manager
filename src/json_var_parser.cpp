#include "iroc_fleet_manager/json_var_parser.h"
#include <ros/ros.h>

namespace iroc_fleet_manager {

template<typename T>
T convert_from_json(const json& j) {
    return T(j);
}

// Specializations for vector types
template<>
std::vector<Point2D> convert_from_json<std::vector<Point2D>>(const json& j) {
    std::vector<Point2D> result;
    for (const auto& item : j) {
        result.emplace_back(item);
    }
    return result;
}

template<>
std::vector<Point3D> convert_from_json<std::vector<Point3D>>(const json& j) {
    std::vector<Point3D> result;
    for (const auto& item : j) {
        result.emplace_back(item);
    }
    return result;
}

template<>
std::vector<Reference> convert_from_json<std::vector<Reference>>(const json& j) {
    std::vector<Reference> result;
    for (const auto& item : j) {
        result.emplace_back(item);
    }
    return result;
}

bool parseVar(const json &js, std::pair<std::string_view, parseable_t> &var) {
  const auto &var_name = var.first;

  if (!js.contains(var_name)) {
    ROS_ERROR_STREAM_THROTTLE(
        1.0, "[Var-parser]: JSON doesn't have the expected member \""
                 << var_name << "\".");
    return false;
  }

  auto &var_out = var.second;
  std::visit(
      [var_name, &js](auto &&var_out) {
        using T = std::remove_pointer_t<std::decay_t<decltype(var_out)>>;
        try {
          *var_out = convert_from_json<T>(js.at(var_name));
        } catch (json::exception &e) {
          ROS_ERROR_STREAM_THROTTLE(
              1.0, "[Var-parser]: Cannot parse member \""
                       << var_name << "\" (value: " << js.at(var_name)
                       << ") as custom type: " << e.what());
        } catch (std::exception &e) {
          ROS_ERROR_STREAM_THROTTLE(1.0, "[Var-parser]: Cannot parse member \""
                                             << var_name << "\" - "
                                             << e.what());
        }
      },
      var_out);
  return true;
}

bool parseVars(const json &js,
                std::vector<std::pair<std::string_view, parseable_t>> &&vars) {
  for (auto &var : vars)
    if (!parseVar(js, var))
      return false;
  return true;
}

} // namespace iroc_fleet_manager
