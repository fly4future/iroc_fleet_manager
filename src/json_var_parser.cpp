#include "iroc_fleet_manager/json_var_parser.h"
#include <ros/ros.h>

namespace iroc_fleet_manager {

bool parse_var(const json &js, std::pair<std::string_view, parseable_t> &var) {
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
          *var_out = T(js.at(var_name));
        } catch (json::exception &e) {
          json type_check = T{};
          ROS_ERROR_STREAM_THROTTLE(
              1.0, "[Var-parser]: Cannot parse member \""
                       << var_name << "\" (value: " << js.at(var_name)
                       << ") as type \"" << type_check.type_name()
                       << "\": " << e.what());
        }
      },
      var_out);
  return true;
}

bool parse_vars(const json &js,
                std::vector<std::pair<std::string_view, parseable_t>> &&vars) {
  for (auto &var : vars)
    if (!parse_var(js, var))
      return false;
  return true;
}

} // namespace iroc_fleet_manager
