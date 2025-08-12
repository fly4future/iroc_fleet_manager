#include "iroc_fleet_manager/json_var_parser.h"
#include <ros/ros.h>

namespace iroc_fleet_manager {

template <typename T>
struct is_vector : std::false_type {};

template <typename U>
struct is_vector<std::vector<U>> : std::true_type {};

template <typename T>
constexpr bool is_vector_v = is_vector<T>::value;

// Updated convertFromJson function
template <typename T>
T convertFromJson(const json& j) {
  if constexpr (is_vector_v<T>) {
    T result;
    result.reserve(j.size());
    for (const auto& item : j) {
      result.emplace_back(typename T::value_type(item));
    }
    return result;
  } else {
    return T(j);
  }
}

bool parseVar(const json& js, std::pair<std::string_view, parseable_t>& var) {
  const auto& var_name = var.first;

  if (!js.contains(var_name)) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[Var-parser]: JSON doesn't have the expected member \"" << var_name << "\".");
    return false;
  }

  auto& var_out = var.second;
  std::visit(
      [var_name, &js](auto&& var_out) {
        using T = std::remove_pointer_t<std::decay_t<decltype(var_out)>>;
        try {
          *var_out = convertFromJson<T>(js.at(var_name));
        } catch (json::exception& e) {
          ROS_ERROR_STREAM_THROTTLE(1.0,
                                    "[Var-parser]: Cannot parse member \"" << var_name << "\" (value: " << js.at(var_name) << ") as custom type: " << e.what());
        } catch (std::exception& e) {
          ROS_ERROR_STREAM_THROTTLE(1.0, "[Var-parser]: Cannot parse member \"" << var_name << "\" - " << e.what());
        }
      },
      var_out);
  return true;
}

bool parseVars(const json& js, std::vector<std::pair<std::string_view, parseable_t>>&& vars) {
  for (auto& var : vars)
    if (!parseVar(js, var))
      return false;
  return true;
}

} // namespace iroc_fleet_manager
