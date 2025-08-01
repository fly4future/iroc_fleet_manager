#pragma once

#include <variant>
#include <nlohmann/json.hpp>

namespace iroc_fleet_manager
{

  using json = nlohmann::json;

  using parseable_t = std::variant
    <
      json*,
      bool*,
      int*,
      double*,
      std::string*,
      std::vector<int>*,
      std::vector<double>*,
      std::vector<std::string>*
    >;

  bool parse_vars(const json& js, std::vector<std::pair<std::string_view, parseable_t>>&& vars);

}
