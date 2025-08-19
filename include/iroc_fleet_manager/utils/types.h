#pragma once

#include <nlohmann/json.hpp>

namespace iroc_fleet_manager {

namespace custom_types {

using json = nlohmann::json;

// Custom types to simplify the parsing and construction from JSON messages

struct Point2D {
  double x, y;

  Point2D() : x(0), y(0) {
  }
  Point2D(const json& j) : x(j.at("x")), y(j.at("y")) {
  }
};

struct Point3D {
  double x, y, z;

  Point3D() : x(0), y(0), z(0) {
  }
  Point3D(const json& j) : x(j.at("x")), y(j.at("y")), z(j.at("z")) {
  }
};

struct Reference {
  double x, y, z, heading;

  Reference() : x(0), y(0), z(0), heading(0) {
  }
  Reference(const json& j) : x(j.at("x")), y(j.at("y")), z(j.at("z")), heading(j.at("heading")) {
  }
};

struct Subtask {
  std::string type;
  std::string parameters;
  bool continue_without_waiting = false;
  bool stop_on_failure = false;
  int  max_retries = 0;
  int  retry_delay = 0; 
 

  Subtask() = default;

  // Json constructor 
  Subtask(const json &j)
      : type(j.at("type")), 
        parameters(j.at("parameters").dump()), 
        continue_without_waiting(j.value("continue_without_waiting", false)),
        stop_on_failure(j.value("stop_on_failure", false)),
        max_retries(j.value("max_retries", 0)),
        retry_delay(j.value("retry_delay", 0))
      {}
};

struct Waypoint {
  Reference reference;
  json subtasks;
  bool parallel_execution;
  Waypoint() : reference(), subtasks(json::array()) {
  }
  // Constructor initialization
  Waypoint(const json& j) : reference(j), 
                            subtasks(j.value("subtasks", json::array())),
                            parallel_execution(j.value("parallel_execution", false))
                          {}

  // Helper method to get typed subtasks
  std::vector<Subtask> getSubtasks() const {
    std::vector<Subtask> result;
    for (const auto& subtask : subtasks) {
      result.emplace_back(subtask);
    }
    return result;
  }
};

} // namespace custom_types
} // namespace iroc_fleet_manager
