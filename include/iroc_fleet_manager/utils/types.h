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

  Subtask() : type(), parameters() {
  }
  Subtask(const json& j) : type(j.at("type")), parameters(j.at("parameters").dump()) {
  }
};

struct Waypoint {
  Reference reference;
  json subtasks;
  Waypoint() : reference(), subtasks(json::array()) {
  }
  Waypoint(const json& j) : reference(j), subtasks(j.value("subtasks", json::array())) {
  }

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
