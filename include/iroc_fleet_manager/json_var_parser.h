#pragma once

#include <nlohmann/json.hpp>
#include <variant>

namespace iroc_fleet_manager {

using json = nlohmann::json;

struct Point2D {
  double x, y;

  Point2D() : x(0), y(0) {}
  Point2D(const json &j) : x(j.at("x")), y(j.at("y")) {}
};

struct Point3D {
  double x, y, z;

  Point3D() : x(0), y(0), z(0) {}
  Point3D(const json &j) : x(j.at("x")), y(j.at("y")), z(j.at("z")) {}
};

struct Reference {
  double x, y, z, heading;

  Reference() : x(0), y(0), z(0), heading(0) {}
  Reference(const json &j)
      : x(j.at("x")), y(j.at("y")), z(j.at("z")), heading(j.at("heading")) {}
};

struct Waypoint {
  double x, y, z, heading;
  json subtasks;

  Waypoint() : x(0), y(0), z(0), heading(0), subtasks() {}
  Waypoint(const json &j)
      : x(j.at("x")), y(j.at("y")), z(j.at("z")), heading(j.at("heading")),
        subtasks(j.value("subtasks", json::array())) {}
};

using parseable_t =
    std::variant<json *, bool *, int *, double *, std::string *,
                 std::vector<int> *, std::vector<double> *,
                 std::vector<std::string> *, Point2D *, Point3D *, Reference *,
                 Waypoint *, std::vector<Point2D> *, std::vector<Point3D> *,
                 std::vector<Reference> *, std::vector<Waypoint> *>;

bool parseVars(const json &js,
               std::vector<std::pair<std::string_view, parseable_t>> &&vars);

// Helper functions
template <typename Msg_T> Msg_T toRosMsg(const Point2D &point) {
  Msg_T msg_point;
  msg_point.x = point.x;
  msg_point.y = point.y;
  return msg_point;
}

// 3D Point conversion
template <typename Msg_T> Msg_T toRosMsg(const Point3D &point) {
  Msg_T msg_point;
  msg_point.x = point.x;
  msg_point.y = point.y;
  msg_point.z = point.z;
  return msg_point;
}

// Reference conversion
template <typename Msg_T> Msg_T toRosMsg(const Reference &point) {
  Msg_T msg_point;
  msg_point.x = point.x;
  msg_point.y = point.y;
  msg_point.z = point.z;
  msg_point.heading = point.heading;
  return msg_point;
}

// Convert entire vectors
template <typename Msg_T, typename Point_T>
std::vector<Msg_T> toRosMsgVector(const std::vector<Point_T> &points) {
  std::vector<Msg_T> result;
  result.reserve(points.size());

  for (const auto &point : points) {
    result.emplace_back(toRosMsg<Msg_T>(point));
  }

  return result;
}

} // namespace iroc_fleet_manager
