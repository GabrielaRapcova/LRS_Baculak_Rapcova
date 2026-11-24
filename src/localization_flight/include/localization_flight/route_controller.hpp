// Route following on the *estimated* pose (no ROS).
//
// RouteController turns the estimated position/yaw into an ENU velocity and a
// yaw-rate command toward a target (saturated P control). Mission keeps the
// list of absolute targets built from the relative route and the captured
// origin, and advances when the estimate is within the acceptance radius.
#pragma once

#include <algorithm>
#include <cstddef>
#include <utility>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace localization_flight
{

struct ControllerParams
{
  double kp_xy = 0.8;          // [1/s]
  double max_speed_xy = 0.5;   // [m/s] cruise speed (spec: ~0.5 m/s)
  double kp_z = 0.8;           // [1/s]
  double max_speed_z = 0.4;    // [m/s]
  double kp_yaw = 1.0;         // [1/s]
  double max_yaw_rate = 0.3;   // [rad/s]
};

struct VelocityCommand
{
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // ENU [m/s]
  double yaw_rate = 0.0;                               // ENU, CCW positive [rad/s]
};

class RouteController
{
public:
  explicit RouteController(const ControllerParams & p = ControllerParams{})
  : p_(p) {}

  // Velocity toward `target` (straight line, slowing down inside kp/max radius)
  // and yaw rate toward `yaw_ref`.
  VelocityCommand compute(
    const Eigen::Vector3d & position, double yaw,
    const Eigen::Vector3d & target, double yaw_ref) const;

  const ControllerParams & params() const {return p_;}

private:
  ControllerParams p_;
};

struct Waypoint
{
  Eigen::Vector3d position;   // absolute ENU (map)
  std::string label;
};

// Relative ENU offsets (x, y) of the §4 route; flown `laps` times at `altitude`
// above the origin, starting and ending above the origin.
std::vector<Waypoint> build_route(
  const Eigen::Vector3d & origin, double altitude, int laps,
  const std::vector<Eigen::Vector2d> & offsets);

class Mission
{
public:
  Mission(std::vector<Waypoint> route, double accept_xy, double accept_z)
  : route_(std::move(route)), accept_xy_(accept_xy), accept_z_(accept_z) {}

  // True if the current waypoint was reached by this update (index advanced).
  bool update(const Eigen::Vector3d & position);

  bool done() const {return index_ >= route_.size();}
  std::size_t index() const {return index_;}  // number of waypoints reached
  std::size_t size() const {return route_.size();}
  // Waypoint being flown to (the last one once the mission is done).
  const Waypoint & current() const {return route_[std::min(index_, route_.size() - 1)];}
  // Waypoint reached most recently (requires index() > 0).
  const Waypoint & previous() const {return route_[index_ - 1];}

private:
  std::vector<Waypoint> route_;
  double accept_xy_;
  double accept_z_;
  std::size_t index_ = 0;
};

}  // namespace localization_flight
