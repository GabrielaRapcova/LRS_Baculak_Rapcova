#include "localization_flight/route_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <angles/angles.h>

namespace localization_flight
{

VelocityCommand RouteController::compute(
  const Eigen::Vector3d & position, double yaw,
  const Eigen::Vector3d & target, double yaw_ref) const
{
  VelocityCommand cmd;
  const Eigen::Vector2d e_xy = (target - position).head<2>();
  const double dist = e_xy.norm();
  if (dist > 1e-6) {
    const double speed = std::min(p_.max_speed_xy, p_.kp_xy * dist);
    cmd.velocity.head<2>() = e_xy / dist * speed;
  }
  cmd.velocity.z() = std::clamp(p_.kp_z * (target.z() - position.z()),
      -p_.max_speed_z, p_.max_speed_z);
  cmd.yaw_rate = std::clamp(p_.kp_yaw * angles::shortest_angular_distance(yaw, yaw_ref),
      -p_.max_yaw_rate, p_.max_yaw_rate);
  return cmd;
}

std::vector<Waypoint> build_route(
  const Eigen::Vector3d & origin, double altitude, int laps,
  const std::vector<Eigen::Vector2d> & offsets)
{
  std::vector<Waypoint> route;
  for (int lap = 1; lap <= laps; ++lap) {
    for (std::size_t i = 0; i < offsets.size(); ++i) {
      // The first point of laps 2.. equals the last point of the previous lap.
      if (lap > 1 && i == 0) {continue;}
      char label[64];
      std::snprintf(label, sizeof(label), "lap %d wp %zu (%+.1f, %+.1f)",
        lap, i, offsets[i].x(), offsets[i].y());
      route.push_back({origin + Eigen::Vector3d(offsets[i].x(), offsets[i].y(), altitude), label});
    }
  }
  return route;
}

bool Mission::update(const Eigen::Vector3d & position)
{
  if (done()) {return false;}
  const Eigen::Vector3d e = route_[index_].position - position;
  if (e.head<2>().norm() <= accept_xy_ && std::abs(e.z()) <= accept_z_) {
    ++index_;
    return true;
  }
  return false;
}

}  // namespace localization_flight
