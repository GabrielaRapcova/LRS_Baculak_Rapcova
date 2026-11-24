// Closed-loop check without ROS/Gazebo: the real Localizer (drone_localization)
// estimates from simulated sensors, and the real RouteController/Mission fly a
// simple vehicle model *using only the estimate*. Truth is used for scoring.
//
// Vehicle: velocity loop with first-order lag (ArduCopter GUIDED velocity
// control), yaw-rate loop with lag. Sensors reproduce the characterised
// corruption: GPS white noise (0.6/0.6/0.9 m), VO body velocity with scale,
// bias and noise, VO yaw with a slow drift, gyro Z with a constant bias.

#include <gtest/gtest.h>

#include <cmath>
#include <algorithm>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <angles/angles.h>

#include "drone_localization/localizer.hpp"
#include "localization_flight/route_controller.hpp"

namespace dl = drone_localization;
namespace lf = localization_flight;

namespace
{

struct Result
{
  bool completed = false;
  double route_time = 0.0;        // CLIMB done -> last waypoint
  double max_cross_track = 0.0;   // truth distance from the route polyline
  double final_error = 0.0;       // truth distance from origin after the hold
  double max_est_error = 0.0;
  double max_alt_error = 0.0;
  std::size_t gps_rejected = 0;
};

// Distance from point p to the segment a-b.
double segment_distance(
  const Eigen::Vector2d & p, const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  const Eigen::Vector2d ab = b - a;
  const double t = std::clamp((p - a).dot(ab) / std::max(ab.squaredNorm(), 1e-9), 0.0, 1.0);
  return (a + t * ab - p).norm();
}

Result fly(unsigned seed, double gps_off_from, double gps_off_to, bool outlier_after_outage)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> n(0.0, 1.0);
  const double gyro_bias = (seed % 2 ? -1.0 : 1.0) * (0.02 + 0.004 * n(rng));
  const double vo_scale = 1.0 + 0.01 * n(rng);
  const Eigen::Vector3d vo_bias(0.015 * n(rng), 0.015 * n(rng), 0.01 * n(rng));
  const double vo_yaw_drift = 0.0015 * n(rng);

  dl::LocalizerConfig cfg;  // package defaults
  dl::Localizer loc(cfg);

  lf::RouteController ctl;
  const std::vector<Eigen::Vector2d> offsets = {{0, 0}, {0, -4}, {2.5, -4}, {2.5, 0}, {0, 0}};

  // Truth state.
  Eigen::Vector3d p(-7.0, 13.0, 0.0), v = Eigen::Vector3d::Zero(), v_cmd = Eigen::Vector3d::Zero();
  double yaw = M_PI / 2.0, wz = 0.0, wz_cmd = 0.0, vo_yaw = yaw;
  const Eigen::Matrix3d gps_cov = Eigen::Vector3d(0.36, 0.36, 0.81).asDiagonal();

  enum { WAIT, CLIMB, ROUTE, HOLD, DONE } phase = WAIT;
  Eigen::Vector3d origin;
  double origin_yaw = 0.0, phase_t = 0.0, route_start = 0.0;
  std::optional<lf::Mission> mission;
  Result res;
  bool outlier_pending = outlier_after_outage;

  const double dt = 0.01;
  for (int i = 1; i <= 30000 && phase != DONE; ++i) {  // up to 300 s
    const double t = 100.0 + i * dt;

    // Vehicle dynamics (truth).
    const double tau = 0.6;
    v += (v_cmd - v) * (dt / (tau + dt));
    wz += (wz_cmd - wz) * (dt / (0.3 + dt));
    p += v * dt;
    p.z() = std::max(p.z(), 0.0);
    if (p.z() == 0.0) {v.z() = std::max(v.z(), 0.0);}
    yaw = angles::normalize_angle(yaw + wz * dt);
    vo_yaw = angles::normalize_angle(vo_yaw + wz * dt + vo_yaw_drift * dt);

    // Sensors.
    loc.on_imu(t, wz + gyro_bias + 0.004 * n(rng));
    const bool gps_on = !(t >= gps_off_from && t < gps_off_to);
    if (i % 20 == 0 && gps_on) {
      Eigen::Vector3d z = p + Eigen::Vector3d(0.6 * n(rng), 0.6 * n(rng), 0.9 * n(rng));
      if (outlier_pending && t >= gps_off_to) {
        z += Eigen::Vector3d(3.0, -4.0, 2.0);  // ~5.4 m
        outlier_pending = false;
      }
      loc.on_gps(t, z, gps_cov);
    }
    if (i % 3 == 0) {
      const double c = std::cos(yaw), s = std::sin(yaw);
      const Eigen::Vector3d vb(c * v.x() + s * v.y(), -s * v.x() + c * v.y(), v.z());
      loc.on_vo(t, angles::normalize_angle(vo_yaw + 0.005 * n(rng)),
        vo_scale * vb + vo_bias + 0.03 * Eigen::Vector3d(n(rng), n(rng), n(rng)),
        Eigen::Vector3d::Constant(0.0009));
    }
    if (!loc.initialized()) {continue;}

    const Eigen::Vector3d est = loc.ekf().x().head<3>();
    const double est_yaw = loc.ekf().x()(dl::YAW);
    res.max_est_error = std::max(res.max_est_error, (est - p).head<2>().norm());

    // Controller at 20 Hz, on the estimate only.
    if (i % 5 != 0) {continue;}
    lf::VelocityCommand cmd;
    switch (phase) {
      case WAIT:
        if (std::sqrt(loc.ekf().P()(dl::PX, dl::PX)) < 0.3 && t > 120.0) {
          origin = est;
          origin_yaw = est_yaw;
          mission.emplace(lf::build_route(origin, 2.0, 2, offsets), 0.3, 0.35);
          phase = CLIMB;
        }
        break;
      case CLIMB:
        {
          const Eigen::Vector3d target = origin + Eigen::Vector3d(0, 0, 2.0);
          cmd = ctl.compute(est, est_yaw, target, origin_yaw);
          if (std::abs(est.z() - target.z()) < 0.25) {phase = ROUTE; route_start = t;}
        }
        break;
      case ROUTE:
        mission->update(est);
        if (mission->done()) {
          phase = HOLD;
          phase_t = t;
          res.route_time = t - route_start;
          res.completed = true;
          break;
        }
        cmd = ctl.compute(est, est_yaw, mission->current().position, origin_yaw);
        {
          double d = 1e9;
          const Eigen::Vector2d q = (p - origin).head<2>();
          for (std::size_t k = 0; k + 1 < offsets.size(); ++k) {
            d = std::min(d, segment_distance(q, offsets[k], offsets[k + 1]));
          }
          res.max_cross_track = std::max(res.max_cross_track, d);
          res.max_alt_error = std::max(res.max_alt_error, std::abs(p.z() - origin.z() - 2.0));
        }
        break;
      case HOLD:
        cmd = ctl.compute(est, est_yaw, origin + Eigen::Vector3d(0, 0, 2.0), origin_yaw);
        if (t - phase_t > 3.0) {
          res.final_error = (p - origin).head<2>().norm();
          phase = DONE;
        }
        break;
      case DONE:
        break;
    }
    v_cmd = cmd.velocity;
    wz_cmd = cmd.yaw_rate;
  }
  res.gps_rejected = loc.gps_stats().rejected;
  return res;
}

}  // namespace

TEST(RouteController, SaturatesAndPointsAtTarget)
{
  lf::RouteController c;
  const auto cmd = c.compute({0, 0, 2}, 0.0, {0, -4, 2}, 0.0);
  EXPECT_NEAR(cmd.velocity.head<2>().norm(), 0.5, 1e-9);  // cruise speed
  EXPECT_NEAR(cmd.velocity.y(), -0.5, 1e-9);
  EXPECT_NEAR(cmd.velocity.z(), 0.0, 1e-9);
  const auto near = c.compute({0, -3.9, 2}, 0.0, {0, -4, 2}, 0.0);
  EXPECT_NEAR(near.velocity.y(), -0.08, 1e-9);  // slows down: kp * 0.1 m
  const auto yaw = c.compute({0, 0, 2}, M_PI - 0.1, {0, 0, 2}, -M_PI + 0.1);
  EXPECT_GT(yaw.yaw_rate, 0.0);   // shortest way across +-pi
  EXPECT_LE(yaw.yaw_rate, 0.3);
}

TEST(Route, TwoLapsTenWaypointsWithoutDuplicates)
{
  const std::vector<Eigen::Vector2d> offsets = {{0, 0}, {0, -4}, {2.5, -4}, {2.5, 0}, {0, 0}};
  const auto r = lf::build_route({-7, 13, 0.1}, 2.0, 2, offsets);
  ASSERT_EQ(r.size(), 9u);  // 5 + 4 (the shared start point is not repeated)
  EXPECT_TRUE(r.front().position.isApprox(Eigen::Vector3d(-7, 13, 2.1)));
  EXPECT_TRUE(r[1].position.isApprox(Eigen::Vector3d(-7, 9, 2.1)));
  EXPECT_TRUE(r.back().position.isApprox(Eigen::Vector3d(-7, 13, 2.1)));
}

TEST(ClosedLoop, TwoLapsOnEstimateWithFifteenSecondGpsOutage)
{
  for (unsigned seed : {1u, 2u, 3u, 4u, 5u, 6u}) {
    // GPS off for 15 s starting 20 s into the route (a middle leg of lap 1/2),
    // with an outlier on the first fix after the outage.
    const Result r = fly(seed, 145.0, 160.0, true);
    SCOPED_TRACE("seed " + std::to_string(seed));
    EXPECT_TRUE(r.completed);
    EXPECT_GT(r.route_time, 45.0);
    EXPECT_LT(r.route_time, 110.0);
    EXPECT_LT(r.max_cross_track, 0.9);
    EXPECT_LT(r.max_alt_error, 0.6);
    EXPECT_LT(r.final_error, 0.6);
    EXPECT_LT(r.max_est_error, 0.8);
    EXPECT_GE(r.gps_rejected, 1u);  // the outlier
    std::printf("seed %u: route %.1f s, max cross-track %.2f m, max alt err %.2f m, "
      "final %.2f m, max est err %.2f m, GPS rejected %zu\n", seed, r.route_time,
      r.max_cross_track, r.max_alt_error, r.final_error, r.max_est_error, r.gps_rejected);
  }
}
