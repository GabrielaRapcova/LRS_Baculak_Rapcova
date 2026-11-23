#include "drone_localization/localizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace drone_localization
{

namespace
{

std::string format(const char * fmt, ...) __attribute__((format(printf, 1, 2)));

std::string format(const char * fmt, ...)
{
  char buf[512];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  return std::string(buf);
}

double median(std::vector<double> v)
{
  std::sort(v.begin(), v.end());
  const std::size_t n = v.size();
  return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

}  // namespace

Localizer::Localizer(const LocalizerConfig & config)
: config_(config), ekf_(config.noise) {}

void Localizer::set_config(const LocalizerConfig & config)
{
  config_ = config;
  ekf_.set_process_noise(config.noise);
}

void Localizer::log(LogLevel level, const std::string & msg) const
{
  if (log_) {log_(level, msg);}
}

void Localizer::on_imu(double t, double gyro_z)
{
  ++imu_.received;
  if (!ekf_.initialized()) {
    // Before initialisation the drone is on the ground: the gyro mean is the bias.
    gyro_sum_ += gyro_z;
    gyro_sq_sum_ += gyro_z * gyro_z;
    ++gyro_n_;
    last_gyro_ = gyro_z;
    have_gyro_ = true;
    imu_.last_stamp = t;
    return;
  }

  if (!std::isfinite(t) || !std::isfinite(gyro_z)) {return;}
  if (imu_link_.lost) {link_alive(imu_link_, "IMU", t);}

  if (t <= ekf_.time()) {
    // Gyro sample at or before the filter time (a measurement already moved the
    // filter past it). Keep the rate for the next interval but do not integrate twice.
    if (t < ekf_.time() - config_.max_measurement_lag) {++imu_.stale;} else {++imu_.late;}
    last_gyro_ = gyro_z;
    return;
  }

  // A gyro sample stamped t describes the rate over the interval ending at t.
  if (t > gyro_integral_stamp_) {
    gyro_integral_ += gyro_z * (t - gyro_integral_stamp_);
  }
  gyro_integral_stamp_ = t;
  if (!ekf_.predict(t, gyro_z)) {note_numerical_failure("prediction", t);}
  last_gyro_ = gyro_z;
  have_gyro_ = true;
  imu_.last_stamp = t;
  ++imu_.accepted;
  if (config_.zaru_enabled || config_.zupt_enabled) {zaru(t, gyro_z);}
  check_timeouts(t);
}

void Localizer::note_numerical_failure(const char * what, double t)
{
  ++numerical_failures_;
  log(LogLevel::Error, format(
    "%s at t=%.2f produced a non-finite or non-positive-definite result; "
    "kept the previous valid state (failure #%zu)", what, t, numerical_failures_));
}

void Localizer::check_timeouts(double t)
{
  const auto sd = ekf_.P().diagonal().cwiseSqrt();
  if (!gps_link_.lost && gps_last_data_ > 0.0 && t - gps_last_data_ > config_.gps_timeout) {
    gps_link_.lost = true;
    gps_link_.lost_since = gps_last_data_;
    ++gps_link_.outages;
    log(LogLevel::Warn, format(
      "GPS LOST at t=%.2f: no fix for %.1f s. Continuing on VO velocity + motion model "
      "(position sigma %.2f, %.2f m)", t, t - gps_last_data_, sd(PX), sd(PY)));
  }
  if (!vo_link_.lost && vo_last_data_ > 0.0 && t - vo_last_data_ > config_.vo_timeout) {
    vo_link_.lost = true;
    vo_link_.lost_since = vo_last_data_;
    ++vo_link_.outages;
    have_vo_anchor_ = false;  // restart the relative yaw baseline after the gap
    log(LogLevel::Warn, format(
      "VO LOST at t=%.2f: no frame for %.2f s. Continuing on GPS + motion model "
      "(velocity sigma %.2f m/s)", t, t - vo_last_data_, sd(VX)));
  }
}

double Localizer::link_alive(LinkState & link, const char * name, double t)
{
  if (!link.lost) {return 0.0;}
  link.lost = false;
  const double gap = t - link.lost_since;
  const auto sd = ekf_.P().diagonal().cwiseSqrt();
  log(LogLevel::Info, format(
    "%s RECOVERED at t=%.2f after %.1f s (position sigma %.2f, %.2f m, velocity sigma %.2f m/s)",
    name, t, gap, sd(PX), sd(PY), sd(VX)));
  return gap;
}

void Localizer::zaru(double t, double gyro_z)
{
  const auto & x = ekf_.x();
  const double height = x(PZ) - ground_height_;
  const double speed = x.segment<3>(VX).norm();

  // Resting detector with hysteresis.
  bool resting = resting_;
  if (resting_) {
    if (height > config_.rest_exit_height || speed > config_.rest_exit_speed) {resting = false;}
  } else if (height < config_.rest_enter_height && speed < config_.rest_enter_speed) {
    if (rest_candidate_since_ < 0.0) {rest_candidate_since_ = t;}
    resting = t - rest_candidate_since_ >= config_.rest_enter_time;
  } else {
    rest_candidate_since_ = -1.0;
  }
  if (resting != resting_) {
    resting_ = resting;
    rest_candidate_since_ = -1.0;
    log(LogLevel::Info, format(
      "t=%.2f %s (height %.2f m above ground estimate, speed %.2f m/s): "
      "zero-rate/zero-velocity updates %s", t,
      resting ? "RESTING on the ground" : "MOVING / airborne", height, speed,
      resting ? "on" : "off"));
  }
  if (!resting_) {
    zaru_n_ = 0;
    zaru_sum_ = 0.0;
    return;
  }
  // While resting, follow the converged height so a poor initial GPS height
  // does not bias the detector.
  ground_height_ = x(PZ);

  if (zaru_n_ == 0) {zaru_start_ = t;}
  zaru_sum_ += gyro_z;
  ++zaru_n_;
  if (t - zaru_start_ < config_.zaru_window) {return;}

  if (config_.zupt_enabled) {
    Eigen::Matrix<double, 3, kStateDim> H = Eigen::Matrix<double, 3, kStateDim>::Zero();
    H.block<3, 3>(0, VX) = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d innovation = -x.segment<3>(VX);
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * config_.zupt_sigma * config_.zupt_sigma;
    const UpdateResult r = ekf_.update<3>(innovation, H, R, config_.zupt_gate_chi2);
    ++zupt_.received;
    zupt_.last_nis = r.nis;
    zupt_.last_stamp = t;
    if (r.accepted) {
      ++zupt_.accepted;
    } else {
      // VO/GPS disagree with "standing still": we are taking off.
      ++zupt_.rejected;
      resting_ = false;
      log(LogLevel::Info, format(
        "t=%.2f zero-velocity update rejected (NIS %.1f): MOVING / airborne, "
        "zero-rate/zero-velocity updates off", t, r.nis));
      zaru_n_ = 0;
      zaru_sum_ = 0.0;
      return;
    }
  }

  if (config_.zaru_enabled) {
    const double n = static_cast<double>(zaru_n_);
    const double mean = zaru_sum_ / n;
    const double var = config_.noise.gyro_noise * config_.noise.gyro_noise / n +
      config_.zaru_rate_sigma * config_.zaru_rate_sigma;
    ++zaru_.received;
    const UpdateResult r = ekf_.update_bias(mean, var, config_.vo_yaw_gate_chi2);
    zaru_.last_nis = r.nis;
    zaru_.last_stamp = t;
    r.accepted ? ++zaru_.accepted : ++zaru_.rejected;
  }
  zaru_n_ = 0;
  zaru_sum_ = 0.0;
}

bool Localizer::align_to(double t, SensorStats & stats, const char * sensor)
{
  const double now = ekf_.time();
  if (!imu_link_.lost && have_gyro_ && t - imu_.last_stamp > config_.imu_timeout) {
    imu_link_.lost = true;
    imu_link_.lost_since = imu_.last_stamp;
    ++imu_link_.outages;
    log(LogLevel::Warn, format(
      "IMU LOST at t=%.2f: no gyro for %.2f s. Predicting with the last rate %.4f rad/s",
      t, t - imu_.last_stamp, last_gyro_));
  }
  if (t >= now) {
    if (t > now && !ekf_.predict(t, have_gyro_ ? last_gyro_ : ekf_.x()(BIAS))) {
      note_numerical_failure("prediction", t);
      return false;
    }
    return true;
  }
  if (t >= now - config_.max_measurement_lag) {
    ++stats.late;
    return true;
  }
  ++stats.stale;
  log(LogLevel::Warn, format("%s measurement %.3f s older than the filter, dropped",
    sensor, now - t));
  return false;
}

Eigen::Matrix3d Localizer::gps_noise(const Eigen::Matrix3d & cov) const
{
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  const bool usable = config_.gps_use_message_covariance &&
    cov.diagonal().minCoeff() > 0.0;
  if (usable) {
    R = cov;
  } else {
    for (int i = 0; i < 3; ++i) {R(i, i) = config_.gps_sigma[i] * config_.gps_sigma[i];}
  }
  return R * config_.gps_sigma_scale * config_.gps_sigma_scale;
}

void Localizer::on_gps(double t, const Eigen::Vector3d & position, const Eigen::Matrix3d & cov)
{
  if (!std::isfinite(t) || !position.allFinite() || !cov.allFinite()) {
    log(LogLevel::Warn, "GPS message with non-finite values ignored");
    return;
  }
  ++gps_.received;
  if (!ekf_.initialized()) {
    init_gps_.push_back(position);
    init_gps_cov_.push_back(gps_noise(cov));
    init_gps_stamp_ = t;
    gps_.last_stamp = t;
    gps_last_data_ = t;
    try_initialize();
    return;
  }

  if (!align_to(t, gps_, "GPS")) {return;}
  gps_last_data_ = std::max(gps_last_data_, t);
  const bool after_outage = link_alive(gps_link_, "GPS", t) > 0.0;
  const Eigen::Vector3d d = position - ekf_.x().segment<3>(PX);
  const UpdateResult r = ekf_.update_position(position, gps_noise(cov), config_.gps_gate_chi2);
  gps_.last_nis = r.nis;
  gps_.last_stamp = t;
  if (r.numerical_failure) {
    note_numerical_failure("GPS update", t);
    return;
  }
  if (r.accepted) {
    ++gps_.accepted;
    gps_consecutive_rejections_ = 0;
    if (after_outage) {
      log(LogLevel::Info, format(
        "GPS re-acquired: first fix innovation (%.2f, %.2f, %.2f) m, NIS %.1f, accepted; "
        "correction applied through the Kalman gain (no reset)", d.x(), d.y(), d.z(), r.nis));
    }
    return;
  }
  ++gps_.rejected;
  ++gps_consecutive_rejections_;
  log(LogLevel::Warn, format(
    "GPS REJECTED at t=%.2f: NIS %.1f > gate %.2f (chi2, 3 DOF), innovation "
    "(%.2f, %.2f, %.2f) m = %.2f m%s", t, r.nis, config_.gps_gate_chi2,
    d.x(), d.y(), d.z(), d.norm(), after_outage ? " [first fix after outage]" : ""));
  if (config_.gps_max_consecutive_rejections > 0 &&
    gps_consecutive_rejections_ >= config_.gps_max_consecutive_rejections)
  {
    // Persistent disagreement: the estimate has drifted, not GPS. Inflate the
    // position uncertainty to the size of the disagreement so the next fixes
    // pass the gate and pull the state back gradually.
    for (int i = 0; i < 3; ++i) {ekf_.inflate(PX + i, 1, d(i) * d(i));}
    log(LogLevel::Warn, format(
      "%d consecutive GPS rejections: assuming the estimate drifted; inflated position "
      "covariance to (%.2f, %.2f, %.2f) m to re-acquire GPS", gps_consecutive_rejections_,
      std::sqrt(ekf_.P()(PX, PX)), std::sqrt(ekf_.P()(PY, PY)), std::sqrt(ekf_.P()(PZ, PZ))));
    gps_consecutive_rejections_ = 0;
  }
}

double Localizer::gyro_integral_at(double t) const
{
  return gyro_integral_ + last_gyro_ * (t - gyro_integral_stamp_);
}

void Localizer::on_vo(
  double t, double vo_yaw, const Eigen::Vector3d & body_velocity,
  const Eigen::Vector3d & velocity_var)
{
  if (!std::isfinite(t) || !std::isfinite(vo_yaw) || !body_velocity.allFinite()) {
    log(LogLevel::Warn, "VO message with non-finite values ignored");
    return;
  }
  ++vo_vel_.received;
  ++vo_yaw_.received;
  if (!ekf_.initialized()) {return;}
  if (!align_to(t, vo_vel_, "VO")) {return;}
  vo_last_data_ = std::max(vo_last_data_, t);
  link_alive(vo_link_, "VO", t);

  if (config_.vo_velocity_enabled) {
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    const double floor = config_.vo_velocity_sigma * config_.vo_velocity_sigma;
    for (int i = 0; i < 3; ++i) {R(i, i) = std::max(velocity_var(i), floor);}
    const UpdateResult r = ekf_.update_body_velocity(
      body_velocity, R, config_.vo_velocity_gate_chi2);
    vo_vel_.last_nis = r.nis;
    vo_vel_.last_stamp = t;
    if (r.numerical_failure) {
      note_numerical_failure("VO velocity update", t);
    } else if (r.accepted) {
      ++vo_vel_.accepted;
    } else {
      ++vo_vel_.rejected;
      // Throttled by count: log the first and then every 30th rejection.
      if (vo_vel_.rejected == 1 || vo_vel_.rejected % 30 == 0) {
        log(LogLevel::Warn, format(
          "VO velocity rejected at t=%.2f: NIS %.1f > %.2f (%zu rejected so far)",
          t, r.nis, config_.vo_velocity_gate_chi2, vo_vel_.rejected));
      }
    }
  }
  vo_yaw_updates(t, vo_yaw);
}

void Localizer::vo_yaw_updates(double t, double vo_yaw)
{
  vo_yaw_.last_stamp = t;

  if (config_.vo_yaw_absolute && t - last_abs_yaw_t_ >= config_.vo_yaw_period) {
    last_abs_yaw_t_ = t;
    const UpdateResult r = ekf_.update_yaw(
      vo_yaw, config_.vo_yaw_sigma * config_.vo_yaw_sigma, config_.vo_yaw_gate_chi2);
    vo_yaw_.last_nis = r.nis;
    r.accepted ? ++vo_yaw_.accepted : ++vo_yaw_.rejected;
  }

  if (!config_.vo_yaw_relative) {return;}
  const double g = gyro_integral_at(t);
  if (!have_vo_anchor_) {
    have_vo_anchor_ = true;
    vo_anchor_t_ = t;
    vo_anchor_yaw_ = vo_yaw;
    vo_anchor_gyro_ = g;
    return;
  }
  const double dt = t - vo_anchor_t_;
  if (dt < config_.vo_yaw_window) {return;}

  // Over [anchor, t]: gyro turned (g - g0) = (true turn + b*dt); VO saw the
  // true turn (plus its small drift), so their difference per second is b.
  const double vo_turn = wrap_angle(vo_yaw - vo_anchor_yaw_);
  const double gyro_turn = g - vo_anchor_gyro_;
  const double bias_meas = (gyro_turn - vo_turn) / dt;
  const double endpoint = std::sqrt(2.0) * config_.vo_pose_yaw_sigma / dt;
  const double gyro_noise = config_.noise.gyro_noise *
    std::sqrt(dt / config_.noise.gyro_rate) / dt;
  const double var = endpoint * endpoint + gyro_noise * gyro_noise +
    config_.vo_yaw_rate_sigma * config_.vo_yaw_rate_sigma;

  const UpdateResult r = ekf_.update_bias(bias_meas, var, config_.vo_yaw_gate_chi2);
  vo_yaw_.last_nis = r.nis;
  if (r.accepted) {
    ++vo_yaw_.accepted;
  } else {
    ++vo_yaw_.rejected;
    log(LogLevel::Warn, format(
      "VO yaw-rate bias measurement rejected at t=%.2f: %.4f rad/s, NIS %.1f",
      t, bias_meas, r.nis));
  }
  vo_anchor_t_ = t;
  vo_anchor_yaw_ = vo_yaw;
  vo_anchor_gyro_ = g;
}

void Localizer::try_initialize()
{
  const int n = static_cast<int>(init_gps_.size());
  if (n < config_.init_gps_samples) {return;}

  // Component-wise median: robust to a single injected GPS outlier.
  Eigen::Vector3d p0;
  for (int axis = 0; axis < 3; ++axis) {
    std::vector<double> v;
    for (const auto & g : init_gps_) {v.push_back(g(axis));}
    p0(axis) = median(v);
  }
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  for (const auto & c : init_gps_cov_) {R += c;}
  R /= n;
  // Variance of the median of n Gaussian samples is about (pi/2) sigma^2 / n.
  const Eigen::Matrix3d P_pos = R * (M_PI / 2.0) / n;

  double bias0 = 0.0;
  double bias_var = config_.init_bias_sigma * config_.init_bias_sigma;
  if (config_.init_bias_from_gyro && gyro_n_ > 10) {
    bias0 = gyro_sum_ / static_cast<double>(gyro_n_);
    const double var = std::max(
      gyro_sq_sum_ / static_cast<double>(gyro_n_) - bias0 * bias0, 0.0);
    // Standard error of the mean, floored so the filter can still refine it.
    bias_var = std::max(var / static_cast<double>(gyro_n_),
      config_.init_bias_min_sigma * config_.init_bias_min_sigma);
  }

  StateVec x0 = StateVec::Zero();
  x0.segment<3>(PX) = p0;
  x0(YAW) = config_.init_yaw;
  x0(BIAS) = bias0;

  StateMat P0 = StateMat::Zero();
  P0.block<3, 3>(PX, PX) = P_pos;
  for (int i = VX; i <= VZ; ++i) {
    P0(i, i) = config_.init_velocity_sigma * config_.init_velocity_sigma;
  }
  P0(YAW, YAW) = config_.init_yaw_sigma * config_.init_yaw_sigma;
  P0(BIAS, BIAS) = bias_var;

  ekf_.initialize(init_gps_stamp_, x0, P0);
  ground_height_ = p0.z();
  // The gyro integral (for the relative VO yaw measurement) starts at the
  // filter time, never at stamp 0.
  gyro_integral_ = 0.0;
  gyro_integral_stamp_ = init_gps_stamp_;
  have_vo_anchor_ = false;
  log(LogLevel::Info, format(
    "initialised at t=%.2f from %d GPS fixes: p=(%.2f, %.2f, %.2f) m (sigma %.2f m), "
    "yaw=%.1f deg, gyro bias=%+.4f rad/s from %zu samples",
    init_gps_stamp_, n, p0.x(), p0.y(), p0.z(), std::sqrt(P_pos(0, 0)),
    config_.init_yaw * 180.0 / M_PI, bias0, gyro_n_));

  init_gps_.clear();
  init_gps_cov_.clear();
}

}  // namespace drone_localization
