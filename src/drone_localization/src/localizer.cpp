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

  if (t <= ekf_.time()) {
    // Gyro sample at or before the filter time (a measurement already moved the
    // filter past it). Keep the rate for the next interval but do not integrate twice.
    if (t < ekf_.time() - config_.max_measurement_lag) {++imu_.stale;} else {++imu_.late;}
    last_gyro_ = gyro_z;
    return;
  }

  // A gyro sample stamped t describes the rate over the interval ending at t.
  if (t > gyro_integral_stamp_ && gyro_integral_stamp_ > 0.0) {
    gyro_integral_ += gyro_z * (t - gyro_integral_stamp_);
  }
  gyro_integral_stamp_ = t;
  ekf_.predict(t, gyro_z);
  last_gyro_ = gyro_z;
  have_gyro_ = true;
  imu_.last_stamp = t;
  ++imu_.accepted;
}

bool Localizer::align_to(double t, SensorStats & stats, const char * sensor)
{
  const double now = ekf_.time();
  if (t >= now) {
    ekf_.predict(t, have_gyro_ ? last_gyro_ : ekf_.x()(BIAS));  // no-op if t == now
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
  ++gps_.received;
  if (!ekf_.initialized()) {
    init_gps_.push_back(position);
    init_gps_cov_.push_back(gps_noise(cov));
    init_gps_stamp_ = t;
    gps_.last_stamp = t;
    try_initialize();
    return;
  }

  if (!align_to(t, gps_, "GPS")) {return;}
  const UpdateResult r = ekf_.update_position(position, gps_noise(cov), config_.gps_gate_chi2);
  gps_.last_nis = r.nis;
  gps_.last_stamp = t;
  if (r.accepted) {
    ++gps_.accepted;
  } else {
    ++gps_.rejected;
    const Eigen::Vector3d d = position - ekf_.x().segment<3>(PX);
    log(LogLevel::Warn, format(
      "GPS rejected at t=%.2f: NIS %.1f > %.1f, innovation (%.2f, %.2f, %.2f) m",
      t, r.nis, config_.gps_gate_chi2, d.x(), d.y(), d.z()));
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
  ++vo_vel_.received;
  ++vo_yaw_.received;
  if (!ekf_.initialized()) {return;}
  if (!align_to(t, vo_vel_, "VO")) {return;}

  if (config_.vo_velocity_enabled) {
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    const double floor = config_.vo_velocity_sigma * config_.vo_velocity_sigma;
    for (int i = 0; i < 3; ++i) {R(i, i) = std::max(velocity_var(i), floor);}
    const UpdateResult r = ekf_.update_body_velocity(
      body_velocity, R, config_.vo_velocity_gate_chi2);
    vo_vel_.last_nis = r.nis;
    vo_vel_.last_stamp = t;
    if (r.accepted) {
      ++vo_vel_.accepted;
    } else {
      ++vo_vel_.rejected;
      log(LogLevel::Debug, format("VO velocity rejected at t=%.2f: NIS %.1f", t, r.nis));
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
    bias_var = std::max(var / static_cast<double>(gyro_n_), 0.003 * 0.003);
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
  log(LogLevel::Info, format(
    "initialised at t=%.2f from %d GPS fixes: p=(%.2f, %.2f, %.2f) m (sigma %.2f m), "
    "yaw=%.1f deg, gyro bias=%+.4f rad/s from %zu samples",
    init_gps_stamp_, n, p0.x(), p0.y(), p0.z(), std::sqrt(P_pos(0, 0)),
    config_.init_yaw * 180.0 / M_PI, bias0, gyro_n_));

  init_gps_.clear();
  init_gps_cov_.clear();
}

}  // namespace drone_localization
