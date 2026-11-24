#include "drone_localization/localizer.hpp"

#include <algorithm>
#include <cmath>

#include <angles/angles.h>

namespace drone_localization
{

namespace
{

double median(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();
  return (n % 2 == 1) ? values[n / 2] : 0.5 * (values[n / 2 - 1] + values[n / 2]);
}


}  // namespace

Localizer::Localizer(const LocalizerConfig & config, const rclcpp::Logger & logger)
: config_(config), logger_(logger), ekf_(config.noise)
{
  gps_link_ = {"GPS", config.gps_timeout};
  vo_link_ = {"VO", config.vo_timeout};
  imu_link_ = {"IMU", config.imu_timeout};
}

// ---- IMU: prediction ----------------------------------------------------------

void Localizer::on_imu(double t, double gyro_z)
{
  if (!std::isfinite(t) || !std::isfinite(gyro_z)) {
    return;
  }
  mark_data(imu_link_, t);

  if (!ekf_.initialized()) {
    // Still on the ground: the mean gyro reading is the bias.
    init_gyro_sum_ += gyro_z;
    init_gyro_square_sum_ += gyro_z * gyro_z;
    ++init_gyro_count_;
    last_gyro_ = gyro_z;
    return;
  }

  if (t <= ekf_.time()) {
    // A measurement already moved the filter past this sample: keep the rate
    // for the next interval, but do not integrate it twice.
    last_gyro_ = gyro_z;
    return;
  }

  // A gyro sample stamped t describes the rate over the interval ending at t.
  gyro_integral_ += gyro_z * (t - gyro_integral_time_);
  gyro_integral_time_ = t;
  if (!ekf_.predict(t, gyro_z)) {
    report_numerical_failure("prediction", t);
  }
  last_gyro_ = gyro_z;

  update_while_resting(t, gyro_z);
  check_lost(gps_link_, t);
  if (check_lost(vo_link_, t)) {
    have_vo_reference_ = false;  // restart the VO yaw baseline after the gap
  }
}

bool Localizer::predict_to(double t, SensorStats & stats)
{
  check_lost(imu_link_, t);
  const double filter_time = ekf_.time();
  if (t > filter_time) {
    if (!ekf_.predict(t, last_gyro_)) {
      report_numerical_failure("prediction", t);
      return false;
    }
    return true;
  }
  if (t >= filter_time - config_.max_measurement_lag) {
    return true;  // slightly late: apply at the filter time
  }
  ++stats.dropped;
  RCLCPP_WARN(logger_, "measurement %.3f s older than the filter, dropped", filter_time - t);
  return false;
}

// ---- GPS: position ------------------------------------------------------------

void Localizer::on_gps(
  double t, const Eigen::Vector3d & position, const Eigen::Matrix3d & covariance)
{
  if (!std::isfinite(t) || !position.allFinite() || !covariance.allFinite()) {
    RCLCPP_WARN(logger_, "GPS message with non-finite values ignored");
    return;
  }

  if (!ekf_.initialized()) {
    mark_data(gps_link_, t);
    init_positions_.push_back(position);
    init_covariance_sum_ += covariance;
    if (static_cast<int>(init_positions_.size()) >= config_.init_gps_samples) {
      initialize_from_ground_data();
    }
    return;
  }

  if (!predict_to(t, gps_stats_)) {
    return;
  }
  const bool first_fix_after_outage = mark_data(gps_link_, t);
  const Eigen::Vector3d innovation = position - ekf_.x().segment<3>(PX);
  const UpdateResult result =
    ekf_.update_position(position, covariance, config_.gate_chi2_3dof);

  if (result.numerical_failure) {
    report_numerical_failure("GPS update", t);
    return;
  }
  if (result.accepted) {
    ++gps_stats_.accepted;
    gps_consecutive_rejections_ = 0;
    if (first_fix_after_outage) {
      RCLCPP_INFO(logger_,
        "GPS re-acquired: first fix innovation (%.2f, %.2f, %.2f) m, NIS %.1f, accepted; "
        "correction applied through the Kalman gain (no reset)",
        innovation.x(), innovation.y(), innovation.z(), result.nis);
    }
    return;
  }

  ++gps_stats_.rejected;
  ++gps_consecutive_rejections_;
  RCLCPP_WARN(logger_,
    "GPS REJECTED at t=%.2f: NIS %.1f > gate %.2f (chi2, 3 DOF), innovation "
    "(%.2f, %.2f, %.2f) m = %.2f m%s", t, result.nis, config_.gate_chi2_3dof,
    innovation.x(), innovation.y(), innovation.z(), innovation.norm(),
    first_fix_after_outage ? " [first fix after outage]" : "");

  if (gps_consecutive_rejections_ >= config_.gps_max_consecutive_rejections) {
    // GPS keeps disagreeing: the estimate has drifted. Widen the position
    // uncertainty to the disagreement so the next fixes pass the gate and pull
    // the state back gradually (the state itself is never reset).
    ekf_.inflate_position_variance(innovation.cwiseAbs2());
    const Eigen::Vector3d sigma = ekf_.P().diagonal().segment<3>(PX).cwiseSqrt();
    RCLCPP_WARN(logger_,
      "%d consecutive GPS rejections: assuming the estimate drifted; inflated position "
      "sigma to (%.2f, %.2f, %.2f) m to re-acquire GPS",
      gps_consecutive_rejections_, sigma.x(), sigma.y(), sigma.z());
    gps_consecutive_rejections_ = 0;
  }
}

// ---- VO: body velocity and yaw ------------------------------------------------------

void Localizer::on_vo(
  double t, double vo_yaw, const Eigen::Vector3d & body_velocity,
  const Eigen::Vector3d & velocity_variance)
{
  if (!std::isfinite(t) || !std::isfinite(vo_yaw) || !body_velocity.allFinite()) {
    RCLCPP_WARN(logger_, "VO message with non-finite values ignored");
    return;
  }
  if (!ekf_.initialized() || !predict_to(t, vo_stats_)) {
    return;
  }
  mark_data(vo_link_, t);

  const double min_variance = config_.vo_velocity_sigma * config_.vo_velocity_sigma;
  const Eigen::Matrix3d R =
    velocity_variance.cwiseMax(min_variance).asDiagonal();
  const UpdateResult result =
    ekf_.update_body_velocity(body_velocity, R, config_.gate_chi2_3dof);

  if (result.numerical_failure) {
    report_numerical_failure("VO velocity update", t);
  } else if (result.accepted) {
    ++vo_stats_.accepted;
  } else {
    ++vo_stats_.rejected;
    if (vo_stats_.rejected == 1 || vo_stats_.rejected % 30 == 0) {
      RCLCPP_WARN(logger_, "VO velocity rejected at t=%.2f: NIS %.1f (%d rejected so far)",
        t, result.nis, vo_stats_.rejected);
    }
  }

  update_bias_from_vo_yaw(t, vo_yaw);
}

void Localizer::update_bias_from_vo_yaw(double t, double vo_yaw)
{
  // Gyro integral evaluated exactly at the VO stamp by extending the last sample.
  const double gyro_integral = gyro_integral_ + last_gyro_ * (t - gyro_integral_time_);
  if (!have_vo_reference_) {
    have_vo_reference_ = true;
    vo_reference_time_ = t;
    vo_reference_yaw_ = vo_yaw;
    vo_reference_gyro_integral_ = gyro_integral;
    return;
  }
  const double window = t - vo_reference_time_;
  if (window < config_.vo_yaw_window) {
    return;
  }

  // Over the window the gyro turned (true turn + bias * window) and VO saw the
  // true turn (plus its slow drift); their difference per second is the bias.
  const double vo_turn = angles::shortest_angular_distance(vo_reference_yaw_, vo_yaw);
  const double gyro_turn = gyro_integral - vo_reference_gyro_integral_;
  const double bias = (gyro_turn - vo_turn) / window;

  const double vo_noise = std::sqrt(2.0) * config_.vo_pose_yaw_sigma / window;
  const double gyro_noise =
    config_.noise.gyro_noise * std::sqrt(window / config_.noise.gyro_rate) / window;
  const double variance = vo_noise * vo_noise + gyro_noise * gyro_noise +
    config_.vo_yaw_rate_sigma * config_.vo_yaw_rate_sigma;

  const UpdateResult result = ekf_.update_bias(bias, variance, config_.gate_chi2_1dof);
  if (result.accepted) {
    ++vo_yaw_stats_.accepted;
  } else {
    ++vo_yaw_stats_.rejected;
    RCLCPP_WARN(logger_, "VO yaw bias measurement rejected at t=%.2f: %.4f rad/s, NIS %.1f",
      t, bias, result.nis);
  }
  vo_reference_time_ = t;
  vo_reference_yaw_ = vo_yaw;
  vo_reference_gyro_integral_ = gyro_integral;
}

// ---- Resting on the ground: ZARU + ZUPT ---------------------------------------------

void Localizer::update_resting_state(double t)
{
  const double height = ekf_.x()(PZ) - ground_height_;
  const double speed = ekf_.x().segment<3>(VX).norm();

  bool resting = resting_;
  if (resting_) {
    resting = height <= config_.rest_exit_height && speed <= config_.rest_exit_speed;
  } else if (height < config_.rest_enter_height && speed < config_.rest_enter_speed) {
    if (rest_candidate_since_ < 0.0) {
      rest_candidate_since_ = t;
    }
    resting = t - rest_candidate_since_ >= config_.rest_enter_time;
  } else {
    rest_candidate_since_ = -1.0;
  }

  if (resting != resting_) {
    resting_ = resting;
    rest_candidate_since_ = -1.0;
    RCLCPP_INFO(logger_,
      "t=%.2f %s (height %.2f m above ground estimate, speed %.2f m/s): "
      "zero-rate/zero-velocity updates %s", t,
      resting ? "RESTING on the ground" : "MOVING / airborne", height, speed,
      resting ? "on" : "off");
  }
}

void Localizer::update_while_resting(double t, double gyro_z)
{
  update_resting_state(t);
  if (!resting_) {
    rest_gyro_sum_ = 0.0;
    rest_gyro_count_ = 0;
    return;
  }
  // Follow the converged height so a poor initial GPS height does not bias
  // the detector.
  ground_height_ = ekf_.x()(PZ);

  if (rest_gyro_count_ == 0) {
    rest_window_start_ = t;
  }
  rest_gyro_sum_ += gyro_z;
  ++rest_gyro_count_;
  if (t - rest_window_start_ < config_.rest_update_period) {
    return;
  }
  const double gyro_mean = rest_gyro_sum_ / rest_gyro_count_;
  const int sample_count = rest_gyro_count_;
  rest_gyro_sum_ = 0.0;
  rest_gyro_count_ = 0;

  // ZUPT: standing still means v = 0. If VO/GPS disagree, the drone is taking off.
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * config_.zupt_sigma * config_.zupt_sigma;
  const UpdateResult zupt =
    ekf_.update_velocity(Eigen::Vector3d::Zero(), R, config_.gate_chi2_3dof);
  if (!zupt.accepted) {
    ++zupt_stats_.rejected;
    resting_ = false;
    RCLCPP_INFO(logger_,
      "t=%.2f zero-velocity update rejected (NIS %.1f): MOVING / airborne, "
      "zero-rate/zero-velocity updates off", t, zupt.nis);
    return;
  }
  ++zupt_stats_.accepted;

  // ZARU: standing still means the true yaw rate is 0, so the gyro mean is the bias.
  const double variance =
    config_.noise.gyro_noise * config_.noise.gyro_noise / sample_count +
    config_.zaru_rate_sigma * config_.zaru_rate_sigma;
  const UpdateResult zaru = ekf_.update_bias(gyro_mean, variance, config_.gate_chi2_1dof);
  zaru.accepted ? ++zaru_stats_.accepted : ++zaru_stats_.rejected;
}

// ---- Initialisation -------------------------------------------------------------------

void Localizer::initialize_from_ground_data()
{
  const int n = static_cast<int>(init_positions_.size());

  // Component-wise median of the GPS fixes: robust to a single outlier.
  Eigen::Vector3d position;
  for (int axis = 0; axis < 3; ++axis) {
    std::vector<double> values;
    for (const auto & p : init_positions_) {
      values.push_back(p(axis));
    }
    position(axis) = median(values);
  }
  // The median of n Gaussian samples has variance about (pi/2) * sigma^2 / n.
  const Eigen::Matrix3d position_covariance =
    (init_covariance_sum_ / n) * (M_PI / 2.0) / n;

  // Gyro bias = mean gyro reading while waiting on the ground.
  double bias = 0.0;
  double bias_variance = config_.init_bias_sigma * config_.init_bias_sigma;
  if (init_gyro_count_ > 10) {
    bias = init_gyro_sum_ / init_gyro_count_;
    const double sample_variance =
      std::max(init_gyro_square_sum_ / init_gyro_count_ - bias * bias, 0.0);
    bias_variance = std::max(sample_variance / init_gyro_count_,
        config_.init_bias_min_sigma * config_.init_bias_min_sigma);
  }

  StateVector x0 = StateVector::Zero();
  x0.segment<3>(PX) = position;
  x0(YAW) = config_.init_yaw;
  x0(BIAS) = bias;

  StateMatrix P0 = StateMatrix::Zero();
  P0.block<3, 3>(PX, PX) = position_covariance;
  P0.block<3, 3>(VX, VX) = Eigen::Matrix3d::Identity() *
    config_.init_velocity_sigma * config_.init_velocity_sigma;
  P0(YAW, YAW) = config_.init_yaw_sigma * config_.init_yaw_sigma;
  P0(BIAS, BIAS) = bias_variance;

  const double t = gps_link_.last_stamp;
  ekf_.initialize(t, x0, P0);
  ground_height_ = position.z();
  gyro_integral_ = 0.0;
  gyro_integral_time_ = t;
  have_vo_reference_ = false;

  RCLCPP_INFO(logger_,
    "initialised at t=%.2f from %d GPS fixes: p=(%.2f, %.2f, %.2f) m (sigma %.2f m), "
    "yaw=%.1f deg, gyro bias=%+.4f rad/s from %d samples",
    t, n, position.x(), position.y(), position.z(), std::sqrt(position_covariance(0, 0)),
    angles::to_degrees(config_.init_yaw), bias, init_gyro_count_);
  init_positions_.clear();
}

// ---- Sensor loss / recovery -------------------------------------------------------------

bool Localizer::mark_data(SensorLink & link, double t)
{
  const double gap = t - link.last_stamp;
  link.last_stamp = std::max(link.last_stamp, t);
  if (!link.lost) {
    return false;
  }
  link.lost = false;
  const Eigen::VectorXd sigma = ekf_.P().diagonal().cwiseSqrt();
  RCLCPP_INFO(logger_,
    "%s RECOVERED at t=%.2f after %.1f s (position sigma %.2f, %.2f m, velocity sigma %.2f m/s)",
    link.name.c_str(), t, gap, sigma(PX), sigma(PY), sigma(VX));
  return true;
}

bool Localizer::check_lost(SensorLink & link, double t)
{
  if (link.lost || link.last_stamp < 0.0 || t - link.last_stamp <= link.timeout) {
    return false;
  }
  link.lost = true;
  const Eigen::VectorXd sigma = ekf_.P().diagonal().cwiseSqrt();
  RCLCPP_WARN(logger_,
    "%s LOST at t=%.2f: no data for %.2f s; continuing on the other sensors "
    "(position sigma %.2f, %.2f m, velocity sigma %.2f m/s)",
    link.name.c_str(), t, t - link.last_stamp, sigma(PX), sigma(PY), sigma(VX));
  return true;
}

void Localizer::report_numerical_failure(const char * update, double t)
{
  ++numerical_failures_;
  RCLCPP_ERROR(logger_,
    "%s at t=%.2f produced a non-finite or non-positive-definite result; "
    "kept the previous valid state (failure #%d)", update, t, numerical_failures_);
}

}  // namespace drone_localization
