// Sensor-handling layer around the EKF (no ROS).
//
// Responsibilities:
//   * deliberate initialisation from data (median of the first GPS fixes,
//     configured initial ENU yaw, gyro mean on the ground as the first bias guess);
//   * time ordering: every measurement is applied at its own header stamp. The
//     filter is predicted forward to a measurement's stamp with the latest gyro
//     rate; a measurement slightly older than the filter time (arrival jitter)
//     is applied at the filter time, and anything older is dropped and counted;
//   * per-sensor statistics and log messages through a pluggable callback so the
//     ROS node, the offline replay and the unit tests share the same code.
#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "drone_localization/ekf.hpp"

namespace drone_localization
{

enum class LogLevel { Debug, Info, Warn, Error };

struct LocalizerConfig
{
  ProcessNoise noise;

  // Initialisation.
  int init_gps_samples = 10;          // GPS fixes to collect (2 s at 5 Hz)
  double init_yaw = M_PI / 2.0;       // known initial ENU yaw [rad]
  double init_yaw_sigma = 0.17;       // [rad] (~10 deg)
  double init_velocity_sigma = 0.2;   // [m/s]
  double init_bias_sigma = 0.03;      // [rad/s] when not estimated from the gyro
  bool init_bias_from_gyro = true;    // gyro mean while waiting for GPS (drone at rest)

  // Timing.
  double max_measurement_lag = 0.05;  // [s] older measurements are dropped

  // GPS.
  bool gps_use_message_covariance = true;
  std::array<double, 3> gps_sigma{0.6, 0.6, 0.9};  // used if the message has none
  double gps_sigma_scale = 1.0;       // multiplies the GPS standard deviation
  double gps_gate_chi2 = 16.27;       // chi2(3 dof) at 99.9 %; <= 0 disables

  // VO body velocity (nonlinear in yaw).
  bool vo_velocity_enabled = true;
  // [m/s] floor on the per-axis sigma. Deliberately much larger than the
  // per-sample noise (0.03): the VO bias and scale error are correlated in
  // time, and fusing them at 30 Hz as white noise made the filter
  // overconfident (tuned offline: 0.2 gives the lowest error *and* ~95 % 2-sigma
  // consistency).
  double vo_velocity_sigma = 0.2;
  double vo_velocity_gate_chi2 = 16.27;

  // VO yaw. The message covariance (0.3 deg) only describes per-sample noise,
  // not the slow vo_odom drift, so it is not used for the absolute update.
  // Off by default: vo_odom yaw drifts away from map (random walk + rate bias)
  // from the moment the sensor simulator starts, so it is a biased heading;
  // fusing it made yaw inconsistent after an idle period.
  bool vo_yaw_absolute = false;       // fuse VO yaw as absolute heading
  double vo_yaw_sigma = 0.05;         // [rad] absolute: must cover accumulated drift
  double vo_yaw_period = 1.0;         // [s] decimation of the absolute update
  bool vo_yaw_relative = true;        // bias from (gyro integral - VO yaw change)
  double vo_yaw_window = 2.0;         // [s] baseline of the relative measurement
  double vo_pose_yaw_sigma = 0.0052;  // [rad] VO per-sample yaw noise (0.3 deg)
  double vo_yaw_rate_sigma = 0.001;   // [rad/s] VO yaw-rate drift allowance
  double vo_yaw_gate_chi2 = 10.83;    // chi2(1 dof) at 99.9 %
};

struct SensorStats
{
  std::size_t received = 0;
  std::size_t accepted = 0;
  std::size_t rejected = 0;   // failed the innovation gate
  std::size_t stale = 0;      // older than max_measurement_lag
  std::size_t late = 0;       // slightly older than the filter, applied anyway
  double last_stamp = 0.0;
  double last_nis = 0.0;
};

class Localizer
{
public:
  using LogFn = std::function<void (LogLevel, const std::string &)>;

  explicit Localizer(const LocalizerConfig & config = LocalizerConfig{});

  void set_logger(LogFn fn) {log_ = std::move(fn);}
  void set_config(const LocalizerConfig & config);
  const LocalizerConfig & config() const {return config_;}

  void on_imu(double t, double gyro_z);
  // `cov` is the 3x3 position covariance from the message (may be zero).
  void on_gps(double t, const Eigen::Vector3d & position, const Eigen::Matrix3d & cov);
  // `vo_yaw` in vo_odom, `body_velocity` in base_link, `velocity_var` per axis
  // from the message twist covariance (may be zero).
  void on_vo(
    double t, double vo_yaw, const Eigen::Vector3d & body_velocity,
    const Eigen::Vector3d & velocity_var);

  bool initialized() const {return ekf_.initialized();}
  const Ekf & ekf() const {return ekf_;}
  const SensorStats & gps_stats() const {return gps_;}
  const SensorStats & imu_stats() const {return imu_;}
  const SensorStats & vo_velocity_stats() const {return vo_vel_;}
  const SensorStats & vo_yaw_stats() const {return vo_yaw_;}

private:
  void log(LogLevel level, const std::string & msg) const;
  void try_initialize();
  // Bring the filter to time t before an update. False = drop the measurement.
  bool align_to(double t, SensorStats & stats, const char * sensor);
  Eigen::Matrix3d gps_noise(const Eigen::Matrix3d & cov) const;

  LocalizerConfig config_;
  Ekf ekf_;
  LogFn log_;

  // Integral of the raw gyro over IMU stamps, evaluated at any time t by
  // extending the last sample (aligns the gyro with VO stamps).
  double gyro_integral_at(double t) const;
  void vo_yaw_updates(double t, double vo_yaw);

  SensorStats gps_, imu_, vo_vel_, vo_yaw_;
  double gyro_integral_ = 0.0;
  double gyro_integral_stamp_ = 0.0;
  bool have_vo_anchor_ = false;
  double vo_anchor_t_ = 0.0, vo_anchor_yaw_ = 0.0, vo_anchor_gyro_ = 0.0;
  double last_abs_yaw_t_ = -1e9;
  double last_gyro_ = 0.0;
  bool have_gyro_ = false;

  // Initialisation buffers.
  std::vector<Eigen::Vector3d> init_gps_;
  std::vector<Eigen::Matrix3d> init_gps_cov_;
  double init_gps_stamp_ = 0.0;
  double gyro_sum_ = 0.0;
  double gyro_sq_sum_ = 0.0;
  std::size_t gyro_n_ = 0;
};

}  // namespace drone_localization
