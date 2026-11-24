// Sensor handling around the EKF: initialisation, message timing, the
// individual measurement updates and sensor-loss logging.
//
// Timing rule: every measurement is applied at its own header stamp. The filter
// is predicted forward to that stamp with the latest gyro rate. A measurement
// at most `max_measurement_lag` older than the filter is applied at the filter
// time; anything older is dropped.
#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "drone_localization/ekf.hpp"

namespace drone_localization
{

struct LocalizerConfig
{
  ProcessNoise noise;

  // Initialisation (drone at rest on the ground).
  int init_gps_samples = 10;           // median of the first N GPS fixes -> position
  double init_yaw = M_PI / 2.0;        // known spawn heading in ENU [rad]
  double init_yaw_sigma = 0.052;       // [rad] (3 deg)
  double init_velocity_sigma = 0.2;    // [m/s]
  double init_bias_sigma = 0.03;       // [rad/s] if too few gyro samples arrived before init
  double init_bias_min_sigma = 0.003;  // [rad/s] floor on the gyro-mean bias sigma

  double max_measurement_lag = 0.05;   // [s]

  // Chi-square gates at 99.9 %, used by every update of that dimension.
  double gate_chi2_1dof = 10.83;
  double gate_chi2_3dof = 16.27;

  // After this many consecutive GPS rejections the estimate (not GPS) is assumed
  // wrong and the position covariance is inflated so GPS can pull it back.
  int gps_max_consecutive_rejections = 5;

  // VO body velocity. Sigma floor well above the 0.03 m/s sample noise because
  // the VO bias and scale error are correlated in time.
  double vo_velocity_sigma = 0.3;      // [m/s]

  // Gyro bias from VO yaw: (gyro turn - VO turn) / window over `vo_yaw_window`.
  double vo_yaw_window = 2.0;          // [s]
  double vo_pose_yaw_sigma = 0.0052;   // [rad] VO per-sample yaw noise (0.3 deg)
  double vo_yaw_rate_sigma = 0.005;    // [rad/s] VO's own yaw-rate drift

  // Resting on the ground (detected from the estimate, with hysteresis). While
  // resting the true yaw rate and velocity are zero: the gyro mean measures the
  // bias (ZARU) and v = 0 is fused (ZUPT). A rejected ZUPT means takeoff.
  double rest_enter_height = 0.3;      // [m] above the ground-height estimate
  double rest_exit_height = 0.4;       // [m]
  double rest_enter_speed = 0.1;       // [m/s]
  double rest_exit_speed = 0.25;       // [m/s]
  double rest_enter_time = 1.0;        // [s] conditions must hold this long
  double rest_update_period = 1.0;     // [s] ZARU/ZUPT period (gyro averaging window)
  double zaru_rate_sigma = 2e-4;       // [rad/s] allowance for residual rotation
  double zupt_sigma = 0.03;            // [m/s]

  // A sensor is reported lost after this long without data (sim time).
  double gps_timeout = 1.0;            // [s]
  double vo_timeout = 0.3;             // [s]
  double imu_timeout = 0.1;            // [s]
};

struct SensorStats
{
  int accepted = 0;
  int rejected = 0;  // failed the chi-square gate
  int dropped = 0;   // too old to apply
};

// Tracks whether a sensor is delivering data, for loss/recovery logging.
struct SensorLink
{
  std::string name;
  double timeout = 0.0;
  double last_stamp = -1.0;  // < 0: no data yet
  bool lost = false;
};

class Localizer
{
public:
  explicit Localizer(
    const LocalizerConfig & config = LocalizerConfig{},
    const rclcpp::Logger & logger = rclcpp::get_logger("localizer"));

  void on_imu(double t, double gyro_z);
  // `covariance`: 3x3 position covariance from the GPS message.
  void on_gps(double t, const Eigen::Vector3d & position, const Eigen::Matrix3d & covariance);
  // `vo_yaw` in vo_odom, `body_velocity` in base_link, `velocity_variance` per axis.
  void on_vo(
    double t, double vo_yaw, const Eigen::Vector3d & body_velocity,
    const Eigen::Vector3d & velocity_variance);

  bool initialized() const {return ekf_.initialized();}
  const Ekf & ekf() const {return ekf_;}
  const LocalizerConfig & config() const {return config_;}
  bool resting() const {return resting_;}

  const SensorStats & gps_stats() const {return gps_stats_;}
  const SensorStats & vo_stats() const {return vo_stats_;}
  const SensorStats & vo_yaw_stats() const {return vo_yaw_stats_;}
  const SensorStats & zaru_stats() const {return zaru_stats_;}
  const SensorStats & zupt_stats() const {return zupt_stats_;}
  const SensorLink & gps_link() const {return gps_link_;}
  const SensorLink & vo_link() const {return vo_link_;}
  const SensorLink & imu_link() const {return imu_link_;}
  int numerical_failures() const {return numerical_failures_;}

private:
  void initialize_from_ground_data();
  // Moves the filter to the measurement time. False: the measurement is too old.
  bool predict_to(double t, SensorStats & stats);
  void update_bias_from_vo_yaw(double t, double vo_yaw);
  void update_while_resting(double t, double gyro_z);
  void update_resting_state(double t);

  // Sensor-loss bookkeeping: `mark_data` returns true when the sensor has just
  // recovered, `check_lost` returns true when it has just been declared lost.
  bool mark_data(SensorLink & link, double t);
  bool check_lost(SensorLink & link, double t);
  void report_numerical_failure(const char * update, double t);

  LocalizerConfig config_;
  rclcpp::Logger logger_;
  Ekf ekf_;

  SensorStats gps_stats_, vo_stats_, vo_yaw_stats_, zaru_stats_, zupt_stats_;
  SensorLink gps_link_, vo_link_, imu_link_;
  int gps_consecutive_rejections_ = 0;
  int numerical_failures_ = 0;
  double last_gyro_ = 0.0;

  // Data collected on the ground before initialisation.
  std::vector<Eigen::Vector3d> init_positions_;
  Eigen::Matrix3d init_covariance_sum_ = Eigen::Matrix3d::Zero();
  double init_gyro_sum_ = 0.0;
  double init_gyro_square_sum_ = 0.0;
  int init_gyro_count_ = 0;

  // Gyro integral since initialisation, for the VO yaw bias measurement.
  double gyro_integral_ = 0.0;
  double gyro_integral_time_ = 0.0;
  bool have_vo_reference_ = false;
  double vo_reference_time_ = 0.0;
  double vo_reference_yaw_ = 0.0;
  double vo_reference_gyro_integral_ = 0.0;

  // Resting detector and ZARU/ZUPT averaging.
  bool resting_ = false;
  double ground_height_ = 0.0;
  double rest_candidate_since_ = -1.0;  // < 0: rest conditions not met
  double rest_window_start_ = 0.0;
  double rest_gyro_sum_ = 0.0;
  int rest_gyro_count_ = 0;
};

}  // namespace drone_localization
