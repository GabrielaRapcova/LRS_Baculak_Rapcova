// Eight-state extended Kalman filter for drone localisation.
//
// State, ENU `map` frame:
//   x = [px py pz  vx vy vz  yaw  bias]
//   position [m], velocity [m/s], yaw [rad] in (-pi, pi], gyro-Z bias [rad/s].
//   The true yaw rate is (gyro_z - bias).
//
// Process model (constant velocity, yaw driven by the gyro):
//   p    += v * dt
//   v    += white acceleration noise
//   yaw  += (gyro_z - bias) * dt
//   bias += random walk
#pragma once

#include <Eigen/Dense>

namespace drone_localization
{

constexpr int kStateSize = 8;
using StateVector = Eigen::Matrix<double, kStateSize, 1>;
using StateMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;

// Index of each state inside StateVector.
enum StateIndex : int { PX = 0, PY, PZ, VX, VY, VZ, YAW, BIAS };

struct ProcessNoise
{
  double accel_xy = 0.1;     // horizontal white-acceleration density [m^2/s^3]
  double accel_z = 0.1;      // vertical white-acceleration density [m^2/s^3]
  double gyro_noise = 0.004; // gyro white noise per sample [rad/s]
  double gyro_rate = 100.0;  // gyro sample rate, converts gyro_noise to a density [Hz]
  double bias_walk = 2e-5;   // gyro bias random walk [rad/s/sqrt(s)]
};

struct UpdateResult
{
  bool accepted = false;           // the state was corrected
  bool numerical_failure = false;  // S not positive definite or a non-finite result
  double nis = 0.0;                // normalised innovation squared, y' S^-1 y
};

class Ekf
{
public:
  explicit Ekf(const ProcessNoise & noise = ProcessNoise{});

  void initialize(double time, const StateVector & x0, const StateMatrix & P0);
  bool initialized() const {return initialized_;}

  // Moves the filter to `time`, holding `gyro_z` constant over the interval.
  // Returns false (and changes nothing) if `time` is not after the filter time
  // or the result would not be finite.
  bool predict(double time, double gyro_z);

  // Kalman update for any measurement: z - h(x) = innovation, H = dh/dx.
  // The measurement is rejected when its NIS exceeds `gate_chi2`.
  UpdateResult update(
    const Eigen::VectorXd & innovation, const Eigen::MatrixXd & H,
    const Eigen::MatrixXd & R, double gate_chi2);

  // Measurement models used by the localizer.
  UpdateResult update_position(
    const Eigen::Vector3d & position, const Eigen::Matrix3d & R, double gate_chi2);
  UpdateResult update_velocity(
    const Eigen::Vector3d & velocity, const Eigen::Matrix3d & R, double gate_chi2);
  UpdateResult update_body_velocity(
    const Eigen::Vector3d & body_velocity, const Eigen::Matrix3d & R, double gate_chi2);
  UpdateResult update_bias(double bias, double variance, double gate_chi2);

  // VO body-velocity model: world velocity rotated into the yaw-only body
  // frame. Fills `H` (dh/dx) when given.
  static Eigen::Vector3d body_velocity_model(
    const StateVector & x, Eigen::Matrix<double, 3, kStateSize> * H = nullptr);

  // Raises the position variances to at least `variance` (per axis).
  void inflate_position_variance(const Eigen::Vector3d & variance);

  const StateVector & x() const {return x_;}
  const StateMatrix & P() const {return P_;}
  double time() const {return time_;}

private:
  ProcessNoise noise_;
  StateVector x_ = StateVector::Zero();
  StateMatrix P_ = StateMatrix::Identity();
  double time_ = 0.0;
  bool initialized_ = false;
};

}  // namespace drone_localization
