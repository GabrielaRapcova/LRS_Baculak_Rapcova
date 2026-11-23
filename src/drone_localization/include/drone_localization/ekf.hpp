// Eight-state extended Kalman filter for drone localisation (no ROS).
//
// State (ENU `map` frame):
//   x = [ px py pz  vx vy vz  yaw  b ]
//   p   position [m]
//   v   velocity [m/s]
//   yaw heading about +Z, wrapped to [-pi, pi)
//   b   gyro Z bias [rad/s]; true yaw rate = gyro_z - b
//
// Process model (driven by the gyro, constant velocity otherwise):
//   p   += v * dt
//   v   += white acceleration noise
//   yaw += (gyro_z - b) * dt
//   b   += random walk
//
// Updates use the Joseph form and support a chi-square innovation gate.
#pragma once

#include <Eigen/Dense>

namespace drone_localization
{

constexpr int kStateDim = 8;
using StateVec = Eigen::Matrix<double, kStateDim, 1>;
using StateMat = Eigen::Matrix<double, kStateDim, kStateDim>;

enum StateIndex : int { PX = 0, PY, PZ, VX, VY, VZ, YAW, BIAS };

double wrap_angle(double a);

struct ProcessNoise
{
  double accel_xy = 0.1;     // white acceleration spectral density, horizontal [m^2/s^3]
  double accel_z = 0.1;      // vertical [m^2/s^3]
  double gyro_noise = 0.004; // gyro white noise per sample [rad/s]
  double gyro_rate = 100.0;  // nominal gyro rate used to scale gyro_noise [Hz]
  double bias_walk = 2e-5;   // bias random walk [rad/s/sqrt(s)]; Gazebo bias is constant
};

struct UpdateResult
{
  bool accepted = false;
  bool numerical_failure = false;  // S not positive definite or result non-finite
  double nis = 0.0;  // normalised innovation squared, d^2 = y' S^-1 y
  int dof = 0;
};

class Ekf
{
public:
  explicit Ekf(const ProcessNoise & noise = ProcessNoise{});

  void initialize(double t, const StateVec & x0, const StateMat & P0);
  bool initialized() const {return initialized_;}

  // Propagate to time t using the gyro rate held over [time(), t].
  // Returns false (no change) when t is not after the filter time.
  bool predict(double t, double gyro_z);

  // Generic measurement update. `angle_rows` marks innovation rows that are
  // angles and must be wrapped. A non-positive gate disables gating.
  template<int M>
  UpdateResult update(
    const Eigen::Matrix<double, M, 1> & innovation,
    const Eigen::Matrix<double, M, kStateDim> & H,
    const Eigen::Matrix<double, M, M> & R,
    double gate_chi2,
    const Eigen::Matrix<bool, M, 1> & angle_rows = Eigen::Matrix<bool, M, 1>::Constant(false));

  // GPS position (ENU map) with covariance R.
  UpdateResult update_position(
    const Eigen::Vector3d & z, const Eigen::Matrix3d & R, double gate_chi2);

  // VO body-frame velocity: h(x) = Rz(yaw)^T [vx vy vz] (nonlinear in yaw).
  UpdateResult update_body_velocity(
    const Eigen::Vector3d & z, const Eigen::Matrix3d & R, double gate_chi2);

  // VO measurement model h(x) and, optionally, its Jacobian H = dh/dx.
  static Eigen::Vector3d body_velocity_model(
    const StateVec & x, Eigen::Matrix<double, 3, kStateDim> * H = nullptr);

  // Absolute yaw measurement (innovation wrapped).
  UpdateResult update_yaw(double z, double variance, double gate_chi2);

  // Direct measurement of the gyro bias.
  UpdateResult update_bias(double z, double variance, double gate_chi2);

  const StateVec & x() const {return x_;}
  const StateMat & P() const {return P_;}
  double time() const {return t_;}
  void set_process_noise(const ProcessNoise & n) {noise_ = n;}
  const ProcessNoise & process_noise() const {return noise_;}

  // Inflate the covariance of a block (used for re-acquisition after outages).
  void inflate(int index, int size, double variance);

private:
  void enforce_symmetry();

  ProcessNoise noise_;
  StateVec x_ = StateVec::Zero();
  StateMat P_ = StateMat::Identity();
  double t_ = 0.0;
  bool initialized_ = false;
};

// ---- template implementation ---------------------------------------------

template<int M>
UpdateResult Ekf::update(
  const Eigen::Matrix<double, M, 1> & innovation,
  const Eigen::Matrix<double, M, kStateDim> & H,
  const Eigen::Matrix<double, M, M> & R,
  double gate_chi2,
  const Eigen::Matrix<bool, M, 1> & angle_rows)
{
  UpdateResult result;
  result.dof = M;

  Eigen::Matrix<double, M, 1> y = innovation;
  for (int i = 0; i < M; ++i) {
    if (angle_rows(i)) {y(i) = wrap_angle(y(i));}
  }

  const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
  if (!y.allFinite() || !S.allFinite()) {
    result.numerical_failure = true;
    return result;
  }
  const Eigen::LDLT<Eigen::Matrix<double, M, M>> S_ldlt(S);
  if (S_ldlt.info() != Eigen::Success || !S_ldlt.isPositive()) {
    result.numerical_failure = true;
    return result;
  }

  result.nis = y.dot(S_ldlt.solve(y));
  if (gate_chi2 > 0.0 && result.nis > gate_chi2) {return result;}

  // K = P H' S^-1, computed as (S^-1 H P)' since S and P are symmetric.
  const Eigen::Matrix<double, kStateDim, M> K =
    S_ldlt.solve(H * P_).transpose();

  StateVec x_new = x_ + K * y;
  x_new(YAW) = wrap_angle(x_new(YAW));
  const StateMat I_KH = StateMat::Identity() - K * H;
  StateMat P_new = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();  // Joseph form
  if (!x_new.allFinite() || !P_new.allFinite()) {
    result.numerical_failure = true;  // keep the previous, valid state
    return result;
  }
  x_ = x_new;
  P_ = P_new;
  enforce_symmetry();

  result.accepted = true;
  return result;
}

}  // namespace drone_localization
