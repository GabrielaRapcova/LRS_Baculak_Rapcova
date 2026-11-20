#include "drone_localization/ekf.hpp"

#include <algorithm>
#include <cmath>

namespace drone_localization
{

double wrap_angle(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) {a += 2.0 * M_PI;}
  return a - M_PI;
}

Ekf::Ekf(const ProcessNoise & noise)
: noise_(noise) {}

void Ekf::initialize(double t, const StateVec & x0, const StateMat & P0)
{
  t_ = t;
  x_ = x0;
  x_(YAW) = wrap_angle(x_(YAW));
  P_ = P0;
  enforce_symmetry();
  initialized_ = true;
}

bool Ekf::predict(double t, double gyro_z)
{
  const double dt = t - t_;
  if (!initialized_ || dt <= 0.0) {return false;}

  // State transition.
  x_.segment<3>(PX) += x_.segment<3>(VX) * dt;
  x_(YAW) = wrap_angle(x_(YAW) + (gyro_z - x_(BIAS)) * dt);

  // Jacobian F.
  StateMat F = StateMat::Identity();
  F.block<3, 3>(PX, VX) = Eigen::Matrix3d::Identity() * dt;
  F(YAW, BIAS) = -dt;

  // Discrete process noise.
  StateMat Q = StateMat::Zero();
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  for (int axis = 0; axis < 3; ++axis) {
    const double q = axis < 2 ? noise_.accel_xy : noise_.accel_z;
    Q(PX + axis, PX + axis) = q * dt3 / 3.0;
    Q(PX + axis, VX + axis) = q * dt2 / 2.0;
    Q(VX + axis, PX + axis) = q * dt2 / 2.0;
    Q(VX + axis, VX + axis) = q * dt;
  }
  // Gyro white noise: per-sample sigma at gyro_rate -> spectral density
  // sigma^2 / rate, integrated into yaw over dt.
  Q(YAW, YAW) = noise_.gyro_noise * noise_.gyro_noise / noise_.gyro_rate * dt;
  Q(BIAS, BIAS) = noise_.bias_walk * noise_.bias_walk * dt;

  P_ = F * P_ * F.transpose() + Q;
  enforce_symmetry();
  t_ = t;
  return true;
}

UpdateResult Ekf::update_position(
  const Eigen::Vector3d & z, const Eigen::Matrix3d & R, double gate_chi2)
{
  Eigen::Matrix<double, 3, kStateDim> H = Eigen::Matrix<double, 3, kStateDim>::Zero();
  H.block<3, 3>(0, PX) = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d innovation = z - x_.segment<3>(PX);
  return update<3>(innovation, H, R, gate_chi2);
}

Eigen::Vector3d Ekf::body_velocity_model(
  const StateVec & x, Eigen::Matrix<double, 3, kStateDim> * H)
{
  const double c = std::cos(x(YAW));
  const double s = std::sin(x(YAW));
  const double vx = x(VX);
  const double vy = x(VY);
  if (H != nullptr) {
    H->setZero();
    (*H)(0, VX) = c;
    (*H)(0, VY) = s;
    (*H)(1, VX) = -s;
    (*H)(1, VY) = c;
    (*H)(2, VZ) = 1.0;
    (*H)(0, YAW) = -s * vx + c * vy;   // d h0 / d yaw
    (*H)(1, YAW) = -c * vx - s * vy;   // d h1 / d yaw
  }
  // World velocity rotated into the yaw-only body frame (roll/pitch ignored).
  return Eigen::Vector3d(c * vx + s * vy, -s * vx + c * vy, x(VZ));
}

UpdateResult Ekf::update_body_velocity(
  const Eigen::Vector3d & z, const Eigen::Matrix3d & R, double gate_chi2)
{
  Eigen::Matrix<double, 3, kStateDim> H;
  const Eigen::Vector3d h = body_velocity_model(x_, &H);
  return update<3>(z - h, H, R, gate_chi2);
}

UpdateResult Ekf::update_yaw(double z, double variance, double gate_chi2)
{
  Eigen::Matrix<double, 1, kStateDim> H = Eigen::Matrix<double, 1, kStateDim>::Zero();
  H(0, YAW) = 1.0;
  Eigen::Matrix<double, 1, 1> y(z - x_(YAW));
  Eigen::Matrix<double, 1, 1> R(variance);
  Eigen::Matrix<bool, 1, 1> angle(true);
  return update<1>(y, H, R, gate_chi2, angle);
}

UpdateResult Ekf::update_bias(double z, double variance, double gate_chi2)
{
  Eigen::Matrix<double, 1, kStateDim> H = Eigen::Matrix<double, 1, kStateDim>::Zero();
  H(0, BIAS) = 1.0;
  Eigen::Matrix<double, 1, 1> y(z - x_(BIAS));
  Eigen::Matrix<double, 1, 1> R(variance);
  return update<1>(y, H, R, gate_chi2);
}

void Ekf::inflate(int index, int size, double variance)
{
  for (int i = index; i < index + size; ++i) {
    P_(i, i) = std::max(P_(i, i), variance);
  }
}

void Ekf::enforce_symmetry()
{
  P_ = 0.5 * (P_ + P_.transpose());
}

}  // namespace drone_localization
