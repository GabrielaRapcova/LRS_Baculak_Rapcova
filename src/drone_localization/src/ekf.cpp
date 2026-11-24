#include "drone_localization/ekf.hpp"

#include <algorithm>
#include <cmath>

#include <angles/angles.h>

namespace drone_localization
{

Ekf::Ekf(const ProcessNoise & noise)
: noise_(noise)
{
}

void Ekf::initialize(double time, const StateVector & x0, const StateMatrix & P0)
{
  time_ = time;
  x_ = x0;
  x_(YAW) = angles::normalize_angle(x_(YAW));
  P_ = 0.5 * (P0 + P0.transpose());
  initialized_ = true;
}

bool Ekf::predict(double time, double gyro_z)
{
  const double dt = time - time_;
  if (!initialized_ || !(dt > 0.0) || !std::isfinite(gyro_z)) {
    return false;
  }

  StateVector x = x_;
  x.segment<3>(PX) += x.segment<3>(VX) * dt;
  x(YAW) = angles::normalize_angle(x(YAW) + (gyro_z - x(BIAS)) * dt);

  // Jacobian of the process model.
  StateMatrix F = StateMatrix::Identity();
  F.block<3, 3>(PX, VX) = Eigen::Matrix3d::Identity() * dt;
  F(YAW, BIAS) = -dt;

  // Discrete process noise: white acceleration per axis, gyro noise into yaw,
  // random walk into the bias.
  StateMatrix Q = StateMatrix::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    const double q = (axis < 2) ? noise_.accel_xy : noise_.accel_z;
    const int p = PX + axis;
    const int v = VX + axis;
    Q(p, p) = q * dt * dt * dt / 3.0;
    Q(p, v) = q * dt * dt / 2.0;
    Q(v, p) = q * dt * dt / 2.0;
    Q(v, v) = q * dt;
  }
  Q(YAW, YAW) = noise_.gyro_noise * noise_.gyro_noise / noise_.gyro_rate * dt;
  Q(BIAS, BIAS) = noise_.bias_walk * noise_.bias_walk * dt;

  StateMatrix P = F * P_ * F.transpose() + Q;
  P = 0.5 * (P + P.transpose());
  if (!x.allFinite() || !P.allFinite()) {
    return false;
  }
  x_ = x;
  P_ = P;
  time_ = time;
  return true;
}

UpdateResult Ekf::update(
  const Eigen::VectorXd & innovation, const Eigen::MatrixXd & H,
  const Eigen::MatrixXd & R, double gate_chi2)
{
  UpdateResult result;
  const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  const Eigen::LDLT<Eigen::MatrixXd> S_ldlt(S);
  if (!innovation.allFinite() || !S.allFinite() ||
    S_ldlt.info() != Eigen::Success || !S_ldlt.isPositive())
  {
    result.numerical_failure = true;
    return result;
  }

  result.nis = innovation.dot(S_ldlt.solve(innovation));
  if (result.nis > gate_chi2) {
    return result;
  }

  // Kalman gain K = P H' S^-1, computed as (S^-1 H P)' because S and P are symmetric.
  const Eigen::MatrixXd K = S_ldlt.solve(H * P_).transpose();
  StateVector x = x_ + K * innovation;
  x(YAW) = angles::normalize_angle(x(YAW));

  // Joseph form keeps P symmetric and positive semi-definite.
  const StateMatrix I_KH = StateMatrix::Identity() - K * H;
  StateMatrix P = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
  P = 0.5 * (P + P.transpose());

  if (!x.allFinite() || !P.allFinite()) {
    result.numerical_failure = true;
    return result;
  }
  x_ = x;
  P_ = P;
  result.accepted = true;
  return result;
}

UpdateResult Ekf::update_position(
  const Eigen::Vector3d & position, const Eigen::Matrix3d & R, double gate_chi2)
{
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, kStateSize);
  H.block<3, 3>(0, PX) = Eigen::Matrix3d::Identity();
  return update(position - x_.segment<3>(PX), H, R, gate_chi2);
}

UpdateResult Ekf::update_velocity(
  const Eigen::Vector3d & velocity, const Eigen::Matrix3d & R, double gate_chi2)
{
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, kStateSize);
  H.block<3, 3>(0, VX) = Eigen::Matrix3d::Identity();
  return update(velocity - x_.segment<3>(VX), H, R, gate_chi2);
}

Eigen::Vector3d Ekf::body_velocity_model(
  const StateVector & x, Eigen::Matrix<double, 3, kStateSize> * H)
{
  const double c = std::cos(x(YAW));
  const double s = std::sin(x(YAW));
  const double vx = x(VX);
  const double vy = x(VY);
  if (H != nullptr) {
    H->setZero();
    (*H)(0, VX) = c;
    (*H)(0, VY) = s;
    (*H)(0, YAW) = -s * vx + c * vy;
    (*H)(1, VX) = -s;
    (*H)(1, VY) = c;
    (*H)(1, YAW) = -c * vx - s * vy;
    (*H)(2, VZ) = 1.0;
  }
  return Eigen::Vector3d(c * vx + s * vy, -s * vx + c * vy, x(VZ));
}

UpdateResult Ekf::update_body_velocity(
  const Eigen::Vector3d & body_velocity, const Eigen::Matrix3d & R, double gate_chi2)
{
  Eigen::Matrix<double, 3, kStateSize> H;
  const Eigen::Vector3d predicted = body_velocity_model(x_, &H);
  return update(body_velocity - predicted, H, R, gate_chi2);
}

UpdateResult Ekf::update_bias(double bias, double variance, double gate_chi2)
{
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, kStateSize);
  H(0, BIAS) = 1.0;
  const Eigen::VectorXd innovation = Eigen::VectorXd::Constant(1, bias - x_(BIAS));
  const Eigen::MatrixXd R = Eigen::MatrixXd::Constant(1, 1, variance);
  return update(innovation, H, R, gate_chi2);
}

void Ekf::inflate_position_variance(const Eigen::Vector3d & variance)
{
  for (int axis = 0; axis < 3; ++axis) {
    P_(PX + axis, PX + axis) = std::max(P_(PX + axis, PX + axis), variance(axis));
  }
}

}  // namespace drone_localization
