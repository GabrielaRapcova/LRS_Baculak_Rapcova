#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "drone_localization/ekf.hpp"
#include "drone_localization/localizer.hpp"

using namespace drone_localization;

namespace
{

bool is_psd(const StateMat & P)
{
  if (!P.isApprox(P.transpose(), 1e-12)) {return false;}
  Eigen::SelfAdjointEigenSolver<StateMat> es(P);
  return es.eigenvalues().minCoeff() >= -1e-12;
}

Ekf make_ekf(const StateVec & x0, double sigma = 1.0)
{
  Ekf ekf;
  ekf.initialize(0.0, x0, StateMat::Identity() * sigma * sigma);
  return ekf;
}

}  // namespace

TEST(WrapAngle, StaysInRange)
{
  EXPECT_NEAR(wrap_angle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(wrap_angle(3.0 * M_PI / 2.0), -M_PI / 2.0, 1e-12);
  EXPECT_NEAR(wrap_angle(-3.0 * M_PI / 2.0), M_PI / 2.0, 1e-12);
  EXPECT_NEAR(wrap_angle(7.0 * M_PI), -M_PI, 1e-12);
  for (double a = -20.0; a < 20.0; a += 0.37) {
    const double w = wrap_angle(a);
    EXPECT_GE(w, -M_PI);
    EXPECT_LT(w, M_PI);
    EXPECT_NEAR(std::remainder(w - a, 2.0 * M_PI), 0.0, 1e-9);
  }
}

TEST(Ekf, PredictConstantVelocity)
{
  StateVec x0 = StateVec::Zero();
  x0(VX) = 1.0;
  x0(VY) = -0.5;
  Ekf ekf = make_ekf(x0, 0.1);
  const StateMat P_before = ekf.P();
  for (int i = 1; i <= 100; ++i) {ASSERT_TRUE(ekf.predict(i * 0.01, 0.0));}
  EXPECT_NEAR(ekf.x()(PX), 1.0, 1e-9);
  EXPECT_NEAR(ekf.x()(PY), -0.5, 1e-9);
  EXPECT_GT(ekf.P()(PX, PX), P_before(PX, PX));  // uncertainty grows
  EXPECT_TRUE(is_psd(ekf.P()));
}

TEST(Ekf, PredictRejectsNonIncreasingTime)
{
  Ekf ekf = make_ekf(StateVec::Zero());
  ASSERT_TRUE(ekf.predict(1.0, 0.0));
  EXPECT_FALSE(ekf.predict(1.0, 0.0));
  EXPECT_FALSE(ekf.predict(0.5, 0.0));
  EXPECT_DOUBLE_EQ(ekf.time(), 1.0);
}

TEST(Ekf, YawIntegratesBiasCorrectedRateAndWraps)
{
  StateVec x0 = StateVec::Zero();
  x0(YAW) = M_PI - 0.1;
  x0(BIAS) = 0.02;
  Ekf ekf = make_ekf(x0, 0.01);
  // gyro reads 0.22 rad/s, true rate 0.2 rad/s, for 1 s -> yaw +0.2 rad.
  for (int i = 1; i <= 100; ++i) {ekf.predict(i * 0.01, 0.22);}
  EXPECT_NEAR(ekf.x()(YAW), wrap_angle(M_PI + 0.1), 1e-9);
  EXPECT_LT(ekf.x()(YAW), 0.0);  // wrapped through +pi
  // Yaw uncertainty is correlated (negatively) with the bias.
  EXPECT_LT(ekf.P()(YAW, BIAS), 0.0);
}

TEST(Ekf, PositionUpdateMovesTowardMeasurementAndShrinksCovariance)
{
  Ekf ekf = make_ekf(StateVec::Zero(), 1.0);
  const Eigen::Vector3d z(1.0, -2.0, 0.5);
  const auto r = ekf.update_position(z, Eigen::Matrix3d::Identity(), 0.0);
  ASSERT_TRUE(r.accepted);
  // Equal prior and measurement variance -> halfway.
  EXPECT_NEAR(ekf.x()(PX), 0.5, 1e-9);
  EXPECT_NEAR(ekf.x()(PY), -1.0, 1e-9);
  EXPECT_NEAR(ekf.P()(PX, PX), 0.5, 1e-9);
  EXPECT_NEAR(r.nis, (1.0 + 4.0 + 0.25) / 2.0, 1e-9);
  EXPECT_TRUE(is_psd(ekf.P()));
}

TEST(Ekf, GateRejectsOutlierWithoutChangingState)
{
  StateMat P0 = StateMat::Identity() * 0.04;  // 0.2 m position sigma
  Ekf ekf;
  ekf.initialize(0.0, StateVec::Zero(), P0);
  const Eigen::Matrix3d R = Eigen::Vector3d(0.36, 0.36, 0.81).asDiagonal();
  const auto r = ekf.update_position(Eigen::Vector3d(6.0, 0.0, 0.0), R, 16.27);
  EXPECT_FALSE(r.accepted);
  EXPECT_GT(r.nis, 16.27);
  EXPECT_TRUE(ekf.x().isZero());
  EXPECT_TRUE(ekf.P().isApprox(P0));
}

TEST(Ekf, RandomUpdatesKeepCovariancePsd)
{
  std::mt19937 rng(1);
  std::normal_distribution<double> n(0.0, 1.0);
  Ekf ekf = make_ekf(StateVec::Zero(), 2.0);
  for (int i = 1; i < 2000; ++i) {
    ekf.predict(i * 0.01, 0.01 * n(rng));
    if (i % 20 == 0) {
      ekf.update_position(Eigen::Vector3d(n(rng), n(rng), n(rng)),
        Eigen::Matrix3d::Identity() * 0.36, 0.0);
    }
  }
  EXPECT_TRUE(is_psd(ekf.P()));
}

// ---- Localizer ------------------------------------------------------------

TEST(Localizer, InitialisesFromMedianOfGpsAndGyroMean)
{
  LocalizerConfig cfg;
  cfg.init_gps_samples = 5;
  Localizer loc(cfg);
  std::vector<std::string> logs;
  loc.set_logger([&](LogLevel, const std::string & m) {logs.push_back(m);});

  double t = 0.0;
  const Eigen::Matrix3d cov = Eigen::Vector3d(0.36, 0.36, 0.81).asDiagonal();
  const std::vector<Eigen::Vector3d> fixes = {
    {-7.1, 13.0, 0.0}, {-6.9, 13.1, 0.1}, {-1.0, 19.0, 0.0},  // third is a 7 m outlier
    {-7.0, 12.9, -0.1}, {-7.05, 13.05, 0.0}};
  for (const auto & f : fixes) {
    for (int k = 0; k < 20; ++k) {
      t += 0.01;
      loc.on_imu(t, -0.017);
    }
    EXPECT_FALSE(loc.initialized());
    loc.on_gps(t, f, cov);
  }
  ASSERT_TRUE(loc.initialized());
  EXPECT_NEAR(loc.ekf().x()(PX), -7.0, 0.06);
  EXPECT_NEAR(loc.ekf().x()(PY), 13.05, 0.06);
  EXPECT_NEAR(loc.ekf().x()(YAW), M_PI / 2.0, 1e-9);
  EXPECT_NEAR(loc.ekf().x()(BIAS), -0.017, 1e-9);
  EXPECT_DOUBLE_EQ(loc.ekf().time(), t);
  EXPECT_FALSE(logs.empty());
}

TEST(Localizer, MeasurementTiming)
{
  LocalizerConfig cfg;
  cfg.init_gps_samples = 1;
  cfg.max_measurement_lag = 0.05;
  Localizer loc(cfg);
  const Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 0.36;
  loc.on_gps(1.0, Eigen::Vector3d::Zero(), cov);
  ASSERT_TRUE(loc.initialized());

  // A GPS fix ahead of the filter predicts the filter to its stamp.
  loc.on_gps(1.2, Eigen::Vector3d::Zero(), cov);
  EXPECT_DOUBLE_EQ(loc.ekf().time(), 1.2);
  EXPECT_EQ(loc.gps_stats().accepted, 1u);

  // IMU then moves time forward; a slightly late fix is still used...
  loc.on_imu(1.23, 0.0);
  loc.on_gps(1.2, Eigen::Vector3d::Zero(), cov);
  EXPECT_EQ(loc.gps_stats().late, 1u);
  EXPECT_EQ(loc.gps_stats().accepted, 2u);

  // ...but one older than the allowed lag is dropped.
  loc.on_imu(1.40, 0.0);
  loc.on_gps(1.2, Eigen::Vector3d::Zero(), cov);
  EXPECT_EQ(loc.gps_stats().stale, 1u);
  EXPECT_EQ(loc.gps_stats().accepted, 2u);

  // An IMU sample older than the filter time is not integrated twice.
  const double t_before = loc.ekf().time();
  loc.on_imu(1.30, 5.0);
  EXPECT_DOUBLE_EQ(loc.ekf().time(), t_before);
}

TEST(Localizer, StationaryConvergence)
{
  LocalizerConfig cfg;
  Localizer loc(cfg);
  std::mt19937 rng(7);
  std::normal_distribution<double> gps(0.0, 0.6), gyro(0.0, 0.004);
  const Eigen::Vector3d truth(-7.0, 13.0, 0.0);
  const Eigen::Matrix3d cov = Eigen::Vector3d(0.36, 0.36, 0.81).asDiagonal();
  for (int i = 1; i <= 6000; ++i) {  // 60 s
    const double t = i * 0.01;
    loc.on_imu(t, 0.02 + gyro(rng));
    if (i % 20 == 0) {
      loc.on_gps(t, truth + Eigen::Vector3d(gps(rng), gps(rng), 1.5 * gps(rng)), cov);
    }
  }
  ASSERT_TRUE(loc.initialized());
  // GPS-only: velocity is only weakly observable, so check that the errors are
  // consistent with the filter's own covariance (within 3 sigma) instead of a
  // fixed bound. Tight velocity bounds come with the VO update.
  const auto sd = loc.ekf().P().diagonal().cwiseSqrt();
  for (int i = PX; i <= PZ; ++i) {
    EXPECT_LT(std::abs(loc.ekf().x()(i) - truth(i)), 3.0 * sd(i)) << "axis " << i;
    EXPECT_LT(std::abs(loc.ekf().x()(VX + i)), 3.0 * sd(VX + i)) << "axis " << i;
  }
  EXPECT_LT((loc.ekf().x().head<2>() - truth.head<2>()).norm(), 0.5);
  EXPECT_NEAR(loc.ekf().x()(YAW), M_PI / 2.0, 0.05);
  EXPECT_EQ(loc.gps_stats().rejected, 0u);
}

// ---- VO updates -------------------------------------------------------------

TEST(Ekf, BodyVelocityJacobianMatchesNumericalDerivative)
{
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int trial = 0; trial < 20; ++trial) {
    StateVec x0;
    for (int k = 0; k < kStateDim; ++k) {x0(k) = u(rng);}
    x0(YAW) *= M_PI;
    Eigen::Matrix<double, 3, kStateDim> H;
    Ekf::body_velocity_model(x0, &H);
    const double eps = 1e-6;
    for (int j = 0; j < kStateDim; ++j) {
      StateVec xp = x0, xm = x0;
      xp(j) += eps;
      xm(j) -= eps;
      const Eigen::Vector3d numeric =
        (Ekf::body_velocity_model(xp) - Ekf::body_velocity_model(xm)) / (2 * eps);
      EXPECT_LT((numeric - H.col(j)).norm(), 1e-7) << "trial " << trial << " column " << j;
    }
  }
}

TEST(Ekf, BodyVelocityUpdateRotatesIntoWorld)
{
  // Facing north (yaw = pi/2), moving forward at 1 m/s in body x = +y in ENU.
  StateVec x0 = StateVec::Zero();
  x0(YAW) = M_PI / 2.0;
  StateMat P0 = StateMat::Identity();
  P0(YAW, YAW) = 1e-6;
  Ekf ekf;
  ekf.initialize(0.0, x0, P0);
  for (int i = 0; i < 20; ++i) {
    ekf.update_body_velocity(Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Matrix3d::Identity() * 0.01, 0.0);
  }
  EXPECT_NEAR(ekf.x()(VX), 0.0, 0.02);
  EXPECT_NEAR(ekf.x()(VY), 1.0, 0.02);
}

TEST(Ekf, YawUpdateWrapsInnovation)
{
  StateVec x0 = StateVec::Zero();
  x0(YAW) = M_PI - 0.05;
  Ekf ekf = make_ekf(x0, 0.1);
  const auto r = ekf.update_yaw(-M_PI + 0.05, 0.01, 0.0);  // 0.1 rad away, across +-pi
  ASSERT_TRUE(r.accepted);
  EXPECT_LT(r.nis, 1.0);  // small innovation, not ~2 pi
  EXPECT_GT(std::abs(ekf.x()(YAW)), M_PI - 0.05);
}

TEST(Localizer, RelativeVoYawEstimatesGyroBiasInFlight)
{
  LocalizerConfig cfg;
  cfg.init_gps_samples = 5;
  cfg.init_bias_from_gyro = false;  // force the filter to find the bias itself
  Localizer loc(cfg);
  std::mt19937 rng(3);
  std::normal_distribution<double> n(0.0, 1.0);
  const double bias = -0.017;
  const Eigen::Matrix3d gps_cov = Eigen::Vector3d(0.36, 0.36, 0.81).asDiagonal();
  double yaw = M_PI / 2.0;
  for (int i = 1; i <= 6000; ++i) {  // 60 s, turning slowly
    const double t = i * 0.01;
    const double rate = 0.2 * std::sin(0.1 * t);
    yaw = wrap_angle(yaw + rate * 0.01);
    loc.on_imu(t, rate + bias + 0.004 * n(rng));
    if (i % 20 == 0) {
      loc.on_gps(t, Eigen::Vector3d(0.6 * n(rng), 0.6 * n(rng), 0.9 * n(rng)), gps_cov);
    }
    if (i % 3 == 0) {
      loc.on_vo(t, wrap_angle(yaw + 0.005 * n(rng)),
        Eigen::Vector3d(0.03 * n(rng), 0.03 * n(rng), 0.03 * n(rng)),
        Eigen::Vector3d::Constant(0.0009));
    }
  }
  ASSERT_TRUE(loc.initialized());
  EXPECT_NEAR(loc.ekf().x()(BIAS), bias, 0.001);
  EXPECT_GT(loc.vo_yaw_stats().accepted, 20u);
  EXPECT_LT(std::abs(wrap_angle(loc.ekf().x()(YAW) - yaw)), 0.05);
  // Velocity is now tightly constrained (drone is hovering).
  EXPECT_LT(loc.ekf().x().segment<3>(VX).norm(), 0.1);
}
