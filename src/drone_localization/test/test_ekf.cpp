#include <gtest/gtest.h>

#include <cmath>
#include <limits>
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

// Regression: live run started at sim time ~493 s and the first relative VO yaw
// measurement integrated the gyro from stamp 0 (2.96 rad/s, NIS 375641).
TEST(Localizer, RelativeVoYawAtLargeSimTimeHasNoStartupSpike)
{
  LocalizerConfig cfg;
  cfg.init_gps_samples = 3;
  Localizer loc(cfg);
  std::vector<std::string> warnings;
  loc.set_logger([&](LogLevel l, const std::string & m) {
      if (l == LogLevel::Warn) {warnings.push_back(m);}
    });
  const double t0 = 493.0;
  const Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 0.36;
  for (int i = 1; i <= 1000; ++i) {
    const double t = t0 + i * 0.01;
    // GPS and VO arrive in the same instant as initialisation, before the next IMU.
    if (i % 20 == 0) {loc.on_gps(t, Eigen::Vector3d::Zero(), cov);}
    if (i % 3 == 0) {
      loc.on_vo(t, M_PI / 2.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(0.0009));
    }
    loc.on_imu(t, -0.017);
  }
  ASSERT_TRUE(loc.initialized());
  EXPECT_EQ(loc.vo_yaw_stats().rejected, 0u);
  EXPECT_GT(loc.vo_yaw_stats().accepted, 2u);
  EXPECT_NEAR(loc.ekf().x()(BIAS), -0.017, 0.001);
  EXPECT_TRUE(warnings.empty()) << warnings.front();
}

TEST(Localizer, ZeroRateUpdateKeepsYawStillDuringLongGroundWait)
{
  LocalizerConfig cfg;
  cfg.init_gps_samples = 3;  // short ground mean -> poor first bias estimate
  Localizer loc(cfg);
  std::mt19937 rng(11);
  std::normal_distribution<double> n(0.0, 1.0);
  const double bias = 0.0185;
  const Eigen::Matrix3d cov = Eigen::Vector3d(0.36, 0.36, 0.81).asDiagonal();
  for (int i = 1; i <= 50000; ++i) {  // 500 s on the ground
    const double t = 100.0 + i * 0.01;
    loc.on_imu(t, bias + 0.004 * n(rng));
    if (i % 20 == 0) {
      loc.on_gps(t, Eigen::Vector3d(0.6 * n(rng), 0.6 * n(rng), 0.9 * n(rng)), cov);
    }
    if (i % 3 == 0) {
      // VO with a 0.0015 rad/s yaw-rate drift, as measured live.
      loc.on_vo(t, wrap_angle(M_PI / 2.0 + 0.0015 * i * 0.01),
        Eigen::Vector3d(0.03 * n(rng), 0.03 * n(rng), 0.03 * n(rng)),
        Eigen::Vector3d::Constant(0.0009));
    }
  }
  EXPECT_TRUE(loc.resting());
  EXPECT_GT(loc.zaru_stats().accepted, 400u);
  EXPECT_NEAR(loc.ekf().x()(BIAS), bias, 3e-4);
  EXPECT_NEAR(loc.ekf().x()(YAW), M_PI / 2.0, 0.035);  // < 2 deg after 500 s
}

// ---- §3: outages, outliers, robustness ----------------------------------------

namespace
{

struct Sim
{
  Localizer loc;
  std::mt19937 rng{21};
  std::normal_distribution<double> n{0.0, 1.0};
  std::vector<std::string> logs;
  Eigen::Vector3d p{0.0, 0.0, 2.0};
  Eigen::Vector3d v{0.5, 0.0, 0.0};
  double t = 0.0;
  const Eigen::Matrix3d gps_cov = Eigen::Vector3d(0.36, 0.36, 0.81).asDiagonal();

  explicit Sim(LocalizerConfig cfg = {})
  : loc(cfg)
  {
    loc.set_logger([this](LogLevel, const std::string & m) {logs.push_back(m);});
  }

  // Advance 10 ms of straight flight east at 0.5 m/s (yaw 0).
  void step(bool gps = true, bool vo = true, bool outlier = false)
  {
    t += 0.01;
    p += v * 0.01;
    const int i = static_cast<int>(std::lround(t * 100.0));
    loc.on_imu(t, 0.01 + 0.004 * n(rng));
    if (gps && i % 20 == 0) {
      Eigen::Vector3d z = p + Eigen::Vector3d(0.6 * n(rng), 0.6 * n(rng), 0.9 * n(rng));
      if (outlier) {z += Eigen::Vector3d(0.0, 0.0, 5.0);}  // hardest case: vertical 5 m
      loc.on_gps(t, z, gps_cov);
    }
    if (vo && i % 3 == 0) {
      loc.on_vo(t, 0.0, v + 0.03 * Eigen::Vector3d(n(rng), n(rng), n(rng)),
        Eigen::Vector3d::Constant(0.0009));
    }
  }

  bool logged(const std::string & needle) const
  {
    for (const auto & l : logs) {
      if (l.find(needle) != std::string::npos) {return true;}
    }
    return false;
  }
};

LocalizerConfig flying_config()
{
  LocalizerConfig c;
  c.init_yaw = 0.0;
  c.init_gps_samples = 5;
  c.zaru_enabled = false;
  return c;
}

}  // namespace

TEST(Section3, FifteenSecondGpsOutageStaysFiniteAndRecoversWithoutReset)
{
  Sim s(flying_config());
  for (int i = 0; i < 2000; ++i) {s.step();}  // 20 s nominal
  const double sigma_before = std::sqrt(s.loc.ekf().P()(PX, PX));

  for (int i = 0; i < 1500; ++i) {  // 15 s without GPS
    s.step(false, true);
    ASSERT_TRUE(s.loc.ekf().x().allFinite());
    ASSERT_TRUE(s.loc.ekf().P().allFinite());
  }
  EXPECT_TRUE(s.loc.gps_link().lost);
  EXPECT_TRUE(s.logged("GPS LOST"));
  const double sigma_outage = std::sqrt(s.loc.ekf().P()(PX, PX));
  EXPECT_GT(sigma_outage, sigma_before);  // uncertainty grows honestly
  EXPECT_LT((s.loc.ekf().x().head<3>() - s.p).norm(), 1.0);  // VO carried it

  // Recovery: largest single-step change of the estimate stays small.
  double max_jump = 0.0;
  Eigen::Vector3d prev = s.loc.ekf().x().head<3>();
  for (int i = 0; i < 500; ++i) {
    s.step();
    const Eigen::Vector3d cur = s.loc.ekf().x().head<3>();
    max_jump = std::max(max_jump, (cur - prev).norm());
    prev = cur;
  }
  EXPECT_FALSE(s.loc.gps_link().lost);
  EXPECT_TRUE(s.logged("GPS RECOVERED"));
  EXPECT_LT(max_jump, 0.5);
  EXPECT_LT(std::sqrt(s.loc.ekf().P()(PX, PX)), sigma_outage);
  EXPECT_LT((s.loc.ekf().x().head<3>() - s.p).norm(), 0.6);
  EXPECT_EQ(s.loc.numerical_failures(), 0u);
}

TEST(Section3, VoDropoutCarriedByGps)
{
  Sim s(flying_config());
  for (int i = 0; i < 2000; ++i) {s.step();}
  for (int i = 0; i < 300; ++i) {s.step(true, false);}  // 3 s without VO
  EXPECT_TRUE(s.loc.vo_link().lost);
  EXPECT_TRUE(s.logged("VO LOST"));
  EXPECT_LT((s.loc.ekf().x().head<3>() - s.p).norm(), 1.0);
  for (int i = 0; i < 300; ++i) {s.step();}
  EXPECT_FALSE(s.loc.vo_link().lost);
  EXPECT_TRUE(s.logged("VO RECOVERED"));
  EXPECT_LT((s.loc.ekf().x().segment<3>(VX) - s.v).norm(), 0.15);
}

TEST(Section3, VerticalFiveMetreOutlierRejectedEvenRightAfterOutage)
{
  Sim s(flying_config());
  for (int i = 0; i < 2000; ++i) {s.step();}
  for (int i = 0; i < 1500; ++i) {s.step(false, true);}
  const auto rejected_before = s.loc.gps_stats().rejected;
  // First fix after the outage carries the outlier (pending service request).
  bool sent = false;
  while (!sent) {
    const int next = static_cast<int>(std::lround((s.t + 0.01) * 100.0));
    const bool is_gps = next % 20 == 0;
    s.step(true, true, is_gps);
    sent = is_gps;
  }
  EXPECT_EQ(s.loc.gps_stats().rejected, rejected_before + 1);
  EXPECT_TRUE(s.logged("first fix after outage"));
  EXPECT_LT(std::abs(s.loc.ekf().x()(PZ) - s.p.z()), 0.6);  // not pulled 5 m up
}

TEST(Section3, ConsecutiveRejectionsInflateCovarianceInsteadOfLockingOut)
{
  LocalizerConfig cfg = flying_config();
  Sim s(cfg);
  for (int i = 0; i < 2000; ++i) {s.step();}
  // Teleport the truth 4 m north: the estimate is now wrong, GPS is right.
  s.p.y() += 4.0;
  for (int i = 0; i < 1000; ++i) {s.step();}
  EXPECT_TRUE(s.logged("consecutive GPS rejections"));
  EXPECT_LT((s.loc.ekf().x().head<2>() - s.p.head<2>()).norm(), 0.6);  // re-acquired
}

TEST(Section3, NonFiniteInputsAreIgnored)
{
  Sim s(flying_config());
  for (int i = 0; i < 500; ++i) {s.step();}
  const double nan = std::numeric_limits<double>::quiet_NaN();
  s.loc.on_gps(s.t + 0.001, Eigen::Vector3d(nan, 0, 0), s.gps_cov);
  s.loc.on_vo(s.t + 0.001, nan, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  s.loc.on_imu(s.t + 0.002, nan);
  s.loc.on_imu(nan, 0.0);
  EXPECT_TRUE(s.loc.ekf().x().allFinite());
  EXPECT_TRUE(s.loc.ekf().P().allFinite());
  for (int i = 0; i < 100; ++i) {s.step();}
  EXPECT_TRUE(s.loc.ekf().x().allFinite());
}

TEST(Localizer, ZeroVelocityUpdateDoesNotPinTakeoff)
{
  LocalizerConfig cfg;
  cfg.init_gps_samples = 5;
  Localizer loc(cfg);
  std::vector<std::string> logs;
  loc.set_logger([&](LogLevel, const std::string & m) {logs.push_back(m);});
  std::mt19937 rng(4);
  std::normal_distribution<double> n(0.0, 1.0);
  const Eigen::Matrix3d cov = Eigen::Vector3d(0.36, 0.36, 0.81).asDiagonal();
  double z = 0.0, vz = 0.0;
  for (int i = 1; i <= 4000; ++i) {  // 20 s on the ground, then climb at 0.5 m/s
    const double t = i * 0.01;
    if (t > 20.0) {vz = 0.5;}
    z += vz * 0.01;
    loc.on_imu(t, 0.01 + 0.004 * n(rng));
    if (i % 20 == 0) {loc.on_gps(t, Eigen::Vector3d(0.6 * n(rng), 0.6 * n(rng), z + 0.9 * n(rng)), cov);}
    if (i % 3 == 0) {
      loc.on_vo(t, M_PI / 2.0, Eigen::Vector3d(0.03 * n(rng), 0.03 * n(rng), vz + 0.03 * n(rng)),
        Eigen::Vector3d::Constant(0.0009));
    }
    if (std::abs(t - 19.9) < 0.005) {
      EXPECT_TRUE(loc.resting());
      EXPECT_GT(loc.zupt_stats().accepted, 10u);
    }
  }
  EXPECT_FALSE(loc.resting());
  EXPECT_NEAR(loc.ekf().x()(VZ), 0.5, 0.1);  // climb seen, not clamped to 0
  EXPECT_NEAR(loc.ekf().x()(PZ), z, 0.4);
  int transitions = 0;
  for (const auto & l : logs) {
    if (l.find("RESTING") != std::string::npos || l.find("MOVING") != std::string::npos) {++transitions;}
  }
  EXPECT_EQ(transitions, 2);  // rest once, leave once: no flapping
}
