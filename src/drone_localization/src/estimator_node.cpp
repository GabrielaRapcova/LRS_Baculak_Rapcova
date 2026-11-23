// Eight-state localisation EKF as a ROS 2 node.
//
// Inputs (the only topics this node subscribes to):
//   /localization/gps  geometry_msgs/PoseWithCovarianceStamped  (map)
//   /localization/vo   nav_msgs/Odometry  (vo_odom -> base_link, body-frame twist)
//   /localization/imu  sensor_msgs/Imu    (only angular_velocity.z)
//
// Outputs:
//   /localization/estimate        nav_msgs/Odometry, frame `map`, child `base_link`.
//       pose: position + yaw quaternion, covariance from P (x, y, z, yaw);
//       twist: velocity in the child (body) frame per the Odometry convention,
//       angular.z = gyro_z - estimated bias.
//   /localization/estimate_state  std_msgs/Float64MultiArray
//       [t, px, py, pz, vx, vy, vz, yaw, bias, sd_px, …, sd_bias]  (ENU, debug/plots)
//
// All filter logic lives in the ROS-free Localizer; this node only converts
// messages, applies parameters and logs at a bounded rate. Callbacks run on a
// single-threaded executor, so the filter is never touched concurrently.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "drone_localization/localizer.hpp"

namespace drone_localization
{

namespace
{

double stamp_seconds(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

builtin_interfaces::msg::Time to_stamp(double t)
{
  builtin_interfaces::msg::Time s;
  s.sec = static_cast<int32_t>(std::floor(t));
  s.nanosec = static_cast<uint32_t>(std::round((t - std::floor(t)) * 1e9));
  if (s.nanosec >= 1000000000u) {
    s.sec += 1;
    s.nanosec -= 1000000000u;
  }
  return s;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class EstimatorNode : public rclcpp::Node
{
public:
  EstimatorNode()
  : Node("localization_estimator")
  {
    const auto gps_topic = declare_parameter<std::string>("gps_topic", "/localization/gps");
    const auto vo_topic = declare_parameter<std::string>("vo_topic", "/localization/vo");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/localization/imu");
    const auto est_topic =
      declare_parameter<std::string>("estimate_topic", "/localization/estimate");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    vo_frame_ = declare_parameter<std::string>("vo_frame", "vo_odom");
    publish_decimation_ = std::max<int64_t>(1, declare_parameter<int64_t>("publish_every_n_imu", 1));
    status_period_ = declare_parameter<double>("status_period_s", 5.0);
    time_jump_reset_s_ = declare_parameter<double>("time_jump_reset_s", 1.0);

    localizer_.set_config(load_config());
    localizer_.set_logger(make_logger());

    const auto qos = rclcpp::SensorDataQoS();
    gps_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      gps_topic, qos,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {on_gps(*m);});
    vo_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      vo_topic, qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_vo(*m);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, qos, [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {on_imu(*m);});

    est_pub_ = create_publisher<nav_msgs::msg::Odometry>(est_topic, 10);
    state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      est_topic + "_state", 10);

    const auto & c = localizer_.config();
    RCLCPP_INFO(get_logger(),
      "estimator up: Q accel_xy=%.2f accel_z=%.2f gyro=%.4f bias_walk=%.1e | "
      "GPS gate chi2=%.2f | VO vel sigma=%.2f, yaw relative=%s window=%.1fs, absolute=%s | "
      "waiting for %d GPS fixes to initialise",
      c.noise.accel_xy, c.noise.accel_z, c.noise.gyro_noise, c.noise.bias_walk,
      c.gps_gate_chi2, c.vo_velocity_sigma, c.vo_yaw_relative ? "on" : "off",
      c.vo_yaw_window, c.vo_yaw_absolute ? "on" : "off", c.init_gps_samples);
  }

private:
  Localizer::LogFn make_logger()
  {
    return [this](LogLevel level, const std::string & msg) {
             switch (level) {
               case LogLevel::Debug: RCLCPP_DEBUG(get_logger(), "%s", msg.c_str()); break;
               case LogLevel::Info: RCLCPP_INFO(get_logger(), "%s", msg.c_str()); break;
               case LogLevel::Warn: RCLCPP_WARN(get_logger(), "%s", msg.c_str()); break;
               case LogLevel::Error: RCLCPP_ERROR(get_logger(), "%s", msg.c_str()); break;
             }
           };
  }

  LocalizerConfig load_config()
  {
    LocalizerConfig c;
    auto d = [this](const std::string & name, double def) {
        return declare_parameter<double>(name, def);
      };
    auto b = [this](const std::string & name, bool def) {
        return declare_parameter<bool>(name, def);
      };
    c.noise.accel_xy = d("accel_xy", c.noise.accel_xy);
    c.noise.accel_z = d("accel_z", c.noise.accel_z);
    c.noise.gyro_noise = d("gyro_noise", c.noise.gyro_noise);
    c.noise.gyro_rate = d("gyro_rate", c.noise.gyro_rate);
    c.noise.bias_walk = d("bias_walk", c.noise.bias_walk);
    c.init_gps_samples = static_cast<int>(
      declare_parameter<int64_t>("init_gps_samples", c.init_gps_samples));
    c.init_yaw = d("init_yaw", c.init_yaw);
    c.init_yaw_sigma = d("init_yaw_sigma", c.init_yaw_sigma);
    c.init_velocity_sigma = d("init_velocity_sigma", c.init_velocity_sigma);
    c.init_bias_sigma = d("init_bias_sigma", c.init_bias_sigma);
    c.init_bias_from_gyro = b("init_bias_from_gyro", c.init_bias_from_gyro);
    c.init_bias_min_sigma = d("init_bias_min_sigma", c.init_bias_min_sigma);
    c.max_measurement_lag = d("max_measurement_lag", c.max_measurement_lag);
    c.gps_use_message_covariance = b("gps_use_message_covariance", c.gps_use_message_covariance);
    const auto sig = declare_parameter<std::vector<double>>(
      "gps_sigma", std::vector<double>(c.gps_sigma.begin(), c.gps_sigma.end()));
    for (std::size_t i = 0; i < 3 && i < sig.size(); ++i) {c.gps_sigma[i] = sig[i];}
    c.gps_sigma_scale = d("gps_sigma_scale", c.gps_sigma_scale);
    c.gps_gate_chi2 = d("gps_gate_chi2", c.gps_gate_chi2);
    c.gps_max_consecutive_rejections = static_cast<int>(declare_parameter<int64_t>(
        "gps_max_consecutive_rejections", c.gps_max_consecutive_rejections));
    c.gps_timeout = d("gps_timeout", c.gps_timeout);
    c.vo_timeout = d("vo_timeout", c.vo_timeout);
    c.imu_timeout = d("imu_timeout", c.imu_timeout);
    c.vo_velocity_enabled = b("vo_velocity_enabled", c.vo_velocity_enabled);
    c.vo_velocity_sigma = d("vo_velocity_sigma", c.vo_velocity_sigma);
    c.vo_velocity_gate_chi2 = d("vo_velocity_gate_chi2", c.vo_velocity_gate_chi2);
    c.vo_yaw_absolute = b("vo_yaw_absolute", c.vo_yaw_absolute);
    c.vo_yaw_sigma = d("vo_yaw_sigma", c.vo_yaw_sigma);
    c.vo_yaw_period = d("vo_yaw_period", c.vo_yaw_period);
    c.vo_yaw_relative = b("vo_yaw_relative", c.vo_yaw_relative);
    c.vo_yaw_window = d("vo_yaw_window", c.vo_yaw_window);
    c.vo_pose_yaw_sigma = d("vo_pose_yaw_sigma", c.vo_pose_yaw_sigma);
    c.vo_yaw_rate_sigma = d("vo_yaw_rate_sigma", c.vo_yaw_rate_sigma);
    c.vo_yaw_gate_chi2 = d("vo_yaw_gate_chi2", c.vo_yaw_gate_chi2);
    c.zaru_enabled = b("zaru_enabled", c.zaru_enabled);
    c.zupt_enabled = b("zupt_enabled", c.zupt_enabled);
    c.zaru_window = d("zaru_window", c.zaru_window);
    c.rest_enter_height = d("rest_enter_height", c.rest_enter_height);
    c.rest_exit_height = d("rest_exit_height", c.rest_exit_height);
    c.rest_enter_speed = d("rest_enter_speed", c.rest_enter_speed);
    c.rest_exit_speed = d("rest_exit_speed", c.rest_exit_speed);
    c.rest_enter_time = d("rest_enter_time", c.rest_enter_time);
    c.zaru_rate_sigma = d("zaru_rate_sigma", c.zaru_rate_sigma);
    c.zupt_sigma = d("zupt_sigma", c.zupt_sigma);
    c.zupt_gate_chi2 = d("zupt_gate_chi2", c.zupt_gate_chi2);
    return c;
  }

  bool frame_ok(const std::string & got, const std::string & expected, const char * what)
  {
    if (got == expected) {return true;}
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "ignoring %s with frame '%s' (expected '%s')", what, got.c_str(), expected.c_str());
    return false;
  }

  void on_gps(const geometry_msgs::msg::PoseWithCovarianceStamped & m)
  {
    if (!frame_ok(m.header.frame_id, map_frame_, "GPS")) {return;}
    const auto & p = m.pose.pose.position;
    const auto & c = m.pose.covariance;
    Eigen::Matrix3d cov;
    cov << c[0], c[1], c[2], c[6], c[7], c[8], c[12], c[13], c[14];
    localizer_.on_gps(stamp_seconds(m.header.stamp), Eigen::Vector3d(p.x, p.y, p.z), cov);
  }

  void on_vo(const nav_msgs::msg::Odometry & m)
  {
    if (!frame_ok(m.header.frame_id, vo_frame_, "VO") ||
      !frame_ok(m.child_frame_id, body_frame_, "VO child"))
    {
      return;
    }
    const auto & v = m.twist.twist.linear;  // base_link (body) frame
    const auto & c = m.twist.covariance;
    localizer_.on_vo(stamp_seconds(m.header.stamp),
      yaw_from_quaternion(m.pose.pose.orientation),
      Eigen::Vector3d(v.x, v.y, v.z), Eigen::Vector3d(c[0], c[7], c[14]));
  }

  void on_imu(const sensor_msgs::msg::Imu & m)
  {
    const double t = stamp_seconds(m.header.stamp);
    if (localizer_.initialized() && t < localizer_.ekf().time() - time_jump_reset_s_) {
      // Simulation time went backwards (Gazebo reset): the old state is meaningless.
      RCLCPP_WARN(get_logger(),
        "simulation time jumped back %.1f s; re-initialising the estimator",
        localizer_.ekf().time() - t);
      localizer_ = Localizer(localizer_.config());
      localizer_.set_logger(make_logger());
    }

    const bool was_initialized = localizer_.initialized();
    const std::size_t before = localizer_.imu_stats().accepted;
    localizer_.on_imu(t, m.angular_velocity.z);
    last_gyro_ = m.angular_velocity.z;
    if (!localizer_.initialized() || localizer_.imu_stats().accepted == before) {return;}
    if (!was_initialized) {next_status_ = t + status_period_;}

    if (++imu_count_ % publish_decimation_ == 0) {publish();}
    if (t >= next_status_) {
      next_status_ = t + status_period_;
      log_status();
    }
  }

  void publish()
  {
    const auto & x = localizer_.ekf().x();
    const auto & P = localizer_.ekf().P();
    const double t = localizer_.ekf().time();
    const double yaw = x(YAW);
    const double c = std::cos(yaw), s = std::sin(yaw);

    nav_msgs::msg::Odometry od;
    od.header.stamp = to_stamp(t);
    od.header.frame_id = map_frame_;
    od.child_frame_id = body_frame_;
    od.pose.pose.position.x = x(PX);
    od.pose.pose.position.y = x(PY);
    od.pose.pose.position.z = x(PZ);
    od.pose.pose.orientation.z = std::sin(0.5 * yaw);
    od.pose.pose.orientation.w = std::cos(0.5 * yaw);
    // 6x6 pose covariance (x y z roll pitch yaw); roll/pitch not estimated.
    const int idx[4] = {PX, PY, PZ, YAW};
    const int row[4] = {0, 1, 2, 5};
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        od.pose.covariance[row[i] * 6 + row[j]] = P(idx[i], idx[j]);
      }
    }
    od.pose.covariance[3 * 6 + 3] = 1e6;
    od.pose.covariance[4 * 6 + 4] = 1e6;
    // Body-frame twist (Odometry convention: twist is in child_frame_id).
    od.twist.twist.linear.x = c * x(VX) + s * x(VY);
    od.twist.twist.linear.y = -s * x(VX) + c * x(VY);
    od.twist.twist.linear.z = x(VZ);
    od.twist.twist.angular.z = last_gyro_ - x(BIAS);
    for (int i = 0; i < 3; ++i) {od.twist.covariance[i * 7] = P(VX + i, VX + i);}
    od.twist.covariance[35] = P(BIAS, BIAS);
    est_pub_->publish(od);

    std_msgs::msg::Float64MultiArray st;
    st.data.reserve(1 + 2 * kStateDim);
    st.data.push_back(t);
    for (int i = 0; i < kStateDim; ++i) {st.data.push_back(x(i));}
    for (int i = 0; i < kStateDim; ++i) {st.data.push_back(std::sqrt(P(i, i)));}
    state_pub_->publish(st);
  }

  void log_status()
  {
    const auto & x = localizer_.ekf().x();
    const auto sd = localizer_.ekf().P().diagonal().cwiseSqrt();
    const auto & g = localizer_.gps_stats();
    const auto & vv = localizer_.vo_velocity_stats();
    const auto & vy = localizer_.vo_yaw_stats();
    const auto & z = localizer_.zaru_stats();
    RCLCPP_INFO(get_logger(),
      "t=%.1f p=(%.2f, %.2f, %.2f)±(%.2f, %.2f, %.2f) yaw=%.1f±%.1f deg b=%+.4f±%.4f | "
      "GPS ok/rej %zu/%zu%s  VOvel ok/rej %zu/%zu%s  VOyaw ok/rej %zu/%zu  ZARU %zu%s",
      localizer_.ekf().time(), x(PX), x(PY), x(PZ), sd(PX), sd(PY), sd(PZ),
      x(YAW) * 180.0 / M_PI, sd(YAW) * 180.0 / M_PI, x(BIAS), sd(BIAS),
      g.accepted, g.rejected, localizer_.gps_link().lost ? " [LOST]" : "",
      vv.accepted, vv.rejected, localizer_.vo_link().lost ? " [LOST]" : "",
      vy.accepted, vy.rejected,
      z.accepted, localizer_.resting() ? " (resting)" : "");
  }

  Localizer localizer_;
  std::string map_frame_, body_frame_, vo_frame_;
  int64_t publish_decimation_ = 1;
  std::size_t imu_count_ = 0;
  double status_period_ = 5.0;
  double next_status_ = 0.0;
  double time_jump_reset_s_ = 1.0;
  double last_gyro_ = 0.0;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr est_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
};

}  // namespace drone_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_localization::EstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
