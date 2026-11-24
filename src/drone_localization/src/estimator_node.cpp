// ROS 2 node around the Localizer (eight-state EKF).
//
// Subscribes only to the three localisation sensors:
//   /localization/gps  geometry_msgs/PoseWithCovarianceStamped  (frame map)
//   /localization/vo   nav_msgs/Odometry                        (vo_odom -> base_link)
//   /localization/imu  sensor_msgs/Imu                          (angular_velocity.z)
// Publishes:
//   /localization/estimate        nav_msgs/Odometry, map -> base_link, body-frame twist
//   /localization/estimate_state  std_msgs/Float64MultiArray
//                                 [t, px, py, pz, vx, vy, vz, yaw, bias, 8 sigmas]

#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "drone_localization/localizer.hpp"

namespace drone_localization
{

class EstimatorNode : public rclcpp::Node
{
public:
  EstimatorNode()
  : Node("localization_estimator"),
    localizer_(load_config(), get_logger())
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    vo_frame_ = declare_parameter<std::string>("vo_frame", "vo_odom");
    status_period_ = declare_parameter("status_period_s", 5.0);
    const auto estimate_topic =
      declare_parameter<std::string>("estimate_topic", "/localization/estimate");

    const auto sensor_qos = rclcpp::SensorDataQoS();
    gps_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      declare_parameter<std::string>("gps_topic", "/localization/gps"), sensor_qos,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {on_gps(*msg);});
    vo_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("vo_topic", "/localization/vo"), sensor_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_vo(*msg);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("imu_topic", "/localization/imu"), sensor_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {on_imu(*msg);});

    estimate_pub_ = create_publisher<nav_msgs::msg::Odometry>(estimate_topic, 10);
    state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(estimate_topic + "_state", 10);

    const auto & c = localizer_.config();
    RCLCPP_INFO(get_logger(),
      "estimator up: Q accel_xy=%.2f accel_z=%.2f gyro=%.4f bias_walk=%.1e | "
      "gates chi2 %.2f (1 DOF) / %.2f (3 DOF) | VO velocity sigma %.2f m/s | "
      "waiting for %d GPS fixes to initialise",
      c.noise.accel_xy, c.noise.accel_z, c.noise.gyro_noise, c.noise.bias_walk,
      c.gate_chi2_1dof, c.gate_chi2_3dof, c.vo_velocity_sigma, c.init_gps_samples);
  }

private:
  LocalizerConfig load_config()
  {
    LocalizerConfig c;
    // Process noise.
    c.noise.accel_xy = declare_parameter("accel_xy", c.noise.accel_xy);
    c.noise.accel_z = declare_parameter("accel_z", c.noise.accel_z);
    c.noise.gyro_noise = declare_parameter("gyro_noise", c.noise.gyro_noise);
    c.noise.gyro_rate = declare_parameter("gyro_rate", c.noise.gyro_rate);
    c.noise.bias_walk = declare_parameter("bias_walk", c.noise.bias_walk);
    // Initialisation.
    c.init_gps_samples = declare_parameter("init_gps_samples", c.init_gps_samples);
    c.init_yaw = declare_parameter("init_yaw", c.init_yaw);
    c.init_yaw_sigma = declare_parameter("init_yaw_sigma", c.init_yaw_sigma);
    c.init_velocity_sigma = declare_parameter("init_velocity_sigma", c.init_velocity_sigma);
    c.init_bias_sigma = declare_parameter("init_bias_sigma", c.init_bias_sigma);
    c.init_bias_min_sigma = declare_parameter("init_bias_min_sigma", c.init_bias_min_sigma);
    // Timing and gates.
    c.max_measurement_lag = declare_parameter("max_measurement_lag", c.max_measurement_lag);
    c.gate_chi2_1dof = declare_parameter("gate_chi2_1dof", c.gate_chi2_1dof);
    c.gate_chi2_3dof = declare_parameter("gate_chi2_3dof", c.gate_chi2_3dof);
    c.gps_max_consecutive_rejections =
      declare_parameter("gps_max_consecutive_rejections", c.gps_max_consecutive_rejections);
    // VO.
    c.vo_velocity_sigma = declare_parameter("vo_velocity_sigma", c.vo_velocity_sigma);
    c.vo_yaw_window = declare_parameter("vo_yaw_window", c.vo_yaw_window);
    c.vo_pose_yaw_sigma = declare_parameter("vo_pose_yaw_sigma", c.vo_pose_yaw_sigma);
    c.vo_yaw_rate_sigma = declare_parameter("vo_yaw_rate_sigma", c.vo_yaw_rate_sigma);
    // Resting on the ground.
    c.rest_enter_height = declare_parameter("rest_enter_height", c.rest_enter_height);
    c.rest_exit_height = declare_parameter("rest_exit_height", c.rest_exit_height);
    c.rest_enter_speed = declare_parameter("rest_enter_speed", c.rest_enter_speed);
    c.rest_exit_speed = declare_parameter("rest_exit_speed", c.rest_exit_speed);
    c.rest_enter_time = declare_parameter("rest_enter_time", c.rest_enter_time);
    c.rest_update_period = declare_parameter("rest_update_period", c.rest_update_period);
    c.zaru_rate_sigma = declare_parameter("zaru_rate_sigma", c.zaru_rate_sigma);
    c.zupt_sigma = declare_parameter("zupt_sigma", c.zupt_sigma);
    // Sensor timeouts.
    c.gps_timeout = declare_parameter("gps_timeout", c.gps_timeout);
    c.vo_timeout = declare_parameter("vo_timeout", c.vo_timeout);
    c.imu_timeout = declare_parameter("imu_timeout", c.imu_timeout);
    return c;
  }

  bool has_frame(const std::string & actual, const std::string & expected, const char * what)
  {
    if (actual == expected) {
      return true;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "ignoring %s with frame '%s' (expected '%s')", what, actual.c_str(), expected.c_str());
    return false;
  }

  void on_gps(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    if (!has_frame(msg.header.frame_id, map_frame_, "GPS")) {
      return;
    }
    const auto & p = msg.pose.pose.position;
    // Top-left 3x3 (x, y, z) block of the row-major 6x6 pose covariance.
    const Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance(
      msg.pose.covariance.data());
    localizer_.on_gps(rclcpp::Time(msg.header.stamp).seconds(),
      Eigen::Vector3d(p.x, p.y, p.z), covariance.topLeftCorner<3, 3>());
  }

  void on_vo(const nav_msgs::msg::Odometry & msg)
  {
    if (!has_frame(msg.header.frame_id, vo_frame_, "VO") ||
      !has_frame(msg.child_frame_id, body_frame_, "VO child"))
    {
      return;
    }
    const auto & v = msg.twist.twist.linear;  // base_link frame
    const auto & c = msg.twist.covariance;
    localizer_.on_vo(rclcpp::Time(msg.header.stamp).seconds(),
      tf2::getYaw(msg.pose.pose.orientation),
      Eigen::Vector3d(v.x, v.y, v.z), Eigen::Vector3d(c[0], c[7], c[14]));
  }

  void on_imu(const sensor_msgs::msg::Imu & msg)
  {
    const double t = rclcpp::Time(msg.header.stamp).seconds();
    if (localizer_.initialized() && t < localizer_.ekf().time() - 1.0) {
      // Simulation time jumped back (Gazebo reset): the old state is meaningless.
      RCLCPP_WARN(get_logger(), "simulation time jumped back %.1f s; re-initialising",
        localizer_.ekf().time() - t);
      localizer_ = Localizer(localizer_.config(), get_logger());
    }

    const double time_before = localizer_.ekf().time();
    localizer_.on_imu(t, msg.angular_velocity.z);
    gyro_z_ = msg.angular_velocity.z;
    if (!localizer_.initialized() || localizer_.ekf().time() <= time_before) {
      return;
    }
    publish_estimate();
    if (t >= next_status_time_) {
      next_status_time_ = t + status_period_;
      log_status();
    }
  }

  void publish_estimate()
  {
    const auto & x = localizer_.ekf().x();
    const auto & P = localizer_.ekf().P();
    const double t = localizer_.ekf().time();
    const double yaw = x(YAW);

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(t * 1e9)), RCL_ROS_TIME);
    msg.header.frame_id = map_frame_;
    msg.child_frame_id = body_frame_;
    msg.pose.pose.position.x = x(PX);
    msg.pose.pose.position.y = x(PY);
    msg.pose.pose.position.z = x(PZ);
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);
    msg.pose.pose.orientation = tf2::toMsg(orientation);

    // Pose covariance rows/columns: x y z roll pitch yaw (roll/pitch not estimated).
    const int state_index[4] = {PX, PY, PZ, YAW};
    const int pose_index[4] = {0, 1, 2, 5};
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        msg.pose.covariance[pose_index[i] * 6 + pose_index[j]] = P(state_index[i], state_index[j]);
      }
    }
    msg.pose.covariance[3 * 6 + 3] = 1e6;
    msg.pose.covariance[4 * 6 + 4] = 1e6;

    // Odometry convention: twist in the child (body) frame.
    const Eigen::Vector3d body_velocity = Ekf::body_velocity_model(x);
    msg.twist.twist.linear.x = body_velocity.x();
    msg.twist.twist.linear.y = body_velocity.y();
    msg.twist.twist.linear.z = body_velocity.z();
    msg.twist.twist.angular.z = gyro_z_ - x(BIAS);
    for (int i = 0; i < 3; ++i) {
      msg.twist.covariance[i * 7] = P(VX + i, VX + i);
    }
    msg.twist.covariance[35] = P(BIAS, BIAS);  // read by the flight node
    estimate_pub_->publish(msg);

    std_msgs::msg::Float64MultiArray state;
    state.data.push_back(t);
    for (int i = 0; i < kStateSize; ++i) {
      state.data.push_back(x(i));
    }
    for (int i = 0; i < kStateSize; ++i) {
      state.data.push_back(std::sqrt(P(i, i)));
    }
    state_pub_->publish(state);
  }

  void log_status()
  {
    const auto & x = localizer_.ekf().x();
    const Eigen::VectorXd sigma = localizer_.ekf().P().diagonal().cwiseSqrt();
    const auto & gps = localizer_.gps_stats();
    const auto & vo = localizer_.vo_stats();
    RCLCPP_INFO(get_logger(),
      "t=%.1f p=(%.2f, %.2f, %.2f)±(%.2f, %.2f, %.2f) yaw=%.1f±%.1f deg b=%+.4f±%.4f | "
      "GPS ok/rej %d/%d%s  VO ok/rej %d/%d%s  VO-yaw %d  ZARU %d%s",
      localizer_.ekf().time(), x(PX), x(PY), x(PZ), sigma(PX), sigma(PY), sigma(PZ),
      x(YAW) * 180.0 / M_PI, sigma(YAW) * 180.0 / M_PI, x(BIAS), sigma(BIAS),
      gps.accepted, gps.rejected, localizer_.gps_link().lost ? " [LOST]" : "",
      vo.accepted, vo.rejected, localizer_.vo_link().lost ? " [LOST]" : "",
      localizer_.vo_yaw_stats().accepted, localizer_.zaru_stats().accepted,
      localizer_.resting() ? " (resting)" : "");
  }

  Localizer localizer_;
  std::string map_frame_, body_frame_, vo_frame_;
  double status_period_ = 5.0;
  double next_status_time_ = 0.0;
  double gyro_z_ = 0.0;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimate_pub_;
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
