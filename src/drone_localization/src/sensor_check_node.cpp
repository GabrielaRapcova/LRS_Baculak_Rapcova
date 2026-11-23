// §1 Sensor acquisition and characterisation.
//
// Subscribes to the three public localisation sensors with sensor-data QoS and
// reports, every `window_s` of *simulation* time (taken from IMU header
// stamps, so the window holds the same amount of data at any real-time factor):
//   * rate measured from header stamps (simulation time) and from arrival;
//   * frame_id / child_frame_id checks against the sensor contract;
//   * duplicate and out-of-order stamps;
//   * truth-free statistics that make the corruption visible:
//       - GPS sample mean / std over the window (noise while hovering),
//       - gyro Z mean / std over the window (bias while not turning),
//       - VO pose displacement since the window start and mean body speed
//         (drift while stationary).
//
// This node never subscribes to ground truth or MAVROS localisation.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace drone_localization
{

namespace
{

double stamp_seconds(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Welford running mean / variance for one scalar.
struct RunningStat
{
  std::size_t n = 0;
  double mean = 0.0;
  double m2 = 0.0;

  void add(double v)
  {
    ++n;
    const double d = v - mean;
    mean += d / static_cast<double>(n);
    m2 += d * (v - mean);
  }
  double stddev() const
  {
    return n > 1 ? std::sqrt(m2 / static_cast<double>(n - 1)) : 0.0;
  }
};

// Timing bookkeeping shared by every stream.
struct StreamStats
{
  std::size_t count = 0;
  std::size_t duplicates = 0;
  std::size_t out_of_order = 0;
  std::size_t frame_errors = 0;
  double first_stamp = std::numeric_limits<double>::quiet_NaN();
  double last_stamp = std::numeric_limits<double>::quiet_NaN();
  double max_gap = 0.0;
  rclcpp::Time first_arrival;
  rclcpp::Time last_arrival;
  bool have_arrival = false;

  // Returns false for duplicate / out-of-order stamps.
  bool add(double stamp, const rclcpp::Time & arrival)
  {
    if (!have_arrival) {
      first_arrival = arrival;
      have_arrival = true;
    }
    last_arrival = arrival;
    if (count > 0) {
      if (stamp == last_stamp) {
        ++duplicates;
        return false;
      }
      if (stamp < last_stamp) {
        ++out_of_order;
        return false;
      }
      max_gap = std::max(max_gap, stamp - last_stamp);
    } else {
      first_stamp = stamp;
    }
    last_stamp = stamp;
    ++count;
    return true;
  }

  double stamp_rate() const
  {
    const double span = last_stamp - first_stamp;
    return (count > 1 && span > 0.0) ?
           static_cast<double>(count - 1) / span : 0.0;
  }

  double arrival_rate() const
  {
    if (!have_arrival || count < 2) {return 0.0;}
    const double span = (last_arrival - first_arrival).seconds();
    return span > 0.0 ? static_cast<double>(count - 1) / span : 0.0;
  }
};

}  // namespace

class SensorCheckNode : public rclcpp::Node
{
public:
  SensorCheckNode()
  : Node("sensor_check_node")
  {
    gps_topic_ = declare_parameter<std::string>("gps_topic", "/localization/gps");
    vo_topic_ = declare_parameter<std::string>("vo_topic", "/localization/vo");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/localization/imu");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    vo_frame_ = declare_parameter<std::string>("vo_frame", "vo_odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    window_s_ = declare_parameter<double>("window_s", 10.0);
    const double stall_period =
      declare_parameter<double>("stall_warning_s", 10.0);

    const auto qos = rclcpp::SensorDataQoS();
    gps_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      gps_topic_, qos,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {on_gps(*m);});
    vo_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      vo_topic_, qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_vo(*m);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {on_imu(*m);});

    // Wall timer only to complain when no sensor data arrives at all.
    stall_timer_ = create_wall_timer(
      std::chrono::duration<double>(stall_period), [this]() {check_stall();});

    RCLCPP_INFO(get_logger(),
      "sensor_check: GPS=%s VO=%s IMU=%s (use_sim_time=%s)",
      gps_topic_.c_str(), vo_topic_.c_str(), imu_topic_.c_str(),
      get_parameter("use_sim_time").as_bool() ? "true" : "false");
  }

private:
  void check_frame(
    const char * sensor, const char * field, const std::string & got,
    const std::string & expected, StreamStats & stats)
  {
    if (got == expected) {return;}
    ++stats.frame_errors;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "%s %s is '%s', expected '%s'", sensor, field, got.c_str(), expected.c_str());
  }

  void on_gps(const geometry_msgs::msg::PoseWithCovarianceStamped & m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    check_frame("GPS", "frame_id", m.header.frame_id, map_frame_, gps_);
    if (!gps_.add(stamp_seconds(m.header.stamp), steady_clock_.now())) {return;}
    gps_x_.add(m.pose.pose.position.x);
    gps_y_.add(m.pose.pose.position.y);
    gps_z_.add(m.pose.pose.position.z);
    gps_sigma_ = {std::sqrt(m.pose.covariance[0]),
      std::sqrt(m.pose.covariance[7]), std::sqrt(m.pose.covariance[14])};
  }

  void on_vo(const nav_msgs::msg::Odometry & m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    check_frame("VO", "frame_id", m.header.frame_id, vo_frame_, vo_);
    check_frame("VO", "child_frame_id", m.child_frame_id, body_frame_, vo_);
    if (!vo_.add(stamp_seconds(m.header.stamp), steady_clock_.now())) {return;}
    const auto & p = m.pose.pose.position;
    const double yaw = yaw_from_quaternion(m.pose.pose.orientation);
    if (!vo_window_started_) {
      vo_start_ = {p.x, p.y, p.z};
      vo_start_yaw_ = yaw;
      vo_window_started_ = true;
    }
    vo_last_ = {p.x, p.y, p.z};
    vo_last_yaw_ = yaw;
    const auto & v = m.twist.twist.linear;  // body frame (base_link)
    vo_body_speed_.add(std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z));
  }

  void on_imu(const sensor_msgs::msg::Imu & m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    check_frame("IMU", "frame_id", m.header.frame_id, body_frame_, imu_);
    const double stamp = stamp_seconds(m.header.stamp);
    if (!imu_.add(stamp, steady_clock_.now())) {return;}
    gyro_z_.add(m.angular_velocity.z);
    gyro_var_ = m.angular_velocity_covariance[8];
    imu_orientation_flag_ = m.orientation_covariance[0];
    ++total_imu_;
    if (stamp - imu_.first_stamp >= window_s_) {
      report(stamp - imu_.first_stamp);
    }
  }

  void check_stall()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (total_imu_ == last_total_imu_) {
      RCLCPP_WARN(get_logger(),
        "no IMU data in the last wall-clock period: is the sensor simulator running?");
    }
    last_total_imu_ = total_imu_;
  }

  void report_stream(const char * name, const StreamStats & s)
  {
    RCLCPP_INFO(get_logger(),
      "  %-3s n=%-5zu stamp-rate=%6.2f Hz  arrival-rate=%6.2f Hz  max-gap=%.3f s  "
      "dup=%zu ooo=%zu frame-err=%zu",
      name, s.count, s.stamp_rate(), s.arrival_rate(), s.max_gap,
      s.duplicates, s.out_of_order, s.frame_errors);
  }

  // Called with mutex_ held.
  void report(double sim_span)
  {
    const double wall_span = (imu_.last_arrival - imu_.first_arrival).seconds();
    RCLCPP_INFO(get_logger(),
      "---- sensor window: %.1f s sim time in %.1f s wall (real-time factor %.2f) ----",
      sim_span, wall_span, wall_span > 0.0 ? sim_span / wall_span : 0.0);
    report_stream("GPS", gps_);
    report_stream("VO", vo_);
    report_stream("IMU", imu_);

    if (gps_x_.n > 1) {
      RCLCPP_INFO(get_logger(),
        "  GPS mean=(%.2f, %.2f, %.2f) std=(%.2f, %.2f, %.2f) m  "
        "[reported sigma=(%.2f, %.2f, %.2f)]",
        gps_x_.mean, gps_y_.mean, gps_z_.mean,
        gps_x_.stddev(), gps_y_.stddev(), gps_z_.stddev(),
        gps_sigma_[0], gps_sigma_[1], gps_sigma_[2]);
    }
    if (gyro_z_.n > 1) {
      RCLCPP_INFO(get_logger(),
        "  gyro_z mean=%+.4f rad/s (%+.2f deg/s) std=%.4f  "
        "[reported var=%.2e -> sigma=%.4f, orientation flag=%.0f]",
        gyro_z_.mean, gyro_z_.mean * 180.0 / M_PI, gyro_z_.stddev(),
        gyro_var_, std::sqrt(std::max(gyro_var_, 0.0)), imu_orientation_flag_);
    }
    if (vo_window_started_) {
      const double dx = vo_last_[0] - vo_start_[0];
      const double dy = vo_last_[1] - vo_start_[1];
      const double dz = vo_last_[2] - vo_start_[2];
      double dyaw = vo_last_yaw_ - vo_start_yaw_;
      dyaw = std::remainder(dyaw, 2.0 * M_PI);
      RCLCPP_INFO(get_logger(),
        "  VO displacement over window=(%+.3f, %+.3f, %+.3f) m, dyaw=%+.2f deg, "
        "mean |v_body|=%.3f m/s",
        dx, dy, dz, dyaw * 180.0 / M_PI, vo_body_speed_.mean);
    }
    RCLCPP_INFO(get_logger(),
      "  (while stationary: GPS std = noise, gyro mean = bias, VO displacement = drift)");

    // Start a fresh window so each report characterises the last period only.
    gps_ = vo_ = imu_ = StreamStats{};
    gps_x_ = gps_y_ = gps_z_ = gyro_z_ = vo_body_speed_ = RunningStat{};
    vo_window_started_ = false;
  }

  std::string gps_topic_, vo_topic_, imu_topic_;
  std::string map_frame_, vo_frame_, body_frame_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr stall_timer_;
  double window_s_ = 10.0;
  std::size_t total_imu_ = 0, last_total_imu_ = 0;

  // Arrival is measured on a steady wall clock, independent of /clock.
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::mutex mutex_;
  StreamStats gps_, vo_, imu_;
  RunningStat gps_x_, gps_y_, gps_z_, gyro_z_, vo_body_speed_;
  std::array<double, 3> gps_sigma_{0.0, 0.0, 0.0};
  double gyro_var_ = 0.0;
  double imu_orientation_flag_ = 0.0;
  bool vo_window_started_ = false;
  std::array<double, 3> vo_start_{}, vo_last_{};
  double vo_start_yaw_ = 0.0, vo_last_yaw_ = 0.0;
};

}  // namespace drone_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_localization::SensorCheckNode>());
  rclcpp::shutdown();
  return 0;
}
