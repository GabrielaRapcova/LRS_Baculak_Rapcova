// §1 live sensor check.
//
// Subscribes to the three localisation sensors and, every `window_s` of
// simulation time (measured on IMU stamps), reports per sensor:
//   * rate from header stamps and from wall-clock arrival, largest stamp gap,
//     out-of-order stamps and unexpected frame ids;
//   * truth-free statistics that show the corruption while the drone is still:
//     GPS spread (noise), mean gyro Z (bias), VO displacement (drift).
// It never subscribes to ground truth or MAVROS.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace drone_localization
{

// Mean and standard deviation of a series of values.
struct Statistic
{
  int count = 0;
  double sum = 0.0;
  double sum_of_squares = 0.0;

  void add(double value)
  {
    ++count;
    sum += value;
    sum_of_squares += value * value;
  }
  double mean() const {return count > 0 ? sum / count : 0.0;}
  double stddev() const
  {
    if (count < 2) {
      return 0.0;
    }
    const double variance = (sum_of_squares - sum * sum / count) / (count - 1);
    return std::sqrt(std::max(variance, 0.0));
  }
};

// Timing of one sensor stream over the current window.
struct StreamTiming
{
  int count = 0;
  int out_of_order = 0;
  int frame_errors = 0;
  double first_stamp = 0.0;
  double last_stamp = 0.0;
  double max_gap = 0.0;
  double first_arrival = 0.0;  // steady clock [s]
  double last_arrival = 0.0;

  // Returns false for a stamp that is not newer than the previous one.
  bool add(double stamp, double arrival)
  {
    if (count > 0 && stamp <= last_stamp) {
      ++out_of_order;
      return false;
    }
    if (count == 0) {
      first_stamp = stamp;
      first_arrival = arrival;
    } else {
      max_gap = std::max(max_gap, stamp - last_stamp);
    }
    last_stamp = stamp;
    last_arrival = arrival;
    ++count;
    return true;
  }
  double stamp_rate() const
  {
    return (count > 1) ? (count - 1) / (last_stamp - first_stamp) : 0.0;
  }
  double arrival_rate() const
  {
    const double span = last_arrival - first_arrival;
    return (count > 1 && span > 0.0) ? (count - 1) / span : 0.0;
  }
};

class SensorCheckNode : public rclcpp::Node
{
public:
  SensorCheckNode()
  : Node("sensor_check_node")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    vo_frame_ = declare_parameter<std::string>("vo_frame", "vo_odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    window_s_ = declare_parameter("window_s", 10.0);

    const auto qos = rclcpp::SensorDataQoS();
    gps_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/gps", qos,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {on_gps(*msg);});
    vo_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/vo", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_vo(*msg);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/localization/imu", qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {on_imu(*msg);});

    // Wall timer only to complain when no data arrives at all.
    stall_timer_ = create_wall_timer(std::chrono::seconds(10), [this]() {
          if (imu_messages_ == imu_messages_at_last_check_) {
            RCLCPP_WARN(get_logger(), "no IMU data: is the sensor simulator running?");
          }
          imu_messages_at_last_check_ = imu_messages_;
        });
    RCLCPP_INFO(get_logger(), "sensor check: reporting every %.0f s of simulation time",
      window_s_);
  }

private:
  double arrival_seconds() const {return steady_clock_.now().seconds();}

  void check_frame(const std::string & actual, const std::string & expected, StreamTiming & s)
  {
    if (actual != expected) {
      ++s.frame_errors;
    }
  }

  void on_gps(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    check_frame(msg.header.frame_id, map_frame_, gps_);
    if (!gps_.add(rclcpp::Time(msg.header.stamp).seconds(), arrival_seconds())) {
      return;
    }
    gps_x_.add(msg.pose.pose.position.x);
    gps_y_.add(msg.pose.pose.position.y);
    gps_z_.add(msg.pose.pose.position.z);
  }

  void on_vo(const nav_msgs::msg::Odometry & msg)
  {
    check_frame(msg.header.frame_id, vo_frame_, vo_);
    check_frame(msg.child_frame_id, body_frame_, vo_);
    if (!vo_.add(rclcpp::Time(msg.header.stamp).seconds(), arrival_seconds())) {
      return;
    }
    const auto & p = msg.pose.pose.position;
    const double yaw = tf2::getYaw(msg.pose.pose.orientation);
    if (vo_.count == 1) {
      vo_start_ = {p.x, p.y, p.z, yaw};
    }
    vo_end_ = {p.x, p.y, p.z, yaw};
    const auto & v = msg.twist.twist.linear;
    vo_speed_.add(std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z));
  }

  void on_imu(const sensor_msgs::msg::Imu & msg)
  {
    check_frame(msg.header.frame_id, body_frame_, imu_);
    ++imu_messages_;
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    if (!imu_.add(stamp, arrival_seconds())) {
      return;
    }
    gyro_z_.add(msg.angular_velocity.z);
    if (stamp - imu_.first_stamp >= window_s_) {
      report();
      reset_window();
    }
  }

  void report_stream(const char * name, const StreamTiming & s)
  {
    RCLCPP_INFO(get_logger(),
      "  %-3s n=%-5d stamp-rate=%6.2f Hz  arrival-rate=%6.2f Hz  max-gap=%.3f s  "
      "out-of-order=%d frame-errors=%d", name, s.count, s.stamp_rate(), s.arrival_rate(),
      s.max_gap, s.out_of_order, s.frame_errors);
  }

  void report()
  {
    const double sim_span = imu_.last_stamp - imu_.first_stamp;
    const double wall_span = imu_.last_arrival - imu_.first_arrival;
    RCLCPP_INFO(get_logger(),
      "---- %.1f s simulation time in %.1f s wall time (real-time factor %.2f) ----",
      sim_span, wall_span, wall_span > 0.0 ? sim_span / wall_span : 0.0);
    report_stream("GPS", gps_);
    report_stream("VO", vo_);
    report_stream("IMU", imu_);
    RCLCPP_INFO(get_logger(), "  GPS mean=(%.2f, %.2f, %.2f) std=(%.2f, %.2f, %.2f) m",
      gps_x_.mean(), gps_y_.mean(), gps_z_.mean(),
      gps_x_.stddev(), gps_y_.stddev(), gps_z_.stddev());
    RCLCPP_INFO(get_logger(), "  gyro_z mean=%+.4f rad/s (%+.2f deg/s) std=%.4f rad/s",
      gyro_z_.mean(), gyro_z_.mean() * 180.0 / M_PI, gyro_z_.stddev());
    RCLCPP_INFO(get_logger(),
      "  VO displacement=(%+.3f, %+.3f, %+.3f) m, yaw change=%+.2f deg, mean |v|=%.3f m/s",
      vo_end_[0] - vo_start_[0], vo_end_[1] - vo_start_[1], vo_end_[2] - vo_start_[2],
      std::remainder(vo_end_[3] - vo_start_[3], 2.0 * M_PI) * 180.0 / M_PI, vo_speed_.mean());
    RCLCPP_INFO(get_logger(),
      "  (drone still: GPS std = noise, gyro mean = bias, VO displacement = drift)");
  }

  void reset_window()
  {
    gps_ = vo_ = imu_ = StreamTiming{};
    gps_x_ = gps_y_ = gps_z_ = gyro_z_ = vo_speed_ = Statistic{};
  }

  std::string map_frame_, vo_frame_, body_frame_;
  double window_s_ = 10.0;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  StreamTiming gps_, vo_, imu_;
  Statistic gps_x_, gps_y_, gps_z_, gyro_z_, vo_speed_;
  std::array<double, 4> vo_start_{}, vo_end_{};  // x, y, z, yaw
  long imu_messages_ = 0;  // total, for the stall warning
  long imu_messages_at_last_check_ = 0;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr stall_timer_;
};

}  // namespace drone_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_localization::SensorCheckNode>());
  rclcpp::shutdown();
  return 0;
}
