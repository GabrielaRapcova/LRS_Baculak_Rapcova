// §4: fly the two-lap route using only the localisation estimate as feedback.
//
// Feedback: /localization/estimate (nav_msgs/Odometry, map ENU) — the only
//           position and yaw source. No MAVROS pose, no ground truth.
// MAVROS:   /mavros/state, set_mode / arming / takeoff services, and velocity +
//           yaw-rate setpoints on /mavros/setpoint_raw/local (ENU, MAVROS converts).
//
// Phases (no manual input after start):
//   WAIT_READY   FCU connected and a valid estimate; settle on the ground, then
//                take the current estimate as the mission origin
//   SET_GUIDED, ARM
//   TAKEOFF      MAVROS takeoff service; ends when the *estimated* height is
//                takeoff_reached_fraction of the altitude
//   CLIMB        velocity control to origin + (0, 0, altitude)
//   ROUTE        the waypoints of all laps
//   FINAL_HOLD   hover above the origin, then LAND and wait for the disarm

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "localization_flight/route_controller.hpp"

namespace localization_flight
{

using mavros_msgs::msg::PositionTarget;
using mavros_msgs::srv::CommandBool;
using mavros_msgs::srv::CommandTOL;
using mavros_msgs::srv::SetMode;

enum class Phase { WAIT_READY, SET_GUIDED, ARM, TAKEOFF, CLIMB, ROUTE, FINAL_HOLD, LAND,
  WAIT_DISARM, DONE };

const char * phase_name(Phase phase)
{
  static const char * names[] = {"WAIT_READY", "SET_GUIDED", "ARM", "TAKEOFF", "CLIMB",
    "ROUTE", "FINAL_HOLD", "LAND", "WAIT_DISARM", "DONE"};
  return names[static_cast<int>(phase)];
}

class EstimatedMissionNode : public rclcpp::Node
{
public:
  EstimatedMissionNode()
  : Node("estimated_mission")
  {
    const auto estimate_topic =
      declare_parameter<std::string>("estimate_topic", "/localization/estimate");
    const auto mavros = declare_parameter<std::string>("mavros_ns", "/mavros");

    altitude_ = declare_parameter("altitude", 2.0);
    laps_ = declare_parameter("laps", 2);
    const auto route = declare_parameter<std::vector<double>>(
      "route", {0.0, 0.0, 0.0, -4.0, 2.5, -4.0, 2.5, 0.0, 0.0, 0.0});
    for (std::size_t i = 0; i + 1 < route.size(); i += 2) {
      route_offsets_.emplace_back(route[i], route[i + 1]);
    }
    accept_xy_ = declare_parameter("accept_xy", 0.3);
    accept_z_ = declare_parameter("accept_z", 0.35);
    final_hold_ = declare_parameter("final_hold_s", 3.0);

    origin_max_sigma_ = declare_parameter("origin_max_sigma", 0.3);
    ground_settle_ = declare_parameter("ground_settle_s", 10.0);
    settle_max_bias_sigma_ = declare_parameter("settle_max_bias_sigma", 2.5e-4);
    takeoff_reached_fraction_ = declare_parameter("takeoff_reached_fraction", 0.7);
    takeoff_timeout_ = declare_parameter("takeoff_timeout_s", 60.0);
    climb_tolerance_ = declare_parameter("climb_tolerance", 0.25);

    estimate_timeout_ = declare_parameter("estimate_timeout_s", 0.5);
    estimate_lost_land_ = declare_parameter("estimate_lost_land_s", 3.0);
    max_position_sigma_ = declare_parameter("max_position_sigma", 2.0);
    mode_abort_after_ = declare_parameter("mode_abort_after_s", 1.5);
    status_period_ = declare_parameter("status_period_s", 2.0);
    const double setpoint_rate = declare_parameter("setpoint_rate_hz", 20.0);

    ControllerParams gains;
    gains.kp_xy = declare_parameter("kp_xy", gains.kp_xy);
    gains.max_speed_xy = declare_parameter("max_speed_xy", gains.max_speed_xy);
    gains.kp_z = declare_parameter("kp_z", gains.kp_z);
    gains.max_speed_z = declare_parameter("max_speed_z", gains.max_speed_z);
    gains.kp_yaw = declare_parameter("kp_yaw", gains.kp_yaw);
    gains.max_yaw_rate = declare_parameter("max_yaw_rate", gains.max_yaw_rate);
    controller_ = RouteController(gains);

    estimate_sub_ = create_subscription<nav_msgs::msg::Odometry>(estimate_topic, 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_estimate(*msg);});
    state_sub_ = create_subscription<mavros_msgs::msg::State>(mavros + "/state", 10,
        [this](mavros_msgs::msg::State::ConstSharedPtr msg) {on_state(*msg);});
    setpoint_pub_ = create_publisher<PositionTarget>(mavros + "/setpoint_raw/local", 10);
    mode_client_ = create_client<SetMode>(mavros + "/set_mode");
    arm_client_ = create_client<CommandBool>(mavros + "/cmd/arming");
    takeoff_client_ = create_client<CommandTOL>(mavros + "/cmd/takeoff");

    // Simulation-time timer, so the setpoint rate follows the SITL clock.
    timer_ = rclcpp::create_timer(this, get_clock(),
        rclcpp::Duration::from_seconds(1.0 / setpoint_rate), [this]() {tick();});

    RCLCPP_INFO(get_logger(),
      "estimated mission: feedback=%s only | %d laps, %zu route points, %.1f m, %.2f m/s",
      estimate_topic.c_str(), laps_, route_offsets_.size(), altitude_, gains.max_speed_xy);
  }

private:
  // ---- Inputs ------------------------------------------------------------------------

  void on_state(const mavros_msgs::msg::State & msg)
  {
    if (msg.connected != state_.connected || msg.armed != state_.armed ||
      msg.mode != state_.mode)
    {
      RCLCPP_INFO(get_logger(), "MAVROS state: connected=%d armed=%d mode='%s'",
        msg.connected, msg.armed, msg.mode.c_str());
    }
    state_ = msg;
  }

  void on_estimate(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose.position;
    position_ = Eigen::Vector3d(p.x, p.y, p.z);
    yaw_ = tf2::getYaw(msg.pose.pose.orientation);
    sigma_xy_ = std::sqrt(std::max(msg.pose.covariance[0], msg.pose.covariance[7]));
    bias_sigma_ = std::sqrt(std::max(msg.twist.covariance[35], 0.0));  // gyro-bias variance
    estimate_stamp_ = rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type());
    have_estimate_ = position_.allFinite() && std::isfinite(yaw_);
  }

  double estimate_age() const
  {
    return have_estimate_ ? (now() - estimate_stamp_).seconds() : 1e9;
  }

  // MAVROS publishes connected=0, armed=0, mode='' on a missed heartbeat; that is
  // "unknown", not "disarmed" or "another mode".
  bool known_disarmed() const
  {
    return state_.connected && !state_.mode.empty() && !state_.armed;
  }

  bool known_other_mode() const
  {
    return state_.connected && !state_.mode.empty() && state_.mode != "GUIDED";
  }

  // ---- Outputs -----------------------------------------------------------------------

  void send_velocity(const VelocityCommand & command)
  {
    PositionTarget setpoint;
    setpoint.header.stamp = now();
    setpoint.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;  // ENU in, MAVROS converts
    setpoint.type_mask = PositionTarget::IGNORE_PX | PositionTarget::IGNORE_PY |
      PositionTarget::IGNORE_PZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY |
      PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW;
    setpoint.velocity.x = command.velocity.x();
    setpoint.velocity.y = command.velocity.y();
    setpoint.velocity.z = command.velocity.z();
    setpoint.yaw_rate = static_cast<float>(command.yaw_rate);
    setpoint_pub_->publish(setpoint);
    last_command_ = command;
  }

  // Flies toward `target` on the estimate, or holds / lands if it is unusable.
  void fly_to(const Eigen::Vector3d & target)
  {
    const double age = estimate_age();
    if (age > estimate_lost_land_) {
      land("estimate lost");
    } else if (sigma_xy_ > max_position_sigma_) {
      land("estimate covariance diverged");
    } else if (age > estimate_timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "estimate stale (%.2f s): holding with zero velocity", age);
      send_velocity(VelocityCommand{});
    } else {
      send_velocity(controller_.compute(position_, yaw_, target, origin_yaw_));
    }
  }

  // Service requests are retried every `period` seconds until the phase changes.
  bool time_for_request(double period)
  {
    if ((now() - last_request_).seconds() < period) {
      return false;
    }
    last_request_ = now();
    return true;
  }

  void request_mode(const std::string & mode)
  {
    if (!mode_client_->service_is_ready() || !time_for_request(2.0)) {
      return;
    }
    auto request = std::make_shared<SetMode::Request>();
    request->custom_mode = mode;
    mode_client_->async_send_request(request);  // the result shows up on /mavros/state
  }

  void request_arm()
  {
    if (!arm_client_->service_is_ready() || !time_for_request(3.0)) {
      return;
    }
    auto request = std::make_shared<CommandBool::Request>();
    request->value = true;
    arm_client_->async_send_request(request,
      [this](rclcpp::Client<CommandBool>::SharedFuture response) {
        if (!response.get()->success) {
          RCLCPP_WARN(get_logger(),
            "arming rejected by the FCU (usually pre-arm checks: ArduPilot's own EKF "
            "is not using GPS yet, see 'Arm: ...' in the MAVROS console); retrying");
        }
      });
  }

  void request_takeoff()
  {
    if (takeoff_accepted_ || !takeoff_client_->service_is_ready() || !time_for_request(5.0)) {
      return;
    }
    auto request = std::make_shared<CommandTOL::Request>();
    request->altitude = static_cast<float>(altitude_);
    takeoff_client_->async_send_request(request,
      [this](rclcpp::Client<CommandTOL>::SharedFuture response) {
        takeoff_accepted_ = response.get()->success;
        RCLCPP_INFO(get_logger(), "takeoff command %s",
          takeoff_accepted_ ? "accepted" : "rejected, retrying");
      });
  }

  // ---- Phases ------------------------------------------------------------------------

  void enter(Phase next)
  {
    RCLCPP_INFO(get_logger(), "phase %s -> %s (t=%.1f, est p=(%.2f, %.2f, %.2f) yaw=%.1f deg)",
      phase_name(phase_), phase_name(next), now().seconds(),
      position_.x(), position_.y(), position_.z(), yaw_ * 180.0 / M_PI);
    phase_ = next;
    phase_start_ = now();
    last_request_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  double time_in_phase() const {return (now() - phase_start_).seconds();}

  void land(const char * reason)
  {
    RCLCPP_WARN(get_logger(), "LANDING: %s", reason);
    enter(Phase::LAND);
  }

  void wait_ready()
  {
    if (!state_.connected) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for FCU connection");
      return;
    }
    if (estimate_age() > estimate_timeout_ || sigma_xy_ > origin_max_sigma_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "waiting for a valid estimate (sigma_xy %.2f m, need < %.2f)",
        sigma_xy_, origin_max_sigma_);
      return;
    }

    // Settle on the ground so the estimator pins the gyro bias (yaw then holds
    // while hovering, when nothing else observes it).
    if (!settle_start_) {
      settle_start_ = now();
    }
    const double settled = (now() - *settle_start_).seconds();
    const bool bias_known = bias_sigma_ > 0.0 && bias_sigma_ < settle_max_bias_sigma_;
    if (settled < ground_settle_ || (!bias_known && settled < 4.0 * ground_settle_)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "settling on the ground: %.1f / %.0f s, gyro bias sigma %.5f rad/s (need < %.5f)",
        settled, ground_settle_, bias_sigma_, settle_max_bias_sigma_);
      return;
    }

    origin_ = position_;
    origin_yaw_ = yaw_;
    mission_.emplace(build_route(origin_, altitude_, laps_, route_offsets_),
      accept_xy_, accept_z_);
    RCLCPP_INFO(get_logger(),
      "MISSION ORIGIN from estimate: (%.2f, %.2f, %.2f) m, yaw %.1f deg (sigma_xy %.2f m, "
      "bias sigma %.5f rad/s); %zu waypoints", origin_.x(), origin_.y(), origin_.z(),
      origin_yaw_ * 180.0 / M_PI, sigma_xy_, bias_sigma_, mission_->size());
    enter(state_.mode == "GUIDED" ? Phase::ARM : Phase::SET_GUIDED);
  }

  void set_guided()
  {
    if (state_.mode == "GUIDED") {
      enter(Phase::ARM);
    } else {
      request_mode("GUIDED");
    }
  }

  void arm()
  {
    if (state_.armed) {
      enter(Phase::TAKEOFF);
    } else if (time_in_phase() > 300.0) {
      RCLCPP_ERROR(get_logger(), "arming timed out after 300 s");
      enter(Phase::DONE);
    } else {
      request_arm();
    }
  }

  void takeoff()
  {
    if (!mission_start_) {
      mission_start_ = now();
    }
    // No setpoints during the takeoff command: ArduPilot climbs on its own.
    if (position_.z() - origin_.z() >= takeoff_reached_fraction_ * altitude_) {
      enter(Phase::CLIMB);
    } else if (time_in_phase() > takeoff_timeout_) {
      land("takeoff timeout");
    } else if (known_disarmed() && time_in_phase() > 5.0) {
      RCLCPP_ERROR(get_logger(), "disarmed during takeoff");
      enter(Phase::DONE);
    } else {
      request_takeoff();
    }
  }

  void climb()
  {
    const Eigen::Vector3d target = origin_ + Eigen::Vector3d(0.0, 0.0, altitude_);
    fly_to(target);
    const bool at_altitude = std::abs(position_.z() - target.z()) < climb_tolerance_;
    const bool above_origin = (position_ - target).head<2>().norm() < accept_xy_;
    if (phase_ == Phase::CLIMB && at_altitude && above_origin) {
      enter(Phase::ROUTE);
    }
  }

  void follow_route()
  {
    if (mission_->update(position_)) {
      RCLCPP_INFO(get_logger(), "REACHED %s at t=%.1f: est (%.2f, %.2f, %.2f) [%zu/%zu]",
        mission_->previous().label.c_str(), now().seconds(),
        position_.x(), position_.y(), position_.z(), mission_->index(), mission_->size());
    }
    if (mission_->done()) {
      enter(Phase::FINAL_HOLD);
    } else {
      fly_to(mission_->current().position);
    }
  }

  void final_hold()
  {
    fly_to(origin_ + Eigen::Vector3d(0.0, 0.0, altitude_));
    if (phase_ == Phase::FINAL_HOLD && time_in_phase() >= final_hold_) {
      enter(Phase::LAND);
    }
  }

  void land_mode()
  {
    if (state_.mode == "LAND") {
      enter(Phase::WAIT_DISARM);
    } else {
      request_mode("LAND");
    }
  }

  void wait_disarm()
  {
    if (!known_disarmed()) {
      return;
    }
    const Eigen::Vector2d offset = (position_ - origin_).head<2>();
    RCLCPP_INFO(get_logger(),
      "MISSION COMPLETE: landed and disarmed at est offset (%.2f, %.2f) m from origin, "
      "flight time %.1f s", offset.x(), offset.y(), (now() - *mission_start_).seconds());
    enter(Phase::DONE);
  }

  // A real other mode (not an empty/unknown one) for a while = pilot takeover.
  void check_takeover()
  {
    const bool flying = phase_ == Phase::TAKEOFF || phase_ == Phase::CLIMB ||
      phase_ == Phase::ROUTE || phase_ == Phase::FINAL_HOLD;
    if (!flying || !known_other_mode() || time_in_phase() < 2.0) {
      other_mode_since_.reset();
      return;
    }
    if (!other_mode_since_) {
      other_mode_since_ = now();
    }
    if ((now() - *other_mode_since_).seconds() > mode_abort_after_) {
      RCLCPP_ERROR(get_logger(),
        "FCU in mode '%s' (not GUIDED): pilot/FCU takeover, stopping setpoints",
        state_.mode.c_str());
      enter(Phase::DONE);
    }
  }

  void tick()
  {
    check_takeover();
    switch (phase_) {
      case Phase::WAIT_READY: wait_ready(); break;
      case Phase::SET_GUIDED: set_guided(); break;
      case Phase::ARM: arm(); break;
      case Phase::TAKEOFF: takeoff(); break;
      case Phase::CLIMB: climb(); break;
      case Phase::ROUTE: follow_route(); break;
      case Phase::FINAL_HOLD: final_hold(); break;
      case Phase::LAND: land_mode(); break;
      case Phase::WAIT_DISARM: wait_disarm(); break;
      case Phase::DONE: break;
    }
    log_status();
  }

  void log_status()
  {
    if (phase_ == Phase::WAIT_READY || phase_ == Phase::DONE ||
      (now() - last_status_).seconds() < status_period_)
    {
      return;
    }
    last_status_ = now();
    const Eigen::Vector3d relative = position_ - origin_;
    std::string target = "-";
    if (phase_ == Phase::ROUTE) {
      const Eigen::Vector3d t = mission_->current().position - origin_;
      const double distance = (mission_->current().position - position_).head<2>().norm();
      char text[96];
      std::snprintf(text, sizeof(text), "(%.1f, %.1f, %.1f) dist %.2f m",
        t.x(), t.y(), t.z(), distance);
      target = text;
    }
    RCLCPP_INFO(get_logger(),
      "[%s] est rel (%.2f, %.2f, %.2f) yaw %.1f sigma %.2f m | target %s | "
      "cmd v=(%.2f, %.2f, %.2f) yawrate %.2f | mode %s armed %d",
      phase_name(phase_), relative.x(), relative.y(), relative.z(), yaw_ * 180.0 / M_PI,
      sigma_xy_, target.c_str(), last_command_.velocity.x(), last_command_.velocity.y(),
      last_command_.velocity.z(), last_command_.yaw_rate, state_.mode.c_str(), state_.armed);
  }

  // Parameters.
  double altitude_, accept_xy_, accept_z_, final_hold_;
  double origin_max_sigma_, ground_settle_, settle_max_bias_sigma_;
  double takeoff_reached_fraction_, takeoff_timeout_, climb_tolerance_;
  double estimate_timeout_, estimate_lost_land_, max_position_sigma_;
  double mode_abort_after_, status_period_;
  int laps_;
  std::vector<Eigen::Vector2d> route_offsets_;
  RouteController controller_;

  // Latest estimate.
  bool have_estimate_ = false;
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
  double yaw_ = 0.0;
  double sigma_xy_ = 1e9;
  double bias_sigma_ = 0.0;
  rclcpp::Time estimate_stamp_;

  // Mission state.
  mavros_msgs::msg::State state_;
  Phase phase_ = Phase::WAIT_READY;
  rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_request_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> settle_start_, mission_start_, other_mode_since_;
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
  double origin_yaw_ = 0.0;
  std::optional<Mission> mission_;
  bool takeoff_accepted_ = false;
  VelocityCommand last_command_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr estimate_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Client<SetMode>::SharedPtr mode_client_;
  rclcpp::Client<CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace localization_flight

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<localization_flight::EstimatedMissionNode>());
  rclcpp::shutdown();
  return 0;
}
