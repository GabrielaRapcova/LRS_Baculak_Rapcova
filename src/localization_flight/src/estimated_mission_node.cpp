// §4 flight using the estimated localisation only.
//
// Feedback:   /localization/estimate (nav_msgs/Odometry, map ENU) — the ONLY
//             position/yaw source. No MAVROS pose/odometry/IMU, no truth.
// MAVROS:     /mavros/state (connected/armed/mode), set_mode, arming and
//             takeoff services, and velocity + yaw-rate setpoints on
//             /mavros/setpoint_raw/local (ENU in, MAVROS converts to NED).
//
// Sequence (no manual input after start):
//   WAIT_READY  FCU connected and a valid estimate (sigma_xy small), then
//               settle on the ground (ground_settle_s, bias sigma small)
//               -> capture mission origin (x, y, z, yaw) from the estimate
//   SET_GUIDED, ARM
//   TAKEOFF     MAVROS takeoff service; done when the *estimated* height
//               reaches takeoff_reached_fraction of the target
//   CLIMB       velocity control to origin + (0, 0, altitude)
//   ROUTE       two laps of the relative route at ~0.5 m/s
//   FINAL_HOLD  hover above the origin for final_hold_s
//   LAND        LAND mode, wait until the FCU disarms
//
// Watchdogs: a stale estimate (> estimate_timeout_s) commands zero velocity;
// a lost estimate (> estimate_lost_land_s) or a diverged covariance lands.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <cstdio>
#include <type_traits>
#include <string>
#include <vector>

#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "localization_flight/route_controller.hpp"

namespace localization_flight
{

namespace
{

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

enum class Phase { WAIT_READY, SET_GUIDED, ARM, TAKEOFF, CLIMB, ROUTE, FINAL_HOLD, LAND,
  WAIT_DISARM, DONE };

const char * phase_name(Phase p)
{
  switch (p) {
    case Phase::WAIT_READY: return "WAIT_READY";
    case Phase::SET_GUIDED: return "SET_GUIDED";
    case Phase::ARM: return "ARM";
    case Phase::TAKEOFF: return "TAKEOFF";
    case Phase::CLIMB: return "CLIMB";
    case Phase::ROUTE: return "ROUTE";
    case Phase::FINAL_HOLD: return "FINAL_HOLD";
    case Phase::LAND: return "LAND";
    case Phase::WAIT_DISARM: return "WAIT_DISARM";
    case Phase::DONE: return "DONE";
  }
  return "?";
}

class EstimatedMissionNode : public rclcpp::Node
{
public:
  EstimatedMissionNode()
  : Node("estimated_mission")
  {
    const auto est_topic =
      declare_parameter<std::string>("estimate_topic", "/localization/estimate");
    const auto ns = declare_parameter<std::string>("mavros_ns", "/mavros");
    altitude_ = declare_parameter<double>("altitude", 2.0);
    takeoff_reached_fraction_ = declare_parameter<double>("takeoff_reached_fraction", 0.7);
    takeoff_timeout_ = declare_parameter<double>("takeoff_timeout_s", 60.0);
    climb_tolerance_ = declare_parameter<double>("climb_tolerance", 0.25);
    origin_max_sigma_ = declare_parameter<double>("origin_max_sigma", 0.3);
    ground_settle_ = declare_parameter<double>("ground_settle_s", 10.0);
    settle_max_bias_sigma_ = declare_parameter<double>("settle_max_bias_sigma", 2.5e-4);
    mode_abort_after_ = declare_parameter<double>("mode_abort_after_s", 1.5);
    laps_ = static_cast<int>(declare_parameter<int64_t>("laps", 2));
    const auto flat = declare_parameter<std::vector<double>>(
      "route", {0.0, 0.0, 0.0, -4.0, 2.5, -4.0, 2.5, 0.0, 0.0, 0.0});
    for (std::size_t i = 0; i + 1 < flat.size(); i += 2) {offsets_.emplace_back(flat[i], flat[i + 1]);}
    accept_xy_ = declare_parameter<double>("accept_xy", 0.3);
    accept_z_ = declare_parameter<double>("accept_z", 0.35);
    final_hold_ = declare_parameter<double>("final_hold_s", 3.0);
    estimate_timeout_ = declare_parameter<double>("estimate_timeout_s", 0.5);
    estimate_lost_land_ = declare_parameter<double>("estimate_lost_land_s", 3.0);
    max_position_sigma_ = declare_parameter<double>("max_position_sigma", 2.0);
    status_period_ = declare_parameter<double>("status_period_s", 2.0);
    const double rate = declare_parameter<double>("setpoint_rate_hz", 20.0);

    ControllerParams cp;
    cp.kp_xy = declare_parameter<double>("kp_xy", cp.kp_xy);
    cp.max_speed_xy = declare_parameter<double>("max_speed_xy", cp.max_speed_xy);
    cp.kp_z = declare_parameter<double>("kp_z", cp.kp_z);
    cp.max_speed_z = declare_parameter<double>("max_speed_z", cp.max_speed_z);
    cp.kp_yaw = declare_parameter<double>("kp_yaw", cp.kp_yaw);
    cp.max_yaw_rate = declare_parameter<double>("max_yaw_rate", cp.max_yaw_rate);
    controller_ = RouteController(cp);

    est_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      est_topic, 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_estimate(*m);});
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      ns + "/state", 10, [this](mavros_msgs::msg::State::ConstSharedPtr m) {on_state(*m);});
    sp_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(ns + "/setpoint_raw/local", 10);
    mode_cli_ = create_client<mavros_msgs::srv::SetMode>(ns + "/set_mode");
    arm_cli_ = create_client<mavros_msgs::srv::CommandBool>(ns + "/cmd/arming");
    takeoff_cli_ = create_client<mavros_msgs::srv::CommandTOL>(ns + "/cmd/takeoff");

    // Simulation-time timer: setpoint rate and timeouts follow the sim clock,
    // like ArduPilot SITL's own GUIDED timeouts.
    timer_ = rclcpp::create_timer(this, get_clock(),
        rclcpp::Duration::from_seconds(1.0 / rate), [this]() {tick();});

    RCLCPP_INFO(get_logger(),
      "estimated mission: feedback=%s only | %d laps, %zu route points, %.1f m, "
      "%.2f m/s | accept %.2f m", est_topic.c_str(), laps_, offsets_.size(), altitude_,
      cp.max_speed_xy, accept_xy_);
  }

private:
  // ---- inputs -----------------------------------------------------------------
  void on_state(const mavros_msgs::msg::State & m)
  {
    if (m.connected != state_.connected || m.armed != state_.armed || m.mode != state_.mode) {
      RCLCPP_INFO(get_logger(), "MAVROS state: connected=%d armed=%d mode='%s' (was %d/%d/'%s')",
        m.connected, m.armed, m.mode.c_str(), state_.connected, state_.armed,
        state_.mode.c_str());
    }
    state_ = m;
  }

  void on_estimate(const nav_msgs::msg::Odometry & m)
  {
    const auto & p = m.pose.pose.position;
    est_pos_ = Eigen::Vector3d(p.x, p.y, p.z);
    est_yaw_ = yaw_from_quaternion(m.pose.pose.orientation);
    est_sigma_xy_ = std::sqrt(std::max(m.pose.covariance[0], m.pose.covariance[7]));
    est_stamp_ = rclcpp::Time(m.header.stamp, get_clock()->get_clock_type());
    // drone_localization puts the gyro-bias variance in twist.covariance[35].
    est_bias_sigma_ = std::sqrt(std::max(m.twist.covariance[35], 0.0));
    have_estimate_ = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
      std::isfinite(est_yaw_);
  }

  double estimate_age() const
  {
    if (!have_estimate_) {return 1e9;}
    return (now() - est_stamp_).seconds();
  }

  // ---- phase helpers --------------------------------------------------------------
  double phase_time() const {return (now() - phase_start_).seconds();}

  void go(Phase p)
  {
    RCLCPP_INFO(get_logger(), "phase %s -> %s (t=%.1f, est p=(%.2f, %.2f, %.2f) yaw=%.1f deg)",
      phase_name(phase_), phase_name(p), now().seconds(), est_pos_.x(), est_pos_.y(),
      est_pos_.z(), est_yaw_ * 180.0 / M_PI);
    phase_ = p;
    phase_start_ = now();
    request_sent_ = false;
  }

  template<class Srv>
  void call(typename rclcpp::Client<Srv>::SharedPtr cli,
    std::shared_ptr<typename Srv::Request> req, std::function<void(bool)> done)
  {
    if (request_sent_ || !cli->service_is_ready()) {return;}
    request_sent_ = true;
    last_request_ = now();
    cli->async_send_request(req,
      [this, done](typename rclcpp::Client<Srv>::SharedFuture f) {
        bool ok = false;
        try {
          const auto r = f.get();
          if constexpr (std::is_same_v<Srv, mavros_msgs::srv::SetMode>) {ok = r->mode_sent;} else {
            ok = r->success;
          }
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "service call failed: %s", e.what());
        }
        done(ok);
      });
  }

  // Allow a new request after a response or a 2 s timeout.
  void retry_after(double seconds)
  {
    if (request_sent_ && (now() - last_request_).seconds() > seconds) {request_sent_ = false;}
  }

  void send(const VelocityCommand & c)
  {
    using PT = mavros_msgs::msg::PositionTarget;
    PT sp;
    sp.header.stamp = now();
    sp.coordinate_frame = PT::FRAME_LOCAL_NED;  // ENU in; MAVROS converts
    sp.type_mask = PT::IGNORE_PX | PT::IGNORE_PY | PT::IGNORE_PZ |
      PT::IGNORE_AFX | PT::IGNORE_AFY | PT::IGNORE_AFZ | PT::IGNORE_YAW;
    sp.velocity.x = c.velocity.x();
    sp.velocity.y = c.velocity.y();
    sp.velocity.z = c.velocity.z();
    sp.yaw_rate = static_cast<float>(c.yaw_rate);
    sp_pub_->publish(sp);
    last_cmd_ = c;
  }

  void request_land(const char * reason)
  {
    RCLCPP_WARN(get_logger(), "LANDING: %s", reason);
    go(Phase::LAND);
  }

  // Fly toward target using the estimate. Returns false if the estimate is not usable.
  bool fly_to(const Eigen::Vector3d & target)
  {
    const double age = estimate_age();
    if (age > estimate_lost_land_) {
      request_land("estimate lost");
      return false;
    }
    if (age > estimate_timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "estimate stale (%.2f s): holding with zero velocity", age);
      send(VelocityCommand{});
      return false;
    }
    if (est_sigma_xy_ > max_position_sigma_) {
      request_land("estimate covariance diverged");
      return false;
    }
    send(controller_.compute(est_pos_, est_yaw_, target, origin_yaw_));
    return true;
  }

  // ---- state machine ----------------------------------------------------------------
  void tick()
  {
    // Pilot/FCU takeover detection. An empty mode or a lost link is *unknown*
    // (MAVROS emits those on missed heartbeats, frequent at low real-time
    // factor), not a takeover: keep flying. Only a real other mode, held for
    // mode_abort_after_s, aborts.
    const bool airborne_phase = phase_ == Phase::CLIMB || phase_ == Phase::ROUTE ||
      phase_ == Phase::FINAL_HOLD || phase_ == Phase::TAKEOFF;
    const bool known_other_mode = state_.connected && !state_.mode.empty() &&
      state_.mode != "GUIDED";
    if (airborne_phase && known_other_mode && phase_time() > 2.0) {
      if (mode_bad_since_.nanoseconds() == 0) {mode_bad_since_ = now();}
      if ((now() - mode_bad_since_).seconds() > mode_abort_after_) {
        RCLCPP_ERROR(get_logger(),
          "FCU in mode '%s' for %.1f s (not GUIDED): pilot/FCU takeover, stopping setpoints",
          state_.mode.c_str(), mode_abort_after_);
        go(Phase::DONE);
      }
    } else {
      mode_bad_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      if (airborne_phase && (!state_.connected || state_.mode.empty())) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "MAVROS state unknown (connected=%d, mode='%s'): continuing", state_.connected,
          state_.mode.c_str());
      }
    }

    switch (phase_) {
      case Phase::WAIT_READY:
        if (!state_.connected) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for FCU connection");
          break;
        }
        if (!have_estimate_ || estimate_age() > estimate_timeout_ ||
          est_sigma_xy_ > origin_max_sigma_)
        {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "waiting for a valid estimate (age %.2f s, sigma_xy %.2f m, need < %.2f)",
            estimate_age(), est_sigma_xy_, origin_max_sigma_);
          break;
        }
        // Let the estimator settle on the ground (zero-rate/zero-velocity
        // updates refine the gyro bias, so yaw holds while hovering).
        if (settle_start_.nanoseconds() == 0) {
          settle_start_ = now();
          RCLCPP_INFO(get_logger(),
            "estimate valid; settling on the ground (>= %.0f s, until bias sigma < %.5f rad/s)",
            ground_settle_, settle_max_bias_sigma_);
        }
        {
          const double settled = (now() - settle_start_).seconds();
          const bool bias_ok = est_bias_sigma_ > 0.0 && est_bias_sigma_ < settle_max_bias_sigma_;
          if (settled < ground_settle_ || (!bias_ok && settled < 4.0 * ground_settle_)) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
              "settling on the ground: %.1f / %.0f s sim time, gyro bias sigma %.5f rad/s "
              "(need < %.5f)", settled, ground_settle_, est_bias_sigma_, settle_max_bias_sigma_);
            break;
          }
        }
        origin_ = est_pos_;
        origin_yaw_ = est_yaw_;
        mission_.emplace(build_route(origin_, altitude_, laps_, offsets_), accept_xy_, accept_z_);
        RCLCPP_INFO(get_logger(),
          "MISSION ORIGIN from estimate: (%.2f, %.2f, %.2f) m, yaw %.1f deg (sigma_xy %.2f m, "
          "bias sigma %.5f rad/s); %zu waypoints", origin_.x(), origin_.y(), origin_.z(),
          origin_yaw_ * 180.0 / M_PI, est_sigma_xy_, est_bias_sigma_, mission_->size());
        go(state_.mode == "GUIDED" ? Phase::ARM : Phase::SET_GUIDED);
        break;

      case Phase::SET_GUIDED:
        if (state_.mode == "GUIDED") {go(Phase::ARM); break;}
        retry_after(2.0);
        {
          auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
          req->custom_mode = "GUIDED";
          call<mavros_msgs::srv::SetMode>(mode_cli_, req, [](bool) {});
        }
        break;

      case Phase::ARM:
        if (state_.armed) {go(Phase::TAKEOFF); break;}
        if (phase_time() > 300.0) {
          RCLCPP_ERROR(get_logger(), "arming timed out after 300 s sim time");
          go(Phase::DONE);
          break;
        }
        retry_after(3.0);
        {
          auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
          req->value = true;
          call<mavros_msgs::srv::CommandBool>(arm_cli_, req, [this](bool ok) {
              if (!ok) {
                RCLCPP_WARN(get_logger(),
                  "arming rejected by the FCU (usually pre-arm checks: ArduPilot's own EKF "
                  "is not using GPS yet, see 'Arm: ...' in the MAVROS/SITL console); retrying");
              }
            });
        }
        break;

      case Phase::TAKEOFF:
        {
          const double height = est_pos_.z() - origin_.z();
          if (height >= takeoff_reached_fraction_ * altitude_) {go(Phase::CLIMB); break;}
          if (phase_time() > takeoff_timeout_) {request_land("takeoff timeout"); break;}
          if (!state_.armed && phase_time() > 5.0) {
            RCLCPP_ERROR(get_logger(), "disarmed during takeoff");
            go(Phase::DONE);
            break;
          }
          retry_after(5.0);
          if (!takeoff_accepted_) {
            auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
            req->altitude = static_cast<float>(altitude_);
            call<mavros_msgs::srv::CommandTOL>(takeoff_cli_, req, [this](bool ok) {
                takeoff_accepted_ = ok;
                RCLCPP_INFO(get_logger(), "takeoff command %s", ok ? "accepted" : "rejected");
              });
          }
        }
        break;

      case Phase::CLIMB:
        {
          const Eigen::Vector3d target = origin_ + Eigen::Vector3d(0, 0, altitude_);
          fly_to(target);
          if (std::abs(est_pos_.z() - target.z()) < climb_tolerance_ &&
            (est_pos_ - target).head<2>().norm() < accept_xy_)
          {
            go(Phase::ROUTE);
          }
        }
        break;

      case Phase::ROUTE:
        if (mission_->update(est_pos_)) {
          const auto & reached = mission_->route()[mission_->index() - 1];
          RCLCPP_INFO(get_logger(), "REACHED %s at t=%.1f: est (%.2f, %.2f, %.2f) [%zu/%zu]",
            reached.label.c_str(), now().seconds(), est_pos_.x(), est_pos_.y(), est_pos_.z(),
            mission_->index(), mission_->size());
        }
        if (mission_->done()) {go(Phase::FINAL_HOLD); break;}
        fly_to(mission_->current().position);
        break;

      case Phase::FINAL_HOLD:
        fly_to(origin_ + Eigen::Vector3d(0, 0, altitude_));
        if (phase_time() >= final_hold_) {go(Phase::LAND);}
        break;

      case Phase::LAND:
        if (state_.mode == "LAND") {go(Phase::WAIT_DISARM); break;}
        retry_after(2.0);
        {
          auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
          req->custom_mode = "LAND";
          call<mavros_msgs::srv::SetMode>(mode_cli_, req, [](bool) {});
        }
        break;

      case Phase::WAIT_DISARM:
        if (!state_.armed) {
          const Eigen::Vector2d off = (est_pos_ - origin_).head<2>();
          RCLCPP_INFO(get_logger(),
            "MISSION COMPLETE: landed and disarmed at est offset (%.2f, %.2f) m from origin, "
            "flight time %.1f s", off.x(), off.y(), (now() - mission_start_).seconds());
          go(Phase::DONE);
        }
        break;

      case Phase::DONE:
        break;
    }

    if (phase_ == Phase::TAKEOFF && mission_start_.nanoseconds() == 0) {mission_start_ = now();}
    log_status();
  }

  void log_status()
  {
    if ((now() - last_status_).seconds() < status_period_) {return;}
    last_status_ = now();
    if (phase_ == Phase::WAIT_READY || phase_ == Phase::DONE) {return;}
    const Eigen::Vector3d rel = est_pos_ - origin_;
    std::string target = "-";
    if (phase_ == Phase::ROUTE && mission_ && !mission_->done()) {
      const Eigen::Vector3d tr = mission_->current().position - origin_;
      char buf[96];
      std::snprintf(buf, sizeof(buf), "(%.1f, %.1f, %.1f) dist %.2f m", tr.x(), tr.y(), tr.z(),
        (mission_->current().position - est_pos_).head<2>().norm());
      target = buf;
    }
    RCLCPP_INFO(get_logger(),
      "[%s] est rel (%.2f, %.2f, %.2f) yaw %.1f sigma %.2f m | target %s | cmd v=(%.2f, %.2f, %.2f) "
      "yawrate %.2f | mode %s armed %d", phase_name(phase_), rel.x(), rel.y(), rel.z(),
      est_yaw_ * 180.0 / M_PI, est_sigma_xy_, target.c_str(), last_cmd_.velocity.x(),
      last_cmd_.velocity.y(), last_cmd_.velocity.z(), last_cmd_.yaw_rate, state_.mode.c_str(),
      state_.armed ? 1 : 0);
  }

  // parameters
  double altitude_, takeoff_reached_fraction_, takeoff_timeout_, climb_tolerance_;
  double origin_max_sigma_, accept_xy_, accept_z_, final_hold_;
  double estimate_timeout_, estimate_lost_land_, max_position_sigma_, status_period_;
  double ground_settle_, settle_max_bias_sigma_, mode_abort_after_;
  int laps_;
  std::vector<Eigen::Vector2d> offsets_;
  RouteController controller_;

  // estimate
  bool have_estimate_ = false;
  Eigen::Vector3d est_pos_ = Eigen::Vector3d::Zero();
  double est_yaw_ = 0.0;
  double est_sigma_xy_ = 1e9;
  double est_bias_sigma_ = 0.0;
  rclcpp::Time est_stamp_;

  // mission
  Phase phase_ = Phase::WAIT_READY;
  rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time mission_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_request_{0, 0, RCL_ROS_TIME};
  rclcpp::Time settle_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time mode_bad_since_{0, 0, RCL_ROS_TIME};
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
  double origin_yaw_ = 0.0;
  std::optional<Mission> mission_;
  bool request_sent_ = false;
  bool takeoff_accepted_ = false;
  VelocityCommand last_cmd_;
  mavros_msgs::msg::State state_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr est_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr sp_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_cli_;
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
