#ifndef MISSION_EXECUTOR__OUTDOOR_MISSION_NODE_HPP_
#define MISSION_EXECUTOR__OUTDOOR_MISSION_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
#include "mavros_msgs/msg/home_position.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/message_interval.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mission_executor/outdoor_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mission_executor
{
using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;

enum class OutdoorState
{
  WAIT_READY,
  SET_GUIDED,
  ARM,
  TAKEOFF,
  STABILIZE,
  SQUARE,
  MOVE_TO_CENTER,
  ALIGN_TANGENT,
  FIGURE_EIGHT,
  SETTLE_CENTER,
  RETURN_HOME,
  LAND,
  DISARM,
  DONE,
  ABORTED,
  RELEASE_CONTROL,
  FAILED
};

const char * stateName(OutdoorState state);

/**
 * @brief Execute the outdoor mission through MAVROS.
 *
 * All callbacks run serially on a SingleThreadedExecutor. Service calls are
 * asynchronous; no callback waits for another callback or a service response.
 */
class OutdoorMissionNode : public rclcpp::Node
{
public:
  /// Construct ROS interfaces; flight remains controlled by enable_flight.
  OutdoorMissionNode();

private:
  void declareParameters();
  /// Create telemetry subscriptions, diagnostic publishers and stream client.
  void configureTelemetry();
  /// Create flight interfaces only when flight is explicitly enabled.
  void configureFlightControl();
  /// Keep wall timers active so paused ROS time can still be detected.
  void configureTimers();
  void stateCallback(mavros_msgs::msg::State::ConstSharedPtr msg);
  void extendedStateCallback(mavros_msgs::msg::ExtendedState::ConstSharedPtr msg);
  void gpsFixCallback(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void satellitesCallback(std_msgs::msg::UInt32::ConstSharedPtr msg);
  void homeCallback(mavros_msgs::msg::HomePosition::ConstSharedPtr msg);
  void headingCallback(std_msgs::msg::Float64::ConstSharedPtr msg);
  void relativeAltitudeCallback(std_msgs::msg::Float64::ConstSharedPtr msg);
  void poseCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void velocityCallback(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void abortCallback(
    std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  /// Run readiness or mission progression without blocking the executor.
  void controlLoop();
  /// Detect clock stalls and discard telemetry from an earlier ROS timeline.
  void updateClock();
  void requestLandedStateStream();
  void validateParameters();
  /// Check a telemetry receipt time against the ROS-time freshness limit.
  bool isFresh(rclcpp::Time received) const;
  Point3 localPosition() const;
  GeodeticPosition gpsPosition() const;
  /// Check the navigation telemetry required both before and during flight.
  std::string telemetryProblem() const;
  std::string readinessProblem() const;
  void captureReference();
  void updateReadiness();
  bool isTerminal() const;
  double localYaw() const;
  /// Enter a phase and invalidate responses belonging to the previous phase.
  void changePhase(OutdoorState next, const std::string & reason);
  /// Request landing while control is owned, or release control otherwise.
  void beginAbort(const std::string & reason);
  void requestMode(const std::string & mode);
  void requestArm(bool arm);
  void requestTakeoff();
  std::string flightNavigationProblem() const;
  bool settledAt(Point3 destination);
  Point3 squareDestination() const;
  Point3 centreEnu() const;
  /// Check flight authority and health, then advance the current mission phase.
  void updateFlight();
  /// Publish one ENU position/yaw target after checking control ownership.
  void publishSetpoint();
  static void addValue(
    diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & key, const std::string & value);
  /// Report reference, telemetry and heading consistency without commanding flight.
  void publishDiagnostics();

  // Configuration.
  Point3 poi_;
  double altitude_;
  double side_;
  double amplitude_;
  double omega_;
  double max_speed_;
  double max_distance_;
  double max_altitude_;
  double envelope_margin_;
  double yaw_offset_;
  int min_satellites_;
  double timeout_;
  double clock_stall_timeout_;
  double readiness_duration_;
  double sample_window_;
  bool enable_flight_ = false;
  bool execute_square_ = true;
  bool execute_figure_eight_ = true;
  double position_tolerance_;
  double altitude_tolerance_;
  double velocity_tolerance_;
  double yaw_tolerance_;
  double settle_duration_;
  double return_speed_;
  double command_timeout_;
  double takeoff_timeout_;
  double phase_timeout_;
  double landing_timeout_;
  double max_tracking_error_;
  double max_reference_error_;
  double altitude_disagreement_;

  // Latest telemetry and ROS receipt times.
  mavros_msgs::msg::State state_;
  mavros_msgs::msg::ExtendedState extended_;
  sensor_msgs::msg::NavSatFix fix_;
  mavros_msgs::msg::HomePosition home_;
  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::TwistStamped velocity_;
  unsigned satellites_ = 255;
  double heading_ = 0.0;
  double relative_altitude_ = 0.0;
  bool home_received_ = false;
  rclcpp::Time state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time extended_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time fix_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time satellites_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time velocity_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time heading_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time altitude_time_{0, 0, RCL_ROS_TIME};

  // Captured mission reference.
  Point3 launch_local_{0.0, 0.0, 0.0};
  Point3 launch_enu_{0.0, 0.0, 0.0};
  GeodeticPosition origin_{0.0, 0.0, 0.0};
  double envelope_bound_ = 0.0;
  double launch_relative_altitude_ = 0.0;
  bool envelope_ok_ = false;
  std::unique_ptr<OutdoorReference> reference_;
  std::string origin_source_;
  std::string local_frame_;

  // Mission execution state.
  OutdoorState flight_state_ = OutdoorState::WAIT_READY;
  bool owns_guided_ = false;
  bool aborting_ = false;
  std::size_t square_corner_ = 0;
  bool command_sent_ = false;
  bool command_complete_ = false;
  bool command_accepted_ = false;
  bool arm_pending_ = false;
  bool home_locked_ = false;
  bool land_mode_seen_ = false;
  unsigned command_generation_ = 0;
  TimePoint phase_started_wall_;
  rclcpp::Time phase_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time settled_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_target_time_{0, 0, RCL_ROS_TIME};
  Point3 target_{0.0, 0.0, 0.0};
  double target_yaw_ = 0.0;
  mavros_msgs::msg::HomePosition flight_home_;
  std::string abort_reason_;
  std::string reported_state_;
  std::string reason_;
  rclcpp::Time ready_since_{0, 0, RCL_ROS_TIME};

  // Clock and stream-request state.
  bool stream_requested_ = false;
  bool stream_ready_ = false;
  int64_t stream_request_id_ = 0;
  TimePoint stream_request_time_;
  rclcpp::Time last_clock_time_{0, 0, RCL_ROS_TIME};
  TimePoint last_clock_advance_;
  bool clock_stalled_ = false;

  // ROS interfaces.
  rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedPtr interval_client_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_service_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_sub_;
  rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr relative_altitude_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr satellites_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr readiness_timer_;
  rclcpp::TimerBase::SharedPtr reporting_timer_;
};

}  // namespace mission_executor

#endif  // MISSION_EXECUTOR__OUTDOOR_MISSION_NODE_HPP_
