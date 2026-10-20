#include "mission_executor/outdoor_mission_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace mission_executor
{
namespace
{
constexpr double pi = 3.14159265358979323846;
}  // namespace

using std::placeholders::_1;
using std::placeholders::_2;

const char * getMissionStateName(OutdoorState state)
{
  switch (state) {
    case OutdoorState::WAIT_READY:
      return "WAIT_READY";
    case OutdoorState::SET_GUIDED:
      return "SET_GUIDED";
    case OutdoorState::ARM:
      return "ARM";
    case OutdoorState::TAKEOFF:
      return "TAKEOFF";
    case OutdoorState::STABILIZE:
      return "STABILIZE";
    case OutdoorState::SQUARE:
      return "SQUARE";
    case OutdoorState::MOVE_TO_CENTER:
      return "MOVE_TO_CENTER";
    case OutdoorState::ALIGN_TANGENT:
      return "ALIGN_TANGENT";
    case OutdoorState::FIGURE_EIGHT:
      return "FIGURE_EIGHT";
    case OutdoorState::SETTLE_CENTER:
      return "SETTLE_CENTER";
    case OutdoorState::RETURN_HOME:
      return "RETURN_HOME";
    case OutdoorState::LAND:
      return "LAND";
    case OutdoorState::DISARM:
      return "DISARM";
    case OutdoorState::DONE:
      return "DONE";
    case OutdoorState::ABORTED:
      return "ABORTED";
    case OutdoorState::RELEASE_CONTROL:
      return "RELEASE_CONTROL";
    case OutdoorState::FAILED:
      return "FAILED";
  }
  return "UNKNOWN";
}

OutdoorMissionNode::OutdoorMissionNode()
: Node("outdoor_mission_node")
{
  declareParameters();
  configureTelemetryInterfaces();
  configureFlightInterfaces();
  configureTimers();
  changeMissionPhase(OutdoorState::WAIT_READY, "Waiting for landed-state stream setup");
  const char * mode = "TELEMETRY ONLY";
  if (enable_flight_) {
    if (execute_square_) {
      mode = "OUTDOOR MISSION";
    } else {
      mode = "TAKEOFF/RETURN FLIGHT CHECK";
    }
  }
  RCLCPP_INFO(
    get_logger(), "Outdoor mode: %s; configured yaw offset %.1f degrees.",
    mode, yaw_offset_ * 180.0 / pi);
}

void OutdoorMissionNode::declareParameters()
{
  mavros_prefix_ = declare_parameter("mavros_prefix", std::string("/mavros"));
  if (mavros_prefix_.empty()) {
    throw std::invalid_argument("mavros_prefix must name a non-root namespace");
  }
  if (mavros_prefix_.front() != '/') {
    mavros_prefix_.insert(0, "/");
  }
  while (mavros_prefix_.size() > 1 && mavros_prefix_.back() == '/') {
    mavros_prefix_.pop_back();
  }
  if (mavros_prefix_ == "/") {
    throw std::invalid_argument("mavros_prefix must name a non-root namespace");
  }
  poi_ = {declare_parameter("poi_east", 2.5), declare_parameter("poi_north", 2.5), 0.0};
  coordinate_mode_ = declare_parameter("coordinate_mode", std::string("gps"));
  auto gps_poi = declare_parameter(
    "poi_gps", std::vector<double>{48.15146692284, 17.07378359836});
  if (gps_poi.size() != 2) {
    throw std::invalid_argument("poi_gps must contain [latitude, longitude]");
  }
  gps_poi_ = {gps_poi[0], gps_poi[1], 0.0};
  const std::array<GeodeticPosition, 4> default_corners{{
    {48.15148940569, 17.07375000000, 0.0},
    {48.15148940567, 17.07381719675, 0.0},
    {48.15144443998, 17.07381719669, 0.0},
    {48.15144444000, 17.07375000000, 0.0}}};
  for (std::size_t i = 0; i < gps_square_corners_.size(); ++i) {
    auto coordinates = declare_parameter(
      "square_corner_" + std::to_string(i + 1) + "_gps",
      std::vector<double>{default_corners[i].latitude, default_corners[i].longitude});
    if (coordinates.size() != 2) {
      throw std::invalid_argument("Each square_corner_N_gps must contain [latitude, longitude]");
    }
    gps_square_corners_[i] = {coordinates[0], coordinates[1], 0.0};
  }
  altitude_ = declare_parameter("flight_altitude", 5.0);
  side_ = declare_parameter("square_side", 5.0);
  amplitude_ = declare_parameter("figure8_amplitude", 3.0);
  omega_ = declare_parameter("omega", 0.2);
  max_speed_ = declare_parameter("max_speed", 1.5);
  max_distance_ = declare_parameter("max_horizontal_distance", 10.0);
  max_altitude_ = declare_parameter("max_altitude", 6.0);
  envelope_margin_ = declare_parameter("envelope_margin", 0.5);
  yaw_offset_ = declare_parameter("local_yaw_offset_deg", 0.0) * pi / 180.0;
  min_satellites_ = declare_parameter("min_satellites", 8);
  timeout_ = declare_parameter("telemetry_timeout", 2.0);
  clock_stall_timeout_ = declare_parameter("clock_stall_timeout", 10.0);
  readiness_duration_ = declare_parameter("gps_ready_duration", 3.0);
  sample_window_ = declare_parameter("reference_sample_window", 0.5);
  enable_flight_ = declare_parameter("enable_flight", false);
  execute_square_ = declare_parameter("execute_square", true);
  execute_figure_eight_ = declare_parameter("execute_figure_eight", true);
  position_tolerance_ = declare_parameter("position_tolerance", 0.25);
  altitude_tolerance_ = declare_parameter("altitude_tolerance", 0.2);
  velocity_tolerance_ = declare_parameter("velocity_tolerance", 0.2);
  yaw_tolerance_ = declare_parameter("yaw_tolerance_deg", 5.0) * pi / 180.0;
  settle_duration_ = declare_parameter("settle_duration", 1.0);
  return_speed_ = declare_parameter("transfer_speed", 1.0);
  command_timeout_ = declare_parameter("command_timeout", 10.0);
  takeoff_timeout_ = declare_parameter("takeoff_timeout", 45.0);
  phase_timeout_ = declare_parameter("phase_timeout", 60.0);
  landing_timeout_ = declare_parameter("landing_timeout", 60.0);
  max_tracking_error_ = declare_parameter("max_tracking_error", 2.0);
  max_reference_error_ = declare_parameter("max_reference_error", 3.0);
  altitude_disagreement_ = declare_parameter("altitude_disagreement", 1.0);
  validateParameters();
}

void OutdoorMissionNode::configureTelemetryInterfaces()
{
  auto sensor_qos = rclcpp::SensorDataQoS();
  state_sub_ = create_subscription<mavros_msgs::msg::State>(
    mavros_prefix_ + "/state", sensor_qos,
    std::bind(&OutdoorMissionNode::stateCallback, this, _1));
  extended_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
    mavros_prefix_ + "/extended_state", sensor_qos,
    std::bind(&OutdoorMissionNode::extendedStateCallback, this, _1));
  fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    mavros_prefix_ + "/global_position/global", sensor_qos,
    std::bind(&OutdoorMissionNode::gpsFixCallback, this, _1));
  satellites_sub_ = create_subscription<std_msgs::msg::UInt32>(
    mavros_prefix_ + "/global_position/raw/satellites", sensor_qos,
    std::bind(&OutdoorMissionNode::satellitesCallback, this, _1));
  home_sub_ = create_subscription<mavros_msgs::msg::HomePosition>(
    mavros_prefix_ + "/home_position/home", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&OutdoorMissionNode::homeCallback, this, _1));
  heading_sub_ = create_subscription<std_msgs::msg::Float64>(
    mavros_prefix_ + "/global_position/compass_hdg", sensor_qos,
    std::bind(&OutdoorMissionNode::headingCallback, this, _1));
  relative_altitude_sub_ = create_subscription<std_msgs::msg::Float64>(
    mavros_prefix_ + "/global_position/rel_alt", sensor_qos,
    std::bind(&OutdoorMissionNode::relativeAltitudeCallback, this, _1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    mavros_prefix_ + "/local_position/pose", sensor_qos,
    std::bind(&OutdoorMissionNode::poseCallback, this, _1));
  velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    mavros_prefix_ + "/local_position/velocity_local", sensor_qos,
    std::bind(&OutdoorMissionNode::velocityCallback, this, _1));
  state_pub_ = create_publisher<std_msgs::msg::String>(
    "outdoor_mission/state", rclcpp::QoS(1).reliable().transient_local());
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "outdoor_mission/diagnostics", 10);
  interval_client_ = create_client<mavros_msgs::srv::MessageInterval>(
    mavros_prefix_ + "/set_message_interval");
}

void OutdoorMissionNode::configureFlightInterfaces()
{
  if (enable_flight_) {
    mode_client_ = create_client<mavros_msgs::srv::SetMode>(mavros_prefix_ + "/set_mode");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>(mavros_prefix_ + "/cmd/arming");
    takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>(mavros_prefix_ + "/cmd/takeoff");
    setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
      mavros_prefix_ + "/setpoint_raw/local",
      10);
    abort_service_ = create_service<std_srvs::srv::Trigger>(
      "outdoor_mission/abort",
      std::bind(&OutdoorMissionNode::abortCallback, this, _1, _2));
    setpoint_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / 30.0),
      std::bind(&OutdoorMissionNode::publishFlightSetpoint, this));
  }
}

void OutdoorMissionNode::configureTimers()
{
  readiness_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&OutdoorMissionNode::updateMission, this));
  reporting_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&OutdoorMissionNode::publishDiagnostics, this));
}

void OutdoorMissionNode::stateCallback(mavros_msgs::msg::State::ConstSharedPtr msg)
{
  state_ = *msg;
  state_time_ = now();
}

void OutdoorMissionNode::extendedStateCallback(mavros_msgs::msg::ExtendedState::ConstSharedPtr msg)
{
  extended_ = *msg;
  extended_time_ = now();
}

void OutdoorMissionNode::gpsFixCallback(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  fix_ = *msg;
  fix_time_ = now();
}

void OutdoorMissionNode::satellitesCallback(std_msgs::msg::UInt32::ConstSharedPtr msg)
{
  satellites_ = msg->data;
  satellites_time_ = now();
}

void OutdoorMissionNode::homeCallback(mavros_msgs::msg::HomePosition::ConstSharedPtr msg)
{
  home_ = *msg;
  home_received_ = true;
}

void OutdoorMissionNode::headingCallback(std_msgs::msg::Float64::ConstSharedPtr msg)
{
  heading_ = msg->data;
  heading_time_ = now();
}

void OutdoorMissionNode::relativeAltitudeCallback(std_msgs::msg::Float64::ConstSharedPtr msg)
{
  relative_altitude_ = msg->data;
  altitude_time_ = now();
}

void OutdoorMissionNode::poseCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  pose_ = *msg;
  pose_time_ = now();
}

void OutdoorMissionNode::velocityCallback(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  velocity_ = *msg;
  velocity_time_ = now();
}

void OutdoorMissionNode::abortCallback(
  std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = !isMissionInTerminalState();
  response->message =
    response->success ? "Abort accepted; monitor mission state" : "Mission already terminal";
  if (response->success) {
    requestMissionAbort("Operator requested abort");
  }
}

void OutdoorMissionNode::updateMission()
{
  if (isMissionInTerminalState()) {
    return;
  }
  checkRosClock();
  if (isMissionInTerminalState()) {
    return;
  }
  if (!stream_ready_) {
    requestLandedStateStream();
    if (stream_requested_ &&
      std::chrono::duration<double>(SteadyClock::now() - stream_request_time_).count() >
      command_timeout_)
    {
      interval_client_->remove_pending_request(stream_request_id_);
      changeMissionPhase(OutdoorState::FAILED,
          "Landed-state stream request timed out; restart required");
    }
    return;
  }
  if (enable_flight_) {
    updateFlightStateMachine();
  } else {
    updateMissionReadiness();
  }
}

void OutdoorMissionNode::checkRosClock()
{
  const auto current_time = now();
  const auto wall_time = SteadyClock::now();
  if (current_time < last_clock_time_) {
    // Samples from the old timeline cannot establish readiness after a reset.
    state_time_ = extended_time_ = fix_time_ = satellites_time_ =
      pose_time_ = velocity_time_ = heading_time_ = altitude_time_ =
      ready_since_ = settled_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    if (enable_flight_ && flight_state_ != OutdoorState::WAIT_READY &&
      !isMissionInTerminalState())
    {
      changeMissionPhase(OutdoorState::RELEASE_CONTROL,
          "ROS clock moved backwards; restart mission node");
    }
    RCLCPP_WARN(get_logger(), "ROS clock moved backwards; waiting for new telemetry");
  }
  const bool navigating = enable_flight_ && !isMissionInTerminalState() &&
    flight_state_ != OutdoorState::WAIT_READY && flight_state_ != OutdoorState::LAND &&
    flight_state_ != OutdoorState::DISARM;
  if (navigating && last_clock_time_.nanoseconds() != 0 &&
    (current_time - last_clock_time_).seconds() > timeout_)
  {
    requestMissionAbort("ROS clock discontinuity");
  }
  if (current_time != last_clock_time_ || last_clock_advance_ == TimePoint{}) {
    last_clock_advance_ = wall_time;
  }
  last_clock_time_ = current_time;
  clock_stalled_ = std::chrono::duration<double>(wall_time - last_clock_advance_).count() >
    clock_stall_timeout_;
}

void OutdoorMissionNode::requestLandedStateStream()
{
  if (stream_requested_ || !isWithinTimeout(state_time_) || !state_.connected ||
    !interval_client_->service_is_ready())
  {
    return;
  }
  stream_requested_ = true;
  stream_request_time_ = SteadyClock::now();
  auto request = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
  request->message_id = 245;  // MAVLink EXTENDED_SYS_STATE.
  request->message_rate = 2.0;
  auto result = interval_client_->async_send_request(
    request,
    [this](rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedFuture future) {
      if (isMissionInTerminalState()) {
        return;
      }
      if (!future.get()->success) {
        changeMissionPhase(OutdoorState::FAILED,
          "Landed-state stream request rejected; restart required");
        return;
      }
      stream_ready_ = true;
      RCLCPP_INFO(get_logger(), "Requested landed-state telemetry at 2 Hz");
    });
  stream_request_id_ = result.request_id;
}

void OutdoorMissionNode::validateParameters()
{
  if (coordinate_mode_ != "gps" && coordinate_mode_ != "enu") {
    throw std::invalid_argument("coordinate_mode must be gps or enu");
  }
  if (coordinate_mode_ == "gps") {
    if (!isValidGeodeticPosition(gps_poi_)) {
      throw std::invalid_argument("poi_gps requires valid latitude and longitude");
    }
    for (const auto & corner : gps_square_corners_) {
      if (!isValidGeodeticPosition(corner)) {
        throw std::invalid_argument("GPS square corners require valid latitude and longitude");
      }
    }
  }
  for (double value : {omega_, max_speed_, max_distance_,
      max_altitude_, timeout_, readiness_duration_, sample_window_, position_tolerance_,
      altitude_tolerance_, velocity_tolerance_, yaw_tolerance_, settle_duration_, return_speed_,
      command_timeout_, takeoff_timeout_, phase_timeout_, landing_timeout_, max_tracking_error_,
      max_reference_error_, altitude_disagreement_, clock_stall_timeout_})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(
        "Outdoor dimensions, limits and durations must be finite and positive");
    }
  }
  if (!isFinitePoint(poi_) || !std::isfinite(yaw_offset_) ||
    !std::isfinite(envelope_margin_) || envelope_margin_ < 0.0 ||
    min_satellites_ < 1 || min_satellites_ > 254)
  {
    throw std::invalid_argument("Invalid POI, heading offset, margin or satellite threshold");
  }
  if (altitude_ != 5.0 || side_ != 5.0 || amplitude_ != 3.0) {
    throw std::invalid_argument("A1.3 requires altitude=5, square_side=5, figure8_amplitude=3");
  }
  if (calculateFigureEightPeakSpeed(amplitude_, omega_) > max_speed_) {
    throw std::invalid_argument("Figure-eight peak speed exceeds max_speed");
  }
  if (return_speed_ > max_speed_) {
    throw std::invalid_argument("transfer_speed exceeds max_speed");
  }
  if (altitude_ + envelope_margin_ > max_altitude_) {
    throw std::invalid_argument("Flight altitude plus margin exceeds max_altitude");
  }
}

bool OutdoorMissionNode::isWithinTimeout(rclcpp::Time received) const
{
  if (received.nanoseconds() == 0) {
    return false;
  }
  const double age = (now() - received).seconds();
  return age >= 0.0 && age <= timeout_;
}

Point3 OutdoorMissionNode::getLocalPosition() const
{
  const auto & p = pose_.pose.position;
  return {p.x, p.y, p.z};
}

GeodeticPosition OutdoorMissionNode::getGpsFixPosition() const
{
  return {fix_.latitude, fix_.longitude, fix_.altitude};
}

std::string OutdoorMissionNode::getTelemetryProblemMessage() const
{
  if (now().nanoseconds() == 0) {
    return "Waiting for ROS clock";
  }
  if (clock_stalled_) {
    return "ROS clock stalled";
  }
  if (!isWithinTimeout(fix_time_)) {
    return "Missing/stale GPS fix";
  }
  if (!isWithinTimeout(satellites_time_)) {
    return "Missing/stale satellite count";
  }
  if (!isWithinTimeout(pose_time_)) {
    return "Missing/stale local pose";
  }
  if (!isWithinTimeout(velocity_time_)) {
    return "Missing/stale local velocity";
  }
  if (!isWithinTimeout(extended_time_)) {
    return "Missing/stale landed state";
  }
  if (fix_.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
    !isValidGeodeticPosition(getGpsFixPosition()))
  {
    return "Invalid GPS fix";
  }
  if (satellites_ > 254 ||
    satellites_ < static_cast<unsigned>(min_satellites_))
  {
    return "Insufficient/unknown satellites";
  }
  const auto & v = velocity_.twist.linear;
  const auto & q = pose_.pose.orientation;
  double quaternion_norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!isFinitePoint(getLocalPosition()) || !isFinitePoint({v.x, v.y, v.z}) ||
    !std::isfinite(quaternion_norm) || std::abs(quaternion_norm - 1.0) > 0.01)
  {
    return "Invalid position, velocity or attitude";
  }
  if (pose_.header.frame_id.empty()) {
    return "Local pose frame is empty";
  }
  if (reference_ && pose_.header.frame_id != local_frame_) {
    return "Local pose frame changed";
  }
  return "";
}

std::string OutdoorMissionNode::getReadinessProblemMessage() const
{
  if (!isWithinTimeout(state_time_) || !state_.connected) {
    return "Missing/stale FCU connection";
  }
  auto problem = getTelemetryProblemMessage();
  if (!problem.empty()) {
    return problem;
  }
  if (!isWithinTimeout(heading_time_) || !std::isfinite(heading_) || heading_ < 0.0 ||
    heading_ >= 360.0)
  {
    return "Missing/stale or invalid compass heading";
  }
  if (!isWithinTimeout(altitude_time_) || !std::isfinite(relative_altitude_)) {
    return "Missing/stale or invalid relative altitude";
  }
  double sample_separation = std::abs((pose_time_ - fix_time_).seconds());
  if (!reference_ && sample_separation > sample_window_) {
    return "GPS and local pose samples are too far apart";
  }
  if (!reference_ && (state_.armed ||
    extended_.landed_state != mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND))
  {
    return "Reference capture requires disarmed vehicle on ground";
  }
  return "";
}

void OutdoorMissionNode::captureMissionReference()
{
  origin_ = getGpsFixPosition();
  Point3 anchor = getLocalPosition();
  origin_source_ = "qualified_fix";
  GeodeticPosition home_position{home_.geo.latitude, home_.geo.longitude, home_.geo.altitude};
  Point3 home_local{home_.position.x, home_.position.y, home_.position.z};
  if (home_received_ && isValidGeodeticPosition(home_position) && isFinitePoint(home_local)) {
    origin_ = home_position;
    anchor = home_local;
    origin_source_ = "fcu_home";
  }
  reference_ = std::make_unique<OutdoorReference>(origin_, anchor, yaw_offset_);
  launch_local_ = getLocalPosition();
  launch_enu_ = reference_->convertLocalToMissionEnu(launch_local_);
  launch_relative_altitude_ = relative_altitude_;
  local_frame_ = pose_.header.frame_id;
  double flight_height = launch_enu_.z + altitude_;
  if (coordinate_mode_ == "gps") {
    auto geodetic_poi = gps_poi_;
    geodetic_poi.altitude_ellipsoid = origin_.altitude_ellipsoid;
    poi_ = reference_->convertGeodeticToMissionEnu(geodetic_poi);
    poi_.z = 0.0;
    for (std::size_t i = 0; i < gps_square_corners_.size(); ++i) {
      auto geodetic = gps_square_corners_[i];
      geodetic.altitude_ellipsoid = origin_.altitude_ellipsoid;
      square_corners_[i] = reference_->convertGeodeticToMissionEnu(geodetic);
      square_corners_[i].z = flight_height;
    }
    square_corners_[4] = square_corners_[0];
  } else {
    square_corners_ = calculateSquareCornerPositions(poi_, side_, flight_height);
  }
  envelope_bound_ = calculateMissionHorizontalDistanceBound(
    poi_, launch_enu_, square_corners_, amplitude_);
  envelope_ok_ = envelope_bound_ + envelope_margin_ <= max_distance_;
  for (std::size_t i = 0; i < square_corners_.size(); ++i) {
    auto geodetic = reference_->convertMissionEnuToGeodetic(square_corners_[i]);
    auto round_trip = reference_->convertGeodeticToMissionEnu(geodetic);
    RCLCPP_INFO(
      get_logger(), "Square %zu: ENU [%.2f, %.2f, %.2f], lat/lon [%.8f, %.8f]",
      i + 1, round_trip.x, round_trip.y, round_trip.z,
      geodetic.latitude, geodetic.longitude);
  }
  RCLCPP_INFO(
    get_logger(), "Reference captured from %s; horizontal bound %.2f m + %.2f m margin; "
    "local flight Z %.2f m. Heading alignment is not automatically verified.",
    origin_source_.c_str(), envelope_bound_, envelope_margin_, launch_local_.z + altitude_);
}

void OutdoorMissionNode::updateMissionReadiness()
{
  std::string problem = getReadinessProblemMessage();
  std::string next_state;
  if (!problem.empty()) {
    ready_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    reason_ = problem;
    next_state = reference_ ? "TELEMETRY_LOST" : "WAIT_READY";
  } else {
    if (ready_since_.nanoseconds() == 0) {
      ready_since_ = now();
    }
    double ready_for = (now() - ready_since_).seconds();
    if (ready_for < readiness_duration_) {
      reason_ = "Waiting for continuous telemetry readiness";
      next_state = "WAIT_READY";
    } else {
      if (!reference_) {
        captureMissionReference();
      }
      if (envelope_ok_) {
        next_state = "REFERENCE_READY";
        reason_ = "Telemetry/reference available; diagnostic stage only, no flight authorization";
      } else {
        next_state = "INVALID_ENVELOPE";
        reason_ = "Horizontal mission bound plus margin exceeds configured limit";
      }
    }
  }
  if (next_state != reported_state_) {
    RCLCPP_INFO(get_logger(), "%s: %s", next_state.c_str(), reason_.c_str());
    reported_state_ = next_state;
    std_msgs::msg::String message;
    message.data = reported_state_;
    state_pub_->publish(message);
  }
}

bool OutdoorMissionNode::isMissionInTerminalState() const
{
  return flight_state_ == OutdoorState::DONE || flight_state_ == OutdoorState::ABORTED ||
         flight_state_ == OutdoorState::FAILED || flight_state_ == OutdoorState::RELEASE_CONTROL;
}

double OutdoorMissionNode::getLocalYaw() const
{
  const auto & q = pose_.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void OutdoorMissionNode::changeMissionPhase(OutdoorState next, const std::string & reason)
{
  flight_state_ = next;
  if (next == OutdoorState::LAND) {
    land_mode_seen_ = false;
  }
  phase_started_ = now();
  phase_started_wall_ = SteadyClock::now();
  settled_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  command_sent_ = false;
  command_complete_ = false;
  command_accepted_ = false;
  ++command_generation_;
  reason_ = reason;
  reported_state_ = getMissionStateName(next);
  std_msgs::msg::String message;
  message.data = reported_state_;
  state_pub_->publish(message);
  RCLCPP_INFO(get_logger(), "%s: %s", reported_state_.c_str(), reason.c_str());
}

void OutdoorMissionNode::requestMissionAbort(const std::string & reason)
{
  if (isMissionInTerminalState() || aborting_) {
    return;
  }
  aborting_ = true;
  abort_reason_ = reason;
  if (!isWithinTimeout(state_time_) || !state_.connected) {
    changeMissionPhase(OutdoorState::RELEASE_CONTROL, reason + "; FCU unavailable");
  } else if (owns_guided_ && (state_.mode == "GUIDED" || state_.mode == "LAND")) {
    changeMissionPhase(OutdoorState::LAND, reason + "; requesting LAND");
  } else if (!state_.armed && !arm_pending_) {
    changeMissionPhase(OutdoorState::ABORTED, reason + "; vehicle remains disarmed");
  } else {
    changeMissionPhase(OutdoorState::RELEASE_CONTROL, reason + "; control not owned");
  }
}

void OutdoorMissionNode::requestFlightMode(const std::string & mode)
{
  if (command_sent_ || !mode_client_->service_is_ready()) {
    return;
  }
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = mode;
  command_sent_ = true;
  unsigned generation = command_generation_;
  mode_client_->async_send_request(
    request,
    [this, generation](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      if (generation != command_generation_) {
        return;
      }
      command_complete_ = true;
      try {
        command_accepted_ = future.get()->mode_sent;
      } catch (const std::exception &) {
        command_accepted_ = false;
      }
    });
}

void OutdoorMissionNode::requestArmingState(bool arm)
{
  if (command_sent_ || !arm_client_->service_is_ready()) {
    return;
  }
  if (!arm && (!isWithinTimeout(extended_time_) ||
    extended_.landed_state != mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND))
  {
    return;
  }
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = arm;
  command_sent_ = true;
  arm_pending_ = true;
  unsigned generation = command_generation_;
  arm_client_->async_send_request(
    request,
    [this, generation](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
      arm_pending_ = false;
      if (generation != command_generation_) {
        return;
      }
      command_complete_ = true;
      try {
        command_accepted_ = future.get()->success;
      } catch (const std::exception &) {
        command_accepted_ = false;
      }
    });
}

void OutdoorMissionNode::requestTakeoff()
{
  if (command_sent_ || !takeoff_client_->service_is_ready()) {
    return;
  }
  GeodeticPosition home_geo{home_.geo.latitude, home_.geo.longitude, home_.geo.altitude};
  if (!home_received_ || !isValidGeodeticPosition(home_geo) ||
    !isFinitePoint({home_.position.x, home_.position.y, home_.position.z}))
  {
    return;
  }
// ArduPilot's GUIDED takeoff altitude is relative to FCU home, not local zero.
  launch_relative_altitude_ = relative_altitude_;
  auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  request->altitude = static_cast<float>(launch_relative_altitude_ + altitude_);
  request->yaw = std::numeric_limits<float>::quiet_NaN(); // Keep current heading.
  if (!std::isfinite(request->altitude) || request->altitude <= 0.0f) {
    requestMissionAbort("Invalid home-relative takeoff altitude");
    return;
  }
  flight_home_ = home_;
  home_locked_ = true;
  command_sent_ = true;
  unsigned generation = command_generation_;
  takeoff_client_->async_send_request(
    request,
    [this, generation](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
      if (generation != command_generation_) {
        return;
      }
      command_complete_ = true;
      try {
        command_accepted_ = future.get()->success;
      } catch (const std::exception &) {
        command_accepted_ = false;
      }
    });
}

std::string OutdoorMissionNode::getFlightNavigationProblemMessage() const
{
  auto problem = getTelemetryProblemMessage();
  if (!problem.empty()) {
    return problem;
  }
  if (flight_state_ == OutdoorState::TAKEOFF &&
    (!isWithinTimeout(altitude_time_) || !std::isfinite(relative_altitude_)))
  {
    return "Missing/stale or invalid takeoff relative altitude";
  }
  auto position = getLocalPosition();
  if (std::hypot(position.x - launch_local_.x, position.y - launch_local_.y) > max_distance_ ||
    position.z - launch_local_.z > max_altitude_)
  {
    return "Measured flight envelope exceeded";
  }
  if (home_locked_) {
    GeodeticPosition current{home_.geo.latitude, home_.geo.longitude, home_.geo.altitude};
    GeodeticPosition captured{flight_home_.geo.latitude, flight_home_.geo.longitude,
      flight_home_.geo.altitude};
    if (!isValidGeodeticPosition(current) || !isFinitePoint({home_.position.x, home_.position.y,
        home_.position.z}))
    {
      return "Invalid FCU home";
    }
    auto current_home = reference_->convertGeodeticToMissionEnu(current);
    auto captured_home = reference_->convertGeodeticToMissionEnu(captured);
    if (std::hypot(current_home.x - captured_home.x, current_home.y - captured_home.y) > 0.5 ||
      std::abs(current_home.z - captured_home.z) > 0.5 ||
      std::hypot(
      home_.position.x - flight_home_.position.x,
      home_.position.y - flight_home_.position.y) > 0.5 ||
      std::abs(home_.position.z - flight_home_.position.z) > 0.5)
    {
      return "FCU home changed after takeoff reference was fixed";
    }

  }
  return "";
}

bool OutdoorMissionNode::isSettledAtPosition(Point3 destination)
{
  auto position = getLocalPosition();
  const auto & velocity = velocity_.twist.linear;
  double speed = std::hypot(std::hypot(velocity.x, velocity.y), velocity.z);
  bool within_tolerance = std::hypot(
    position.x - destination.x,
    position.y - destination.y) <= position_tolerance_ &&
    std::abs(position.z - destination.z) <= altitude_tolerance_ &&
    speed < velocity_tolerance_ &&
    std::abs(wrapAngleToPi(getLocalYaw() - target_yaw_)) <= yaw_tolerance_;
  if (!within_tolerance) {
    settled_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return false;
  }
  if (settled_since_.nanoseconds() == 0) {
    settled_since_ = now();
  }
  return (now() - settled_since_).seconds() >= settle_duration_;
}

Point3 OutdoorMissionNode::getCurrentSquareCornerPosition() const
{
  return reference_->convertMissionEnuToLocal(square_corners_[square_corner_]);
}

Point3 OutdoorMissionNode::getFigureEightCentreEnuPosition() const
{
  return {poi_.x, poi_.y, launch_enu_.z + altitude_};
}

void OutdoorMissionNode::updateFlightStateMachine()
{
  if (isMissionInTerminalState()) {
    return;
  }
  if (flight_state_ == OutdoorState::WAIT_READY) {
    updateMissionReadiness();
    if (reported_state_ != "REFERENCE_READY") {
      return;
    }
    auto problem = getFlightNavigationProblemMessage();
    if (!problem.empty() || state_.armed) {
      changeMissionPhase(OutdoorState::FAILED,
          problem.empty() ? "Vehicle unexpectedly armed" : problem);
      return;
    }
    target_ = {launch_local_.x, launch_local_.y, launch_local_.z + altitude_};
    target_yaw_ = getLocalYaw();
    changeMissionPhase(OutdoorState::SET_GUIDED, "Takeoff/return check started");
  }
  if (!isWithinTimeout(state_time_) || !state_.connected) {
    changeMissionPhase(OutdoorState::RELEASE_CONTROL,
        "FCU connection lost; defer to onboard failsafes");
    return;
  }
  if (flight_state_ == OutdoorState::SET_GUIDED && state_.armed) {
    changeMissionPhase(OutdoorState::RELEASE_CONTROL,
        "Vehicle unexpectedly armed before mission arming");
    return;
  }
  bool landing = flight_state_ == OutdoorState::LAND || flight_state_ == OutdoorState::DISARM;
  if (owns_guided_ && !landing && state_.mode != "GUIDED") {
    changeMissionPhase(OutdoorState::RELEASE_CONTROL, "Pilot/autopilot changed mode");
    return;
  }
  if (landing && state_.mode == "LAND") {
    land_mode_seen_ = true;
  }
  if (landing && (state_.mode != "GUIDED" && state_.mode != "LAND")) {
    changeMissionPhase(OutdoorState::RELEASE_CONTROL,
        "Pilot/autopilot changed mode during landing");
    return;
  }
  if (landing && land_mode_seen_ && state_.mode != "LAND") {
    changeMissionPhase(OutdoorState::RELEASE_CONTROL, "Pilot left LAND mode; releasing control");
    return;
  }
  if (!landing) {
    auto problem = getFlightNavigationProblemMessage();
    if (!problem.empty()) {
      requestMissionAbort(problem);
      return;
    }

  }
  const bool command_phase = flight_state_ == OutdoorState::SET_GUIDED ||
    flight_state_ == OutdoorState::ARM || flight_state_ == OutdoorState::DISARM;
  double limit = phase_timeout_;
  if (command_phase) {
    limit = command_timeout_;
  } else if (flight_state_ == OutdoorState::TAKEOFF) {
    limit = takeoff_timeout_;
  } else if (flight_state_ == OutdoorState::LAND) {
    limit = landing_timeout_;
  }
  const double wall_elapsed =
    std::chrono::duration<double>(SteadyClock::now() - phase_started_wall_).count();
  const double phase_elapsed = command_phase ?
    wall_elapsed : (now() - phase_started_).seconds();
  const bool response_timed_out = !command_phase && command_sent_ && !command_complete_ &&
    wall_elapsed > command_timeout_;
  if (phase_elapsed > limit || response_timed_out) {
    if (landing) {
      changeMissionPhase(OutdoorState::FAILED,
          "Landing/disarm timed out; manual recovery required");
    } else {
      requestMissionAbort("Mission phase timed out");
    }
    return;
  }
  if (command_complete_ && !command_accepted_) {
    if (landing) {
      changeMissionPhase(OutdoorState::FAILED, "LAND/disarm request rejected");
    } else {
      requestMissionAbort("MAVROS command rejected");
    }
    return;
  }
  if (flight_state_ != OutdoorState::ARM && flight_state_ != OutdoorState::SET_GUIDED &&
    !landing && !state_.armed)
  {
    requestMissionAbort("Unexpected disarm");
    return;
  }
  switch (flight_state_) {
    case OutdoorState::SET_GUIDED:
      requestFlightMode("GUIDED");
      if (command_complete_ && command_accepted_ && state_.mode == "GUIDED") {
        owns_guided_ = true;
        changeMissionPhase(OutdoorState::ARM, "GUIDED confirmed");
      }
      break;
    case OutdoorState::ARM:
      requestArmingState(true);
      if (command_complete_ && command_accepted_ && state_.armed) {
        changeMissionPhase(OutdoorState::TAKEOFF, "Arming confirmed");
      }
      break;
    case OutdoorState::TAKEOFF:
      requestTakeoff();
      if (command_complete_ && command_accepted_ &&
        extended_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR &&
        std::abs(pose_.pose.position.z - target_.z) <= altitude_tolerance_ &&
        std::abs(relative_altitude_ - launch_relative_altitude_ - altitude_) <= altitude_tolerance_)
      {
        changeMissionPhase(OutdoorState::STABILIZE, "Airborne at target altitude");
      }
      break;
    case OutdoorState::STABILIZE:
      if (isSettledAtPosition(target_)) {
        target_ = getLocalPosition();
        previous_target_time_ = now();
        if (execute_square_) {
          square_corner_ = 0;
          changeMissionPhase(OutdoorState::SQUARE, "Flying to square corner 1 of 5");
        } else {
          changeMissionPhase(OutdoorState::RETURN_HOME,
            "Settled; returning to launch before landing");
        }
      }
      break;
    case OutdoorState::SQUARE:
      if (isSettledAtPosition(getCurrentSquareCornerPosition())) {
        if (square_corner_ < 4) {
          ++square_corner_;
          changeMissionPhase(
            OutdoorState::SQUARE,
            "Flying to square corner " + std::to_string(square_corner_ + 1) + " of 5");
        } else {
          target_ = getLocalPosition();
          previous_target_time_ = now();
          if (execute_figure_eight_) {
            changeMissionPhase(OutdoorState::MOVE_TO_CENTER, "Square closed; moving above POI");
          } else {
            changeMissionPhase(
              OutdoorState::RETURN_HOME,
              "Square closed and settled; returning to launch");
          }
        }
      }
      break;
    case OutdoorState::MOVE_TO_CENTER:
      if (isSettledAtPosition(reference_->convertMissionEnuToLocal(
        getFigureEightCentreEnuPosition())))
      {
        target_ = reference_->convertMissionEnuToLocal(getFigureEightCentreEnuPosition());
        target_yaw_ = unwrapAngle(target_yaw_, pi / 4.0 + yaw_offset_);
        changeMissionPhase(OutdoorState::ALIGN_TANGENT,
          "Centre settled; aligning to initial tangent");
      }
      break;
    case OutdoorState::ALIGN_TANGENT:
      if (isSettledAtPosition(target_)) {
        previous_target_time_ = now();
        changeMissionPhase(OutdoorState::FIGURE_EIGHT,
          "Tangent settled; starting one figure-eight lap");
      }
      break;
    case OutdoorState::FIGURE_EIGHT:
      if ((now() - phase_started_).seconds() >= 2.0 * pi / omega_) {
        target_ = reference_->convertMissionEnuToLocal(getFigureEightCentreEnuPosition());
        target_yaw_ = unwrapAngle(target_yaw_, pi / 4.0 + yaw_offset_);
        changeMissionPhase(OutdoorState::SETTLE_CENTER, "Lap complete; settling at centre");
      }
      break;
    case OutdoorState::SETTLE_CENTER:
      if (isSettledAtPosition(target_)) {
        target_ = getLocalPosition();
        previous_target_time_ = now();
        changeMissionPhase(OutdoorState::RETURN_HOME, "Centre settled; returning to launch");
      }
      break;
    case OutdoorState::RETURN_HOME:
      if (isSettledAtPosition({launch_local_.x, launch_local_.y, launch_local_.z + altitude_})) {
        changeMissionPhase(OutdoorState::LAND, "Return settled; requesting LAND");
      }
      break;
    case OutdoorState::LAND:
      requestFlightMode("LAND");
      if (command_complete_ && command_accepted_ && state_.mode == "LAND" &&
        isWithinTimeout(extended_time_) &&
        extended_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND &&
        !arm_pending_)
      {
        if (state_.armed) {
          changeMissionPhase(OutdoorState::DISARM, "Ground confirmed");
        } else {
          changeMissionPhase(
            aborting_ ? OutdoorState::ABORTED : OutdoorState::DONE,
            "Ground and disarm confirmed");
        }
      }
      break;
    case OutdoorState::DISARM:
      requestArmingState(false);
      if (command_complete_ && command_accepted_ && !state_.armed) {
        changeMissionPhase(
          aborting_ ? OutdoorState::ABORTED : OutdoorState::DONE,
          "Disarm confirmed after landing");
      }
      break;
    default:
      break;
  }
}

void OutdoorMissionNode::publishFlightSetpoint()
{
  bool taking_off = flight_state_ == OutdoorState::TAKEOFF && command_complete_ &&
    command_accepted_ &&
    extended_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR;
  bool holding = flight_state_ == OutdoorState::STABILIZE ||
    flight_state_ == OutdoorState::ALIGN_TANGENT || flight_state_ == OutdoorState::SETTLE_CENTER;
  bool returning = flight_state_ == OutdoorState::RETURN_HOME;
  bool square = flight_state_ == OutdoorState::SQUARE;
  bool centring = flight_state_ == OutdoorState::MOVE_TO_CENTER;
  bool curve = flight_state_ == OutdoorState::FIGURE_EIGHT;
  if (!taking_off && !holding && !returning && !square && !centring && !curve) {
    return;
  }
  if (!isWithinTimeout(state_time_) || !state_.connected || state_.mode != "GUIDED") {
    changeMissionPhase(OutdoorState::RELEASE_CONTROL, "Lost GUIDED control; setpoints stopped");
    return;
  }
  auto problem = getFlightNavigationProblemMessage();
  if (!problem.empty() || !state_.armed) {
    requestMissionAbort(problem.empty() ? "Unexpected disarm" : problem);
    return;
  }
  if (returning || square || centring || curve) {
    auto current_time = now();
    double elapsed = (current_time - previous_target_time_).seconds();
    previous_target_time_ = current_time;
    if (elapsed < 0.0 || elapsed > timeout_) {
      requestMissionAbort("Setpoint clock discontinuity");
      return;
    }
    if (curve) {
      const double curve_elapsed = std::clamp(
        (current_time - phase_started_).seconds(),
        0.0, 2.0 * pi / omega_);
      const auto sample = calculateFigureEightSample(getFigureEightCentreEnuPosition(), amplitude_,
          omega_,
          curve_elapsed);
      target_ = reference_->convertMissionEnuToLocal(sample.position);
      target_yaw_ = unwrapAngle(target_yaw_, sample.yaw + yaw_offset_);
    } else {
      Point3 destination{launch_local_.x, launch_local_.y, launch_local_.z + altitude_};
      if (square || centring) {
        destination =
          square ? getCurrentSquareCornerPosition() :
          reference_->convertMissionEnuToLocal(getFigureEightCentreEnuPosition());
        const auto poi_local = reference_->convertMissionEnuToLocal(poi_);
        target_yaw_ = calculatePoiFacingYaw(getLocalPosition(), poi_local, target_yaw_, 0.2);
      }
      double east = destination.x - target_.x;
      double north = destination.y - target_.y;
      double distance = std::hypot(east, north);
      if (distance > 1e-6) {
        if (returning) {
          target_yaw_ = unwrapAngle(target_yaw_, std::atan2(north, east));
        }
            // Return faces travel; square yaw follows the POI independently.
        if (square || centring ||
          std::abs(wrapAngleToPi(getLocalYaw() - target_yaw_)) <= yaw_tolerance_)
        {
          double step = std::min(return_speed_ * elapsed, distance);
          target_.x += step * east / distance;
          target_.y += step * north / distance;
        }
      }
      target_.z = destination.z;
    }
  }
  if (!taking_off && std::hypot(
    std::hypot(
    target_.x - pose_.pose.position.x,
    target_.y - pose_.pose.position.y), target_.z - pose_.pose.position.z) > max_tracking_error_)
  {
    requestMissionAbort("Position target tracking error exceeded");
    return;
  }
  mavros_msgs::msg::PositionTarget message;
  message.header.stamp = now();
  message.header.frame_id = local_frame_;
  message.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  message.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
    mavros_msgs::msg::PositionTarget::IGNORE_VY | mavros_msgs::msg::PositionTarget::IGNORE_VZ |
    mavros_msgs::msg::PositionTarget::IGNORE_AFX | mavros_msgs::msg::PositionTarget::IGNORE_AFY |
    mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
    mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
  message.position.x = target_.x;
  message.position.y = target_.y;
  message.position.z = target_.z;
  message.yaw = static_cast<float>(target_yaw_);
  setpoint_pub_->publish(message);
}

void OutdoorMissionNode::addDiagnosticValue(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue entry;
  entry.key = key;
  entry.value = value;
  status.values.push_back(entry);
}

void OutdoorMissionNode::publishDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "outdoor_mission/telemetry";
  status.hardware_id = "mavros";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  if (reported_state_ == "REFERENCE_READY") {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  }
  status.message = reported_state_ + ": " + reason_;
  addDiagnosticValue(status, "state", reported_state_);
  addDiagnosticValue(status, "flight_enabled", enable_flight_ ? "true" : "false");
  addDiagnosticValue(
    status, "square_corner", flight_state_ == OutdoorState::SQUARE ?
    std::to_string(square_corner_ + 1) : "0");
  addDiagnosticValue(status, "coordinate_mode", coordinate_mode_);
  addDiagnosticValue(status, "abort_reason", abort_reason_);
  addDiagnosticValue(status, "satellites", std::to_string(satellites_));
  addDiagnosticValue(status, "fix_status", std::to_string(fix_.status.status));
  if (fix_time_.nanoseconds() != 0) {
    addDiagnosticValue(status, "latitude", std::to_string(fix_.latitude));
    addDiagnosticValue(status, "longitude", std::to_string(fix_.longitude));
    addDiagnosticValue(status, "altitude_ellipsoid", std::to_string(fix_.altitude));
    addDiagnosticValue(
      status, "fix_age", std::to_string(
        (now() - fix_time_).seconds()));
  }
  if (isWithinTimeout(heading_time_) && std::isfinite(heading_) && heading_ >= 0.0 &&
    heading_ < 360.0)
  {
    double compass_yaw = convertHeadingDegreesToEnuYaw(heading_);
    addDiagnosticValue(status, "heading_degrees", std::to_string(heading_));
    addDiagnosticValue(status, "compass_yaw_enu_degrees", std::to_string(compass_yaw * 180.0 / pi));
    const auto & q = pose_.pose.orientation;
    double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (isWithinTimeout(pose_time_) && std::isfinite(norm) && std::abs(norm - 1.0) <= 0.01) {
      double local_yaw = getLocalYaw();
      addDiagnosticValue(status, "local_yaw_degrees", std::to_string(local_yaw * 180.0 / pi));
      addDiagnosticValue(
        status, "yaw_alignment_error_degrees", std::to_string(
        wrapAngleToPi(local_yaw - compass_yaw - yaw_offset_) * 180.0 / pi));
    }
  }
  if (reference_) {
    addDiagnosticValue(status, "origin_source", origin_source_);
    addDiagnosticValue(status, "origin_latitude", std::to_string(origin_.latitude));
    addDiagnosticValue(status, "origin_longitude", std::to_string(origin_.longitude));
    addDiagnosticValue(
      status, "horizontal_bound_with_margin",
      std::to_string(envelope_bound_ + envelope_margin_));
    addDiagnosticValue(status, "flight_local_z", std::to_string(launch_local_.z + altitude_));
    if (isWithinTimeout(pose_time_) && isFinitePoint(getLocalPosition())) {
      auto enu = reference_->convertLocalToMissionEnu(getLocalPosition());
      addDiagnosticValue(status, "local_x", std::to_string(pose_.pose.position.x));
      addDiagnosticValue(status, "local_y", std::to_string(pose_.pose.position.y));
      addDiagnosticValue(status, "local_z", std::to_string(pose_.pose.position.z));
      addDiagnosticValue(status, "mission_east", std::to_string(enu.x));
      addDiagnosticValue(status, "mission_north", std::to_string(enu.y));
      addDiagnosticValue(status, "mission_up", std::to_string(enu.z));
      auto geodetic = reference_->convertMissionEnuToGeodetic(enu);
      addDiagnosticValue(status, "local_derived_latitude", std::to_string(geodetic.latitude));
      addDiagnosticValue(status, "local_derived_longitude", std::to_string(geodetic.longitude));
      addDiagnosticValue(
        status, "local_derived_altitude_ellipsoid",
        std::to_string(geodetic.altitude_ellipsoid));
      addDiagnosticValue(
        status, "height_above_launch_local",
        std::to_string(pose_.pose.position.z - launch_local_.z));
    }
    if (isWithinTimeout(fix_time_) && fix_.status.status >= 0 &&
      isValidGeodeticPosition(getGpsFixPosition()))
    {
      auto enu = reference_->convertGeodeticToMissionEnu(getGpsFixPosition());
      addDiagnosticValue(status, "gps_east", std::to_string(enu.x));
      addDiagnosticValue(status, "gps_north", std::to_string(enu.y));
      addDiagnosticValue(status, "gps_up", std::to_string(enu.z));
      if (isWithinTimeout(pose_time_) && isFinitePoint(getLocalPosition())) {
        auto local_enu = reference_->convertLocalToMissionEnu(getLocalPosition());
        double error = std::hypot(enu.x - local_enu.x, enu.y - local_enu.y);
        addDiagnosticValue(status, "gps_local_error_m", std::to_string(error));
        addDiagnosticValue(status, "gps_local_disagreement",
            error > max_reference_error_ ? "true" : "false");
      }
    }
    if (isWithinTimeout(altitude_time_) && std::isfinite(relative_altitude_)) {
      addDiagnosticValue(
        status, "height_above_launch_relative",
        std::to_string(relative_altitude_ - launch_relative_altitude_));
      if (isWithinTimeout(pose_time_) && isFinitePoint(getLocalPosition())) {
        double error = std::abs((pose_.pose.position.z - launch_local_.z) -
            (relative_altitude_ - launch_relative_altitude_));
        addDiagnosticValue(status, "altitude_difference_m", std::to_string(error));
        addDiagnosticValue(status, "altitude_disagreement",
            error > altitude_disagreement_ ? "true" : "false");
      }
    }
  }
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = now();
  message.status.push_back(status);
  diagnostics_pub_->publish(message);
  RCLCPP_INFO(get_logger(), "%s", status.message.c_str());
}

}  // namespace mission_executor
