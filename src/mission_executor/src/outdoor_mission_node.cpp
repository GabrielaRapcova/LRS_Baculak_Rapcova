#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
#include "mavros_msgs/msg/home_position.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/message_interval.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "mission_executor/outdoor_geometry.hpp"

namespace mission_executor
{
using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;
constexpr double pi = 3.14159265358979323846;

enum class OutdoorState
{
    WAIT_READY, SET_GUIDED, ARM, TAKEOFF, STABILIZE, RETURN_HOME,
    LAND, DISARM, DONE, ABORTED, RELEASE_CONTROL, FAILED
};

const char * state_name(OutdoorState state)
{
    switch (state) {
        case OutdoorState::WAIT_READY: return "WAIT_READY";
        case OutdoorState::SET_GUIDED: return "SET_GUIDED";
        case OutdoorState::ARM: return "ARM";
        case OutdoorState::TAKEOFF: return "TAKEOFF";
        case OutdoorState::STABILIZE: return "STABILIZE";
        case OutdoorState::RETURN_HOME: return "RETURN_HOME";
        case OutdoorState::LAND: return "LAND";
        case OutdoorState::DISARM: return "DISARM";
        case OutdoorState::DONE: return "DONE";
        case OutdoorState::ABORTED: return "ABORTED";
        case OutdoorState::RELEASE_CONTROL: return "RELEASE_CONTROL";
        case OutdoorState::FAILED: return "FAILED";
    }
    return "UNKNOWN";
}

// Telemetry, phase changes and service completions use the default callback group.
// The independent setpoint group shares data through short mutex-protected updates.
class OutdoorMissionNode : public rclcpp::Node
{
public:
    OutdoorMissionNode() : Node("outdoor_mission_node")
    {
        poi_ = {declare_parameter("poi_east", 2.5), declare_parameter("poi_north", 2.5), 0.0};
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
        readiness_duration_ = declare_parameter("gps_ready_duration", 3.0);
        sample_window_ = declare_parameter("reference_sample_window", 0.5);
        enable_flight_ = declare_parameter("enable_flight", false);
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
        validate_parameters();

        auto sensor_qos = rclcpp::SensorDataQoS();
        state_sub_ = create_subscription<mavros_msgs::msg::State>("mavros/state", sensor_qos,
            [this](mavros_msgs::msg::State::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                state_ = *msg; state_time_ = SteadyClock::now();
            });
        extended_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
            "mavros/extended_state", sensor_qos,
            [this](mavros_msgs::msg::ExtendedState::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                extended_ = *msg; extended_time_ = SteadyClock::now();
            });
        fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "mavros/global_position/global", sensor_qos,
            [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                fix_ = *msg; fix_time_ = SteadyClock::now();
            });
        satellites_sub_ = create_subscription<std_msgs::msg::UInt32>(
            "mavros/global_position/raw/satellites", sensor_qos,
            [this](std_msgs::msg::UInt32::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                satellites_ = msg->data; satellites_time_ = SteadyClock::now();
            });
        home_sub_ = create_subscription<mavros_msgs::msg::HomePosition>(
            "mavros/home_position/home", rclcpp::QoS(1).reliable().transient_local(),
            [this](mavros_msgs::msg::HomePosition::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                home_ = *msg; home_received_ = true;
            });
        heading_sub_ = create_subscription<std_msgs::msg::Float64>(
            "mavros/global_position/compass_hdg", sensor_qos,
            [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                heading_ = msg->data; heading_time_ = SteadyClock::now();
            });
        relative_altitude_sub_ = create_subscription<std_msgs::msg::Float64>(
            "mavros/global_position/rel_alt", sensor_qos,
            [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                relative_altitude_ = msg->data; altitude_time_ = SteadyClock::now();
            });
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "mavros/local_position/pose", sensor_qos,
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                pose_ = *msg; pose_time_ = SteadyClock::now();
            });
        velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "mavros/local_position/velocity_local", sensor_qos,
            [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                velocity_ = *msg; velocity_time_ = SteadyClock::now();
            });
        state_pub_ = create_publisher<std_msgs::msg::String>(
            "outdoor_mission/state", rclcpp::QoS(1).reliable().transient_local());
        diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "outdoor_mission/diagnostics", 10);
        interval_client_ = create_client<mavros_msgs::srv::MessageInterval>(
            "mavros/set_message_interval");
        if (enable_flight_) {
            mode_client_ = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
            arm_client_ = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
            takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");
            setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);
            abort_service_ = create_service<std_srvs::srv::Trigger>("outdoor_mission/abort",
                [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    response->success = !terminal();
                    response->message = response->success ? "Abort accepted; monitor mission state" : "Mission already terminal";
                    if (response->success) {begin_abort("Operator requested abort");}
                });
            setpoint_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            setpoint_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / 30.0),
                [this]() {publish_setpoint();}, setpoint_group_);
        }
        readiness_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
            std::lock_guard<std::mutex> lock(mutex_);
            update_landed_state_stream();
            if (enable_flight_) {update_flight();} else {update_readiness();}
        });
        reporting_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {report();});
        RCLCPP_INFO(get_logger(), "Outdoor mode: %s; configured yaw offset %.1f degrees.",
            enable_flight_ ? "TAKEOFF/RETURN FLIGHT CHECK" : "TELEMETRY ONLY", yaw_offset_ * 180.0 / pi);
    }

private:
    void update_landed_state_stream()
    {
        const bool connected = fresh(state_time_) && state_.connected;
        if (!connected) {
            if (interval_pending_) {
                interval_client_->remove_pending_request(interval_request_id_);
            }
            interval_pending_ = false;
            interval_configured_ = false;
            interval_attempt_time_ = TimePoint{};
            ++interval_generation_;
            return;
        }
        if (interval_configured_) {return;}
        const auto current_time = SteadyClock::now();
        if (interval_attempt_time_ != TimePoint{} &&
            std::chrono::duration<double>(current_time - interval_attempt_time_).count() < 5.0) {
            return;
        }
        if (interval_pending_) {
            interval_client_->remove_pending_request(interval_request_id_);
            interval_pending_ = false;
            RCLCPP_WARN(get_logger(), "Landed-state stream request timed out; retrying");
        }
        if (!interval_client_->service_is_ready()) {return;}
        auto request = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
        request->message_id = 245;  // MAVLink EXTENDED_SYS_STATE.
        request->message_rate = 2.0;
        interval_attempt_time_ = current_time;
        interval_pending_ = true;
        const auto generation = ++interval_generation_;
        auto result = interval_client_->async_send_request(request,
            [this, generation](rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedFuture future) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (generation != interval_generation_) {return;}
                interval_pending_ = false;
                interval_configured_ = future.get()->success;
                if (interval_configured_) {
                    RCLCPP_INFO(get_logger(), "Requested landed-state telemetry at 2 Hz");
                } else {
                    RCLCPP_WARN(get_logger(), "Landed-state stream request rejected; retrying in 5 seconds");
                }
            });
        interval_request_id_ = result.request_id;
    }

    void validate_parameters()
    {
        for (double value : {altitude_, side_, amplitude_, omega_, max_speed_, max_distance_,
                max_altitude_, timeout_, readiness_duration_, sample_window_, position_tolerance_,
                altitude_tolerance_, velocity_tolerance_, yaw_tolerance_, settle_duration_, return_speed_,
                command_timeout_, takeoff_timeout_, phase_timeout_, landing_timeout_, max_tracking_error_,
                max_reference_error_, altitude_disagreement_}) {
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::invalid_argument("Outdoor dimensions, limits and durations must be finite and positive");
            }
        }
        if (!is_finite(poi_) || !std::isfinite(yaw_offset_) ||
            !std::isfinite(envelope_margin_) || envelope_margin_ < 0.0 ||
            min_satellites_ < 1 || min_satellites_ > 254) {
            throw std::invalid_argument("Invalid POI, heading offset, margin or satellite threshold");
        }
        if (altitude_ != 5.0 || side_ != 5.0 || amplitude_ != 3.0) {
            throw std::invalid_argument("A1.3 requires altitude=5, square_side=5, figure8_amplitude=3");
        }
        if (figure_eight_peak_speed(amplitude_, omega_) > max_speed_) {
            throw std::invalid_argument("Figure-eight peak speed exceeds max_speed");
        }
        if (return_speed_ > max_speed_) {
            throw std::invalid_argument("transfer_speed exceeds max_speed");
        }
        if (altitude_ + envelope_margin_ > max_altitude_) {
            throw std::invalid_argument("Flight altitude plus margin exceeds max_altitude");
        }
    }

    bool fresh(TimePoint received) const
    {
        if (received == TimePoint{}) {
            return false;
        }
        return std::chrono::duration<double>(SteadyClock::now() - received).count() <= timeout_;
    }

    Point3 local_position() const
    {
        const auto & p = pose_.pose.position;
        return {p.x, p.y, p.z};
    }

    GeodeticPosition gps_position() const
    {
        return {fix_.latitude, fix_.longitude, fix_.altitude};
    }

    std::string readiness_problem() const
    {
        if (!fresh(state_time_) || !state_.connected) {return "Missing/stale FCU connection";}
        if (!fresh(fix_time_)) {return "Missing/stale GPS fix";}
        if (!fresh(satellites_time_)) {return "Missing/stale satellite count";}
        if (!fresh(pose_time_)) {return "Missing/stale local pose";}
        if (!fresh(velocity_time_)) {return "Missing/stale local velocity";}
        if (!fresh(heading_time_)) {return "Missing/stale compass heading";}
        if (!fresh(altitude_time_)) {return "Missing/stale relative altitude";}
        if (!fresh(extended_time_)) {return "Missing/stale landed state";}
        if (fix_.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX || !is_valid(gps_position())) {
            return "Invalid GPS fix";
        }
        if (satellites_ == 255 || satellites_ > 254 || satellites_ < static_cast<unsigned>(min_satellites_)) {
            return "Insufficient/unknown satellites";
        }
        const auto & v = velocity_.twist.linear;
        const auto & q = pose_.pose.orientation;
        double quaternion_norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (!is_finite(local_position()) || !is_finite({v.x, v.y, v.z}) ||
            !std::isfinite(quaternion_norm) || std::abs(quaternion_norm - 1.0) > 0.01 ||
            !std::isfinite(heading_) || heading_ < 0.0 || heading_ >= 360.0 ||
            !std::isfinite(relative_altitude_)) {
            return "Invalid position, velocity, attitude, heading or altitude";
        }
        double sample_separation = std::abs(std::chrono::duration<double>(pose_time_ - fix_time_).count());
        if (!reference_ && sample_separation > sample_window_) {
            return "GPS and local pose samples are too far apart";
        }
        if (!reference_ && (state_.armed ||
            extended_.landed_state != mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND)) {
            return "Reference capture requires disarmed vehicle on ground";
        }
        if (pose_.header.frame_id.empty()) {return "Local pose frame is empty";}
        if (reference_ && pose_.header.frame_id != local_frame_) {return "Local pose frame changed";}
        return "";
    }

    void capture_reference()
    {
        origin_ = gps_position();
        Point3 anchor = local_position();
        origin_source_ = "qualified_fix";
        GeodeticPosition home_position{home_.geo.latitude, home_.geo.longitude, home_.geo.altitude};
        Point3 home_local{home_.position.x, home_.position.y, home_.position.z};
        if (home_received_ && is_valid(home_position) && is_finite(home_local)) {
            origin_ = home_position;
            anchor = home_local;
            origin_source_ = "fcu_home";
        }
        reference_ = std::make_unique<OutdoorReference>(origin_, anchor, yaw_offset_);
        launch_local_ = local_position();
        launch_enu_ = reference_->from_local(launch_local_);
        launch_relative_altitude_ = relative_altitude_;
        local_frame_ = pose_.header.frame_id;
        envelope_bound_ = mission_horizontal_bound(poi_, launch_enu_, side_, amplitude_);
        envelope_ok_ = envelope_bound_ + envelope_margin_ <= max_distance_;
        double flight_height = launch_enu_.z + altitude_;
        auto corners = square_corners(poi_, side_, flight_height);
        for (std::size_t i = 0; i < corners.size(); ++i) {
            auto geodetic = reference_->to_geodetic(corners[i]);
            auto round_trip = reference_->to_enu(geodetic);
            RCLCPP_INFO(get_logger(), "Square %zu: ENU [%.2f, %.2f, %.2f], lat/lon [%.8f, %.8f]",
                i + 1, round_trip.x, round_trip.y, round_trip.z,
                geodetic.latitude, geodetic.longitude);
        }
        RCLCPP_INFO(get_logger(), "Reference captured from %s; horizontal bound %.2f m + %.2f m margin; "
            "local flight Z %.2f m. Heading alignment is not automatically verified.",
            origin_source_.c_str(), envelope_bound_, envelope_margin_, launch_local_.z + altitude_);
    }

    void update_readiness()
    {
        std::string problem = readiness_problem();
        std::string next_state;
        if (!problem.empty()) {
            ready_since_ = TimePoint{};
            reason_ = problem;
            next_state = reference_ ? "TELEMETRY_LOST" : "WAIT_READY";
        } else {
            if (ready_since_ == TimePoint{}) {ready_since_ = SteadyClock::now();}
            double ready_for = std::chrono::duration<double>(SteadyClock::now() - ready_since_).count();
            if (ready_for < readiness_duration_) {
                reason_ = "Waiting for continuous telemetry readiness";
                next_state = "WAIT_READY";
            } else {
                if (!reference_) {capture_reference();}
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

    bool terminal() const
    {
        return flight_state_ == OutdoorState::DONE || flight_state_ == OutdoorState::ABORTED ||
               flight_state_ == OutdoorState::FAILED || flight_state_ == OutdoorState::RELEASE_CONTROL;
    }

    double local_yaw() const
    {
        const auto & q = pose_.pose.orientation;
        return std::atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z));
    }

    void change_phase(OutdoorState next, const std::string & reason)
    {
        flight_state_ = next;
        if (next == OutdoorState::LAND) {land_mode_seen_ = false;}
        phase_started_ = SteadyClock::now();
        settled_since_ = TimePoint{};
        command_sent_ = false;
        command_complete_ = false;
        command_accepted_ = false;
        ++command_generation_;
        reason_ = reason;
        reported_state_ = state_name(next);
        std_msgs::msg::String message;
        message.data = reported_state_;
        state_pub_->publish(message);
        RCLCPP_INFO(get_logger(), "%s: %s", reported_state_.c_str(), reason.c_str());
    }

    void begin_abort(const std::string & reason)
    {
        if (terminal() || aborting_) {return;}
        aborting_ = true;
        abort_reason_ = reason;
        if (!fresh(state_time_) || !state_.connected) {
            change_phase(OutdoorState::RELEASE_CONTROL, reason + "; FCU unavailable");
        } else if (owns_guided_ && (state_.mode == "GUIDED" || state_.mode == "LAND")) {
            change_phase(OutdoorState::LAND, reason + "; requesting LAND");
        } else if (!state_.armed && !arm_pending_) {
            change_phase(OutdoorState::ABORTED, reason + "; vehicle remains disarmed");
        } else {
            change_phase(OutdoorState::RELEASE_CONTROL, reason + "; control not owned");
        }
    }

    void request_mode(const std::string & mode)
    {
        if (command_sent_ || !mode_client_->service_is_ready()) {return;}
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;
        command_sent_ = true;
        unsigned generation = command_generation_;
        mode_client_->async_send_request(request,
            [this, generation](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (generation != command_generation_) {return;}
                command_complete_ = true;
                try {command_accepted_ = future.get()->mode_sent;}
                catch (const std::exception &) {command_accepted_ = false;}
            });
    }

    void request_arm(bool arm)
    {
        if (command_sent_ || !arm_client_->service_is_ready()) {return;}
        if (!arm && (!fresh(extended_time_) ||
            extended_.landed_state != mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND)) {
            return;
        }
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = arm;
        command_sent_ = true;
        arm_pending_ = true;
        unsigned generation = command_generation_;
        arm_client_->async_send_request(request,
            [this, generation](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                std::lock_guard<std::mutex> lock(mutex_);
                arm_pending_ = false;
                if (generation != command_generation_) {return;}
                command_complete_ = true;
                try {command_accepted_ = future.get()->success;}
                catch (const std::exception &) {command_accepted_ = false;}
            });
    }

    void request_takeoff()
    {
        if (command_sent_ || !takeoff_client_->service_is_ready()) {return;}
        GeodeticPosition home_geo{home_.geo.latitude, home_.geo.longitude, home_.geo.altitude};
        if (!home_received_ || !is_valid(home_geo) ||
            !is_finite({home_.position.x, home_.position.y, home_.position.z})) {return;}
        // ArduPilot's GUIDED takeoff altitude is relative to FCU home, not local zero.
        launch_relative_altitude_ = relative_altitude_;
        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->altitude = static_cast<float>(launch_relative_altitude_ + altitude_);
        request->yaw = std::numeric_limits<float>::quiet_NaN();  // Keep current heading.
        if (!std::isfinite(request->altitude) || request->altitude <= 0.0f) {
            begin_abort("Invalid home-relative takeoff altitude");
            return;
        }
        flight_home_ = home_;
        home_locked_ = true;
        command_sent_ = true;
        unsigned generation = command_generation_;
        takeoff_client_->async_send_request(request,
            [this, generation](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (generation != command_generation_) {return;}
                command_complete_ = true;
                try {command_accepted_ = future.get()->success;}
                catch (const std::exception &) {command_accepted_ = false;}
            });
    }

    std::string flight_navigation_problem() const
    {
        auto problem = readiness_problem();
        if (!problem.empty()) {return problem;}
        double yaw_error = wrap_angle(local_yaw() - heading_to_yaw(heading_) - yaw_offset_);
        if (std::abs(yaw_error) > yaw_tolerance_) {return "Compass/local yaw alignment disagreement";}
        auto gps_enu = reference_->to_enu(gps_position());
        auto pose_enu = reference_->from_local(local_position());
        if (std::hypot(gps_enu.x - pose_enu.x, gps_enu.y - pose_enu.y) > max_reference_error_) {
            return "GPS/local position reference disagreement";
        }
        auto position = local_position();
        if (std::hypot(position.x - launch_local_.x, position.y - launch_local_.y) > max_distance_ ||
            position.z - launch_local_.z > max_altitude_) {
            return "Measured flight envelope exceeded";
        }
        if (home_locked_) {
            GeodeticPosition current{home_.geo.latitude, home_.geo.longitude, home_.geo.altitude};
            GeodeticPosition captured{flight_home_.geo.latitude, flight_home_.geo.longitude, flight_home_.geo.altitude};
            if (!is_valid(current) || !is_finite({home_.position.x, home_.position.y, home_.position.z})) {
                return "Invalid FCU home";
            }
            auto current_home = reference_->to_enu(current);
            auto captured_home = reference_->to_enu(captured);
            if (std::hypot(current_home.x - captured_home.x, current_home.y - captured_home.y) > 0.5 ||
                std::abs(current_home.z - captured_home.z) > 0.5 ||
                std::hypot(home_.position.x - flight_home_.position.x,
                    home_.position.y - flight_home_.position.y) > 0.5 ||
                std::abs(home_.position.z - flight_home_.position.z) > 0.5) {
                return "FCU home changed after takeoff reference was fixed";
            }
            double local_height = position.z - launch_local_.z;
            double relative_height = relative_altitude_ - launch_relative_altitude_;
            if (std::abs(local_height - relative_height) > altitude_disagreement_) {
                return "Local and relative altitude disagree";
            }
        }
        return "";
    }

    bool settled_at(Point3 destination)
    {
        auto position = local_position();
        const auto & velocity = velocity_.twist.linear;
        double speed = std::hypot(std::hypot(velocity.x, velocity.y), velocity.z);
        bool within_tolerance = std::hypot(position.x - destination.x, position.y - destination.y) <= position_tolerance_ &&
            std::abs(position.z - destination.z) <= altitude_tolerance_ &&
            speed < velocity_tolerance_ && std::abs(wrap_angle(local_yaw() - target_yaw_)) <= yaw_tolerance_;
        if (!within_tolerance) {settled_since_ = TimePoint{}; return false;}
        if (settled_since_ == TimePoint{}) {settled_since_ = SteadyClock::now();}
        return std::chrono::duration<double>(SteadyClock::now() - settled_since_).count() >= settle_duration_;
    }

    void update_flight()
    {
        if (terminal()) {return;}
        if (flight_state_ == OutdoorState::WAIT_READY) {
            update_readiness();
            if (reported_state_ != "REFERENCE_READY") {return;}
            auto problem = flight_navigation_problem();
            if (!problem.empty() || state_.armed) {
                change_phase(OutdoorState::FAILED, problem.empty() ? "Vehicle unexpectedly armed" : problem);
                return;
            }
            target_ = {launch_local_.x, launch_local_.y, launch_local_.z + altitude_};
            target_yaw_ = local_yaw();
            change_phase(OutdoorState::SET_GUIDED, "Takeoff/return check started");
        }
        if (!fresh(state_time_) || !state_.connected) {
            change_phase(OutdoorState::RELEASE_CONTROL, "FCU connection lost; defer to onboard failsafes");
            return;
        }
        if (flight_state_ == OutdoorState::SET_GUIDED && state_.armed) {
            change_phase(OutdoorState::RELEASE_CONTROL, "Vehicle unexpectedly armed before mission arming");
            return;
        }
        bool landing = flight_state_ == OutdoorState::LAND || flight_state_ == OutdoorState::DISARM;
        if (owns_guided_ && !landing && state_.mode != "GUIDED") {
            change_phase(OutdoorState::RELEASE_CONTROL, "Pilot/autopilot changed mode");
            return;
        }
        if (landing && state_.mode == "LAND") {land_mode_seen_ = true;}
        if (landing && (state_.mode != "GUIDED" && state_.mode != "LAND")) {
            change_phase(OutdoorState::RELEASE_CONTROL, "Pilot/autopilot changed mode during landing");
            return;
        }
        if (landing && land_mode_seen_ && state_.mode != "LAND") {
            change_phase(OutdoorState::RELEASE_CONTROL, "Pilot left LAND mode; releasing control");
            return;
        }
        if (!landing) {
            auto problem = flight_navigation_problem();
            if (!problem.empty()) {begin_abort(problem); return;}
            auto current_time = now();
            if (control_clock_started_) {
                double elapsed = (current_time - previous_control_time_).seconds();
                if (elapsed < 0.0 || elapsed > timeout_) {begin_abort("ROS clock discontinuity"); return;}
            }
            previous_control_time_ = current_time;
            control_clock_started_ = true;
        }
        double limit = phase_timeout_;
        if (flight_state_ == OutdoorState::SET_GUIDED || flight_state_ == OutdoorState::ARM ||
            flight_state_ == OutdoorState::DISARM) {limit = command_timeout_;}
        if (flight_state_ == OutdoorState::TAKEOFF) {limit = takeoff_timeout_;}
        if (flight_state_ == OutdoorState::LAND) {limit = landing_timeout_;}
        if (std::chrono::duration<double>(SteadyClock::now() - phase_started_).count() > limit) {
            if (landing) {change_phase(OutdoorState::FAILED, "Landing/disarm timed out; manual recovery required");}
            else {begin_abort("Mission phase timed out");}
            return;
        }
        if (command_complete_ && !command_accepted_) {
            if (landing) {change_phase(OutdoorState::FAILED, "LAND/disarm request rejected");}
            else {begin_abort("MAVROS command rejected");}
            return;
        }
        if (flight_state_ != OutdoorState::ARM && flight_state_ != OutdoorState::SET_GUIDED &&
            !landing && !state_.armed) {
            begin_abort("Unexpected disarm"); return;
        }
        switch (flight_state_) {
            case OutdoorState::SET_GUIDED:
                request_mode("GUIDED");
                if (command_complete_ && command_accepted_ && state_.mode == "GUIDED") {
                    owns_guided_ = true;
                    change_phase(OutdoorState::ARM, "GUIDED confirmed");
                }
                break;
            case OutdoorState::ARM:
                request_arm(true);
                if (command_complete_ && command_accepted_ && state_.armed) {
                    change_phase(OutdoorState::TAKEOFF, "Arming confirmed");
                }
                break;
            case OutdoorState::TAKEOFF:
                request_takeoff();
                if (command_complete_ && command_accepted_ &&
                    extended_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR &&
                    std::abs(pose_.pose.position.z - target_.z) <= altitude_tolerance_ &&
                    std::abs(relative_altitude_ - launch_relative_altitude_ - altitude_) <= altitude_tolerance_) {
                    change_phase(OutdoorState::STABILIZE, "Airborne at target altitude");
                }
                break;
            case OutdoorState::STABILIZE:
                if (settled_at(target_)) {
                    target_ = local_position();
                    previous_target_time_ = now();
                    change_phase(OutdoorState::RETURN_HOME, "Settled; returning to launch before landing");
                }
                break;
            case OutdoorState::RETURN_HOME:
                if (settled_at({launch_local_.x, launch_local_.y, launch_local_.z + altitude_})) {
                    change_phase(OutdoorState::LAND, "Return settled; requesting LAND");
                }
                break;
            case OutdoorState::LAND:
                request_mode("LAND");
                if (command_complete_ && command_accepted_ && state_.mode == "LAND" && fresh(extended_time_) &&
                    extended_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND && !arm_pending_) {
                    if (state_.armed) {change_phase(OutdoorState::DISARM, "Ground confirmed");}
                    else {change_phase(aborting_ ? OutdoorState::ABORTED : OutdoorState::DONE, "Ground and disarm confirmed");}
                }
                break;
            case OutdoorState::DISARM:
                request_arm(false);
                if (command_complete_ && command_accepted_ && !state_.armed) {
                    change_phase(aborting_ ? OutdoorState::ABORTED : OutdoorState::DONE, "Disarm confirmed after landing");
                }
                break;
            default:
                break;
        }
    }

    void publish_setpoint()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        bool taking_off = flight_state_ == OutdoorState::TAKEOFF && command_complete_ && command_accepted_ &&
            extended_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR;
        bool holding = flight_state_ == OutdoorState::STABILIZE;
        bool returning = flight_state_ == OutdoorState::RETURN_HOME;
        if (!taking_off && !holding && !returning) {return;}
        if (!fresh(state_time_) || !state_.connected || state_.mode != "GUIDED") {
            change_phase(OutdoorState::RELEASE_CONTROL, "Lost GUIDED control; setpoints stopped");
            return;
        }
        auto problem = flight_navigation_problem();
        if (!problem.empty() || !state_.armed) {begin_abort(problem.empty() ? "Unexpected disarm" : problem); return;}
        if (returning) {
            auto current_time = now();
            double elapsed = (current_time - previous_target_time_).seconds();
            previous_target_time_ = current_time;
            if (elapsed < 0.0 || elapsed > timeout_) {begin_abort("Setpoint clock discontinuity"); return;}
            Point3 destination{launch_local_.x, launch_local_.y, launch_local_.z + altitude_};
            double east = destination.x - target_.x;
            double north = destination.y - target_.y;
            double distance = std::hypot(east, north);
            if (distance > 1e-6) {
                target_yaw_ = unwrap_angle(target_yaw_, std::atan2(north, east));
                // Align while holding position before advancing along the return segment.
                if (std::abs(wrap_angle(local_yaw() - target_yaw_)) <= yaw_tolerance_) {
                    double step = std::min(return_speed_ * elapsed, distance);
                    target_.x += step * east / distance;
                    target_.y += step * north / distance;
                }
            }
            target_.z = destination.z;
        }
        if (!taking_off && std::hypot(std::hypot(target_.x - pose_.pose.position.x,
            target_.y - pose_.pose.position.y), target_.z - pose_.pose.position.z) > max_tracking_error_) {
            begin_abort("Position target tracking error exceeded"); return;
        }
        mavros_msgs::msg::PositionTarget message;
        message.header.stamp = now();
        message.header.frame_id = local_frame_;
        message.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        message.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
            mavros_msgs::msg::PositionTarget::IGNORE_VY | mavros_msgs::msg::PositionTarget::IGNORE_VZ |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX | mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ | mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
        message.position.x = target_.x;
        message.position.y = target_.y;
        message.position.z = target_.z;
        message.yaw = static_cast<float>(target_yaw_);
        setpoint_pub_->publish(message);
    }

    static void add_value(diagnostic_msgs::msg::DiagnosticStatus & status,
        const std::string & key, const std::string & value)
    {
        diagnostic_msgs::msg::KeyValue entry;
        entry.key = key;
        entry.value = value;
        status.values.push_back(entry);
    }

    void report()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "outdoor_mission/telemetry";
        status.hardware_id = "mavros";
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        if (reported_state_ == "REFERENCE_READY") {status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;}
        status.message = reported_state_ + ": " + reason_;
        add_value(status, "state", reported_state_);
        add_value(status, "flight_enabled", enable_flight_ ? "true" : "false");
        add_value(status, "abort_reason", abort_reason_);
        add_value(status, "satellites", std::to_string(satellites_));
        add_value(status, "fix_status", std::to_string(fix_.status.status));
        if (fix_time_ != TimePoint{}) {
            add_value(status, "latitude", std::to_string(fix_.latitude));
            add_value(status, "longitude", std::to_string(fix_.longitude));
            add_value(status, "altitude_ellipsoid", std::to_string(fix_.altitude));
            add_value(status, "fix_age", std::to_string(
                std::chrono::duration<double>(SteadyClock::now() - fix_time_).count()));
        }
        if (fresh(heading_time_) && std::isfinite(heading_) && heading_ >= 0.0 && heading_ < 360.0) {
            double compass_yaw = heading_to_yaw(heading_);
            add_value(status, "heading_degrees", std::to_string(heading_));
            add_value(status, "compass_yaw_enu_degrees", std::to_string(compass_yaw * 180.0 / pi));
            const auto & q = pose_.pose.orientation;
            double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
            if (fresh(pose_time_) && std::isfinite(norm) && std::abs(norm - 1.0) <= 0.01) {
                double local_yaw = std::atan2(2.0*(q.w*q.z + q.x*q.y), norm - 2.0*(q.y*q.y + q.z*q.z));
                add_value(status, "local_yaw_degrees", std::to_string(local_yaw * 180.0 / pi));
                add_value(status, "yaw_alignment_error_degrees", std::to_string(
                    wrap_angle(local_yaw - compass_yaw - yaw_offset_) * 180.0 / pi));
            }
        }
        if (reference_) {
            add_value(status, "origin_source", origin_source_);
            add_value(status, "origin_latitude", std::to_string(origin_.latitude));
            add_value(status, "origin_longitude", std::to_string(origin_.longitude));
            add_value(status, "horizontal_bound_with_margin", std::to_string(envelope_bound_ + envelope_margin_));
            add_value(status, "flight_local_z", std::to_string(launch_local_.z + altitude_));
            if (fresh(pose_time_) && is_finite(local_position())) {
                auto enu = reference_->from_local(local_position());
                add_value(status, "local_x", std::to_string(pose_.pose.position.x));
                add_value(status, "local_y", std::to_string(pose_.pose.position.y));
                add_value(status, "local_z", std::to_string(pose_.pose.position.z));
                add_value(status, "mission_east", std::to_string(enu.x));
                add_value(status, "mission_north", std::to_string(enu.y));
                add_value(status, "mission_up", std::to_string(enu.z));
                auto geodetic = reference_->to_geodetic(enu);
                add_value(status, "local_derived_latitude", std::to_string(geodetic.latitude));
                add_value(status, "local_derived_longitude", std::to_string(geodetic.longitude));
                add_value(status, "local_derived_altitude_ellipsoid", std::to_string(geodetic.altitude_ellipsoid));
                add_value(status, "height_above_launch_local", std::to_string(pose_.pose.position.z - launch_local_.z));
            }
            if (fresh(fix_time_) && fix_.status.status >= 0 && is_valid(gps_position())) {
                auto enu = reference_->to_enu(gps_position());
                add_value(status, "gps_east", std::to_string(enu.x));
                add_value(status, "gps_north", std::to_string(enu.y));
                add_value(status, "gps_up", std::to_string(enu.z));
            }
            if (fresh(altitude_time_) && std::isfinite(relative_altitude_)) {
                add_value(status, "height_above_launch_relative", std::to_string(relative_altitude_ - launch_relative_altitude_));
            }
        }
        diagnostic_msgs::msg::DiagnosticArray message;
        message.header.stamp = now();
        message.status.push_back(status);
        diagnostics_pub_->publish(message);
        RCLCPP_INFO(get_logger(), "%s", status.message.c_str());
    }

    std::mutex mutex_;
    rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedPtr interval_client_;
    bool interval_pending_ = false, interval_configured_ = false;
    int64_t interval_request_id_ = 0;
    unsigned interval_generation_ = 0;
    TimePoint interval_attempt_time_;
    OutdoorState flight_state_ = OutdoorState::WAIT_READY;
    bool enable_flight_ = false, owns_guided_ = false, aborting_ = false;
    bool command_sent_ = false, command_complete_ = false, command_accepted_ = false;
    bool arm_pending_ = false, home_locked_ = false, land_mode_seen_ = false;
    unsigned command_generation_ = 0;
    TimePoint phase_started_, settled_since_;
    rclcpp::Time previous_control_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time previous_target_time_{0, 0, RCL_ROS_TIME};
    bool control_clock_started_ = false;
    Point3 target_{0.0, 0.0, 0.0};
    double target_yaw_ = 0.0;
    double position_tolerance_, altitude_tolerance_, velocity_tolerance_, yaw_tolerance_;
    double settle_duration_, return_speed_, command_timeout_, takeoff_timeout_, phase_timeout_, landing_timeout_;
    double max_tracking_error_, max_reference_error_, altitude_disagreement_;
    mavros_msgs::msg::HomePosition flight_home_;
    std::string abort_reason_;
    rclcpp::CallbackGroup::SharedPtr setpoint_group_;
    rclcpp::TimerBase::SharedPtr setpoint_timer_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_service_;

    Point3 poi_, launch_local_{0.0, 0.0, 0.0}, launch_enu_{0.0, 0.0, 0.0};
    GeodeticPosition origin_{0.0, 0.0, 0.0};
    double altitude_, side_, amplitude_, omega_, max_speed_, max_distance_, max_altitude_;
    double envelope_margin_, yaw_offset_, timeout_, readiness_duration_, sample_window_;
    int min_satellites_;
    double envelope_bound_ = 0.0, launch_relative_altitude_ = 0.0;
    bool envelope_ok_ = false, home_received_ = false;
    std::unique_ptr<OutdoorReference> reference_;
    std::string origin_source_, local_frame_, reported_state_, reason_;
    TimePoint ready_since_, state_time_, extended_time_, fix_time_, satellites_time_;
    TimePoint pose_time_, velocity_time_, heading_time_, altitude_time_;
    mavros_msgs::msg::State state_;
    mavros_msgs::msg::ExtendedState extended_;
    mavros_msgs::msg::HomePosition home_;
    sensor_msgs::msg::NavSatFix fix_;
    geometry_msgs::msg::PoseStamped pose_;
    geometry_msgs::msg::TwistStamped velocity_;
    unsigned satellites_ = 255;
    double heading_ = 0.0, relative_altitude_ = 0.0;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_sub_;
    rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_, relative_altitude_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr satellites_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr readiness_timer_, reporting_timer_;
};
}  // namespace mission_executor

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        auto node = std::make_shared<mission_executor::OutdoorMissionNode>();
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception & error) {
        RCLCPP_FATAL(rclcpp::get_logger("outdoor_mission_node"), "%s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
