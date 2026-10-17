#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"

#include "mission_executor/mission_parser.hpp"
#include "mission_executor/mission_types.hpp"
#include "mission_executor/mission_state.hpp"
#include "mission_executor/position_controller.hpp"

using namespace std::chrono_literals;

namespace mission_executor
{

class MissionExecutorNode : public rclcpp::Node
{
public:
    MissionExecutorNode()
        : Node("mission_executor_node"),
          current_state_(MissionState::IDLE),
          position_controller_(0.15, 0.50, 0.15, 1.0)
    {
        this->declare_parameter<std::string>("mission_file", "");
        this->declare_parameter<bool>("enable_arming", false);
        this->declare_parameter<bool>("enable_mavros_setpoints", false);
        this->declare_parameter<bool>("enable_navigation_diagnostics", false);
        this->declare_parameter<bool>("debug_waypoint_setpoints", false);
        this->declare_parameter<bool>("enable_navigation_test", false);
        this->declare_parameter<bool>("enable_navigation_flight", false);
        this->declare_parameter<bool>("enable_hard_soft_flight_test", false);

        enable_arming_ = this->get_parameter("enable_arming").as_bool();
        enable_mavros_setpoints_ =
            this->get_parameter("enable_mavros_setpoints").as_bool();
        navigation_diagnostics_enabled_ =
            this->get_parameter("enable_navigation_diagnostics").as_bool();
        debug_waypoint_setpoints_ =
            this->get_parameter("debug_waypoint_setpoints").as_bool();
        enable_navigation_test_ =
            this->get_parameter("enable_navigation_test").as_bool();
        enable_navigation_flight_ =
            this->get_parameter("enable_navigation_flight").as_bool();
        enable_hard_soft_flight_test_ =
            this->get_parameter("enable_hard_soft_flight_test").as_bool();
        if (enable_hard_soft_flight_test_ && !enable_navigation_flight_) {
            throw std::runtime_error(
                "enable_hard_soft_flight_test requires enable_navigation_flight=true.");
        }

        if (enable_navigation_test_ && enable_navigation_flight_) {
            throw std::runtime_error(
                "Choose only one: navigation test OR navigation flight.");
        }
        if ((enable_navigation_test_ || enable_navigation_flight_) &&
            (enable_arming_ || enable_mavros_setpoints_ ||
             !navigation_diagnostics_enabled_ || !debug_waypoint_setpoints_)) {
            throw std::runtime_error(
                "A1.2.2 navigation requires enable_arming=false, "
                "enable_mavros_setpoints=false, "
                "enable_navigation_diagnostics=true and "
                "debug_waypoint_setpoints=true.");
        }

        if (enable_arming_ && enable_mavros_setpoints_) {
            throw std::runtime_error(
                "enable_arming and enable_mavros_setpoints cannot both be "
                "true in this test version."
            );
        }

        RCLCPP_INFO(this->get_logger(), "Automatic arming: %s",
                    enable_arming_ ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(this->get_logger(), "Navigation diagnostics: %s",
                    navigation_diagnostics_enabled_ ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(this->get_logger(), "Debug waypoint setpoints: %s",
                    debug_waypoint_setpoints_ ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(this->get_logger(),
            "NAVIGATE modes: test=%s flight=%s hard_soft=%s. "
            "Flight is NOT safety-validated.",
            enable_navigation_test_ ? "ON" : "OFF",
            enable_navigation_flight_ ? "ON" : "OFF",
            enable_hard_soft_flight_test_ ? "ON" : "OFF");

        const std::string mission_file =
            this->get_parameter("mission_file").as_string();
        if (mission_file.empty()) {
            throw std::runtime_error("Parameter 'mission_file' is empty.");
        }
        mission_ = MissionParser::load_from_csv(mission_file);
        if (mission_.empty()) {
            throw std::runtime_error("Mission does not contain any waypoints.");
        }
        RCLCPP_INFO(this->get_logger(), "Mission loaded: %zu waypoints",
                    mission_.size());
        if (enable_hard_soft_flight_test_) {
            validate_hard_soft_test_mission();
            RCLCPP_WARN(this->get_logger(),
                "HARD/SOFT FLIGHT TEST: direct position targets (NO planner). "
                "Four nearby WPs, last WP HOLD. "
                "Only fly after VISUAL confirmation that the full corridor "
                "and surrounding safety margin are free of racks/obstacles!");
        }
        RCLCPP_INFO(this->get_logger(),
                    "PositionController: HARD=0.15m, SOFT=0.50m, "
                    "HARD speed<=0.15m/s, stable>=1.0s");

        state_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        setpoint_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        pose_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions state_options;
        state_options.callback_group = state_group_;
        mavros_state_sub_ =
            this->create_subscription<mavros_msgs::msg::State>(
                "/mavros/state", 10,
                std::bind(&MissionExecutorNode::mavros_state_callback, this,
                          std::placeholders::_1),
                state_options);

        rclcpp::SubscriptionOptions pose_options;
        pose_options.callback_group = pose_group_;
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
            std::bind(&MissionExecutorNode::pose_callback, this,
                      std::placeholders::_1),
            pose_options);

        velocity_sub_ =
            this->create_subscription<geometry_msgs::msg::TwistStamped>(
                "/mavros/local_position/velocity_local",
                rclcpp::SensorDataQoS(),
                std::bind(&MissionExecutorNode::velocity_callback, this,
                          std::placeholders::_1),
                pose_options);

        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
            "/mavros/set_mode", rclcpp::ServicesQoS(), state_group_);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
            "/mavros/cmd/arming", rclcpp::ServicesQoS(), state_group_);

        debug_setpoint_pub_ =
            this->create_publisher<mavros_msgs::msg::PositionTarget>(
                "/mission_executor/debug_setpoint", 10);
        mavros_setpoint_pub_ =
            this->create_publisher<mavros_msgs::msg::PositionTarget>(
                "/mavros/setpoint_raw/local", 10);

        state_timer_ = this->create_wall_timer(
            100ms, std::bind(&MissionExecutorNode::state_machine_callback, this),
            state_group_);
        setpoint_timer_ = this->create_wall_timer(
            50ms, std::bind(&MissionExecutorNode::publish_setpoint, this),
            setpoint_group_);

        RCLCPP_INFO(this->get_logger(),
                    "Callback groups initialized: state, setpoint, telemetry.");
        RCLCPP_INFO(this->get_logger(),
                    "Debug setpoint publisher initialized at 20 Hz.");
        RCLCPP_INFO(this->get_logger(), "Real MAVROS setpoints: %s",
                    enable_navigation_flight_ ?
                        (enable_hard_soft_flight_test_ ?
                         "ENABLED (SITL, 4-WP HARD/SOFT TEST ONLY)" :
                         "ENABLED (SITL, WP1 only)") :
                    (enable_mavros_setpoints_ ? "ENABLED (disarmed only)"
                                              : "DISABLED"));
    }

private:
    Mission mission_;
    MissionState current_state_;
    PositionController position_controller_;
    std::atomic<std::size_t> current_waypoint_index_{0};

    bool enable_arming_ = false;
    bool enable_mavros_setpoints_ = false;
    bool navigation_diagnostics_enabled_ = false;
    bool debug_waypoint_setpoints_ = false;
    bool enable_navigation_test_ = false;
    bool enable_navigation_flight_ = false;
    bool enable_hard_soft_flight_test_ = false;
    std::atomic<bool> flight_navigation_active_{false};

    std::mutex mavros_mutex_;
    bool mavros_connected_ = false;
    bool mavros_armed_ = false;
    bool mavros_state_received_ = false;
    bool arming_disabled_logged_ = false;
    std::string mavros_mode_;
    std::chrono::steady_clock::time_point last_mavros_state_time_{};

    static constexpr auto MAVROS_TIMEOUT = 10s;
    static constexpr auto MODE_TIMEOUT = 10s;
    static constexpr auto ARM_TIMEOUT = 12s;
    static constexpr auto POSE_TIMEOUT = 2500ms;
    static constexpr auto VELOCITY_TIMEOUT = 2500ms;

    std::mutex pose_mutex_;
    geometry_msgs::msg::PoseStamped current_pose_;
    bool pose_received_ = false;
    std::chrono::steady_clock::time_point last_pose_time_{};


    geometry_msgs::msg::TwistStamped current_velocity_;
    bool velocity_received_ = false;
    std::chrono::steady_clock::time_point last_velocity_time_{};

    bool hold_initialized_ = false;
    geometry_msgs::msg::Point hold_position_;
    float hold_yaw_ = 0.0f;

    rclcpp::CallbackGroup::SharedPtr state_group_;
    rclcpp::CallbackGroup::SharedPtr setpoint_group_;
    rclcpp::CallbackGroup::SharedPtr pose_group_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr
        mavros_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
        velocity_sub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr
        debug_setpoint_pub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr
        mavros_setpoint_pub_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr setpoint_timer_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr
        set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr
        arming_client_;

    bool mode_request_sent_ = false;
    bool mode_response_received_ = false;
    bool mode_request_accepted_ = false;
    std::chrono::steady_clock::time_point mode_request_time_{};
    bool arm_request_sent_ = false;
    bool arm_response_received_ = false;
    bool arm_request_accepted_ = false;
    std::chrono::steady_clock::time_point arm_request_time_{};

    void validate_hard_soft_test_mission()
    {
        if (mission_.size() != 4) {
            throw std::runtime_error(
                "HARD/SOFT SITL flight accepts EXACTLY 4 waypoints.");
        }
        const Precision expected[4] = {
            Precision::SOFT, Precision::SOFT,
            Precision::HARD, Precision::SOFT};
        const auto first = position_controller_.gazebo_to_enu(mission_[0]);
        for (std::size_t i = 0; i < mission_.size(); ++i) {
            const auto &w = mission_[i];
            const auto target = position_controller_.gazebo_to_enu(w);
            if (w.precision != expected[i] ||
                !std::isfinite(target.x) || !std::isfinite(target.y) ||
                !std::isfinite(target.z) ||
                target.z < 1.8 || target.z > 2.2 ||
                std::fabs(target.z - first.z) > 0.05 ||
                std::hypot(target.x - first.x, target.y - first.y) > 2.05) {
                throw std::runtime_error(
                    "Unsafe HARD/SOFT test mission: expected SOFT/SOFT/HARD/SOFT, "
                    "height near 2 m, and max 2.05 m horizontal from WP1.");
            }
            if (i > 0) {
                const auto prev =
                    position_controller_.gazebo_to_enu(mission_[i - 1]);
                if (std::hypot(target.x - prev.x, target.y - prev.y) > 1.15) {
                    throw std::runtime_error(
                        "Unsafe HARD/SOFT test mission: segment > 1.15 m.");
                }
            }
        }
    }

    void mavros_state_callback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mavros_mutex_);
        mavros_state_received_ = true;
        mavros_connected_ = msg->connected;
        mavros_armed_ = msg->armed;
        mavros_mode_ = msg->mode;
        last_mavros_state_time_ = std::chrono::steady_clock::now();
    }

    void pose_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_pose_ = *msg;
            pose_received_ = true;
            last_pose_time_ = std::chrono::steady_clock::now();
        }
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Current position: x=%.2f, y=%.2f, z=%.2f",
            msg->pose.position.x, msg->pose.position.y,
            msg->pose.position.z);
    }

    void velocity_callback(
        const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_velocity_ = *msg;
            velocity_received_ = true;
            last_velocity_time_ = std::chrono::steady_clock::now();
        }
        const auto &v = msg->twist.linear;
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Current velocity: vx=%.3f, vy=%.3f, vz=%.3f m/s",
            v.x, v.y, v.z);
    }

    bool process_current_waypoint()
    {
        if (!navigation_diagnostics_enabled_) {
            return false;
        }
        const std::size_t index = current_waypoint_index_.load();
        if (index >= mission_.size()) {
            position_controller_.reset_arrival();
            return false;
        }

        geometry_msgs::msg::PoseStamped pose_snapshot;
        geometry_msgs::msg::TwistStamped velocity_snapshot;
        bool pose_ok = false;
        bool velocity_ok = false;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            const auto now = std::chrono::steady_clock::now();
            pose_ok = pose_received_ &&
                now - last_pose_time_ <= POSE_TIMEOUT;
            velocity_ok = velocity_received_ &&
                now - last_velocity_time_ <= VELOCITY_TIMEOUT;
            if (pose_ok) {
                pose_snapshot = current_pose_;
            }
            if (velocity_ok) {
                velocity_snapshot = current_velocity_;
            }
        }

          if (!pose_ok || !velocity_ok) {
            position_controller_.reset_arrival();
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Navigation diagnostics waiting for fresh %s%s%s",
                !pose_ok ? "pose" : "",
                !pose_ok && !velocity_ok ? " and " : "",
                !velocity_ok ? "velocity" : "");
            return false;
        }

        const MissionWaypoint &waypoint = mission_[index];
        const auto target = position_controller_.gazebo_to_enu(waypoint);
        const auto &position = pose_snapshot.pose.position;
        const auto &velocity = velocity_snapshot.twist.linear;
        const auto arrival = position_controller_.update_arrival(
            position, velocity, waypoint, std::chrono::steady_clock::now());

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Waypoint %zu/%zu | CSV=(%.2f, %.2f, %.2f) "
            "ENU=(%.2f, %.2f, %.2f) | Distance=%.3f m | "
            "Speed=%.3f m/s | Stable=%.2f s | Precision=%s | Reached=%s",
            index + 1, mission_.size(),
            waypoint.x, waypoint.y, waypoint.z,
            target.x, target.y, target.z,
            arrival.distance_m, arrival.speed_mps, arrival.stable_time_s,
            waypoint.precision == Precision::HARD ? "HARD" : "SOFT",
            arrival.reached ? "YES" : "NO");

        if (arrival.reached) {
            RCLCPP_INFO(this->get_logger(),
                "ARRIVAL WP%zu: precision=%s distance=%.3f m "
                "speed=%.3f m/s stable=%.2f s",
                index + 1,
                waypoint.precision == Precision::HARD ? "HARD" : "SOFT",
                arrival.distance_m, arrival.speed_mps, arrival.stable_time_s);
        }
        return arrival.reached;
    }

    void change_state(MissionState new_state)
    {
        if (current_state_ == new_state) {
            return;
        }
        RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s",
                    state_to_string(current_state_),
                    state_to_string(new_state));
        current_state_ = new_state;
        if (new_state == MissionState::ERROR) {
            flight_navigation_active_.store(false);
        }
    }

    mavros_msgs::msg::PositionTarget create_position_target()
    {
        mavros_msgs::msg::PositionTarget msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.coordinate_frame =
            mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        msg.type_mask =
            mavros_msgs::msg::PositionTarget::IGNORE_VX |
            mavros_msgs::msg::PositionTarget::IGNORE_VY |
            mavros_msgs::msg::PositionTarget::IGNORE_VZ |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
        return msg;
    }

    void publish_setpoint()
    {
        auto debug_msg = create_position_target();
        const auto index = current_waypoint_index_.load();
          if (index >= mission_.size()) {
            return;
        }
        if (debug_waypoint_setpoints_ && index < mission_.size()) {
            const auto &waypoint = mission_[index];
            debug_msg.position = position_controller_.gazebo_to_enu(waypoint);
            debug_msg.type_mask |=
                mavros_msgs::msg::PositionTarget::IGNORE_YAW;
        } else if (hold_initialized_) {
            debug_msg.position = hold_position_;
            debug_msg.yaw = hold_yaw_;
        }
        debug_setpoint_pub_->publish(debug_msg);

        if (!enable_mavros_setpoints_ && !enable_navigation_flight_) {
            return;
        }
        bool connected = false;
        bool armed = false;
        bool state_fresh = false;
        std::string mode;
        {
            std::lock_guard<std::mutex> lock(mavros_mutex_);
            connected = mavros_state_received_ && mavros_connected_;
            armed = mavros_armed_;
            mode = mavros_mode_;
            state_fresh = mavros_state_received_ &&
                (std::chrono::steady_clock::now() - last_mavros_state_time_
                 <= MAVROS_TIMEOUT);
        }
        if (enable_navigation_flight_) {
            if (!flight_navigation_active_.load() ||
                !connected || !state_fresh || !armed || mode != "GUIDED") {
                return;
            }
            geometry_msgs::msg::PoseStamped pose_snapshot;
            bool telemetry_fresh = false;
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                const auto now = std::chrono::steady_clock::now();
                telemetry_fresh = pose_received_ && velocity_received_ &&
                    now - last_pose_time_ <= POSE_TIMEOUT &&
                    now - last_velocity_time_ <= VELOCITY_TIMEOUT;
                if (telemetry_fresh) {
                    pose_snapshot = current_pose_;
                }
            }
            if (!telemetry_fresh) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(),
                    2000, "Flight navigation: telemetry stale! "
                    "No new targets; autopilot may retain previous target.");
                return;
            }
            const auto &p = pose_snapshot.pose.position;
            if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
                !std::isfinite(p.z) || p.z < 0.5) {
                return;
            }
            const auto goal_idx = current_waypoint_index_.load();
            if (goal_idx >= mission_.size()) {
                return;
            }
            auto target = create_position_target();
            target.position =
                position_controller_.gazebo_to_enu(mission_[goal_idx]);
            target.type_mask |=
                mavros_msgs::msg::PositionTarget::IGNORE_YAW;
            mavros_setpoint_pub_->publish(target);
            return;
        }

        if (!connected || !state_fresh || armed) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Real setpoint paused: disconnected, stale state, or armed.");
            return;
        }

        geometry_msgs::msg::PoseStamped pose_snapshot;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (!pose_received_ ||
                std::chrono::steady_clock::now() - last_pose_time_ > POSE_TIMEOUT)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Real setpoint paused: missing or stale pose.");
                return;
            }
            pose_snapshot = current_pose_;
        }

        if (!hold_initialized_) {
            const auto &p = pose_snapshot.pose.position;
            const auto &q = pose_snapshot.pose.orientation;
            if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
                !std::isfinite(p.z) || !std::isfinite(q.x) ||
                !std::isfinite(q.y) || !std::isfinite(q.z) ||
                !std::isfinite(q.w))
            {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Cannot initialize hold: invalid pose values.");
                return;
            }
            const double q_norm_sq = q.x * q.x + q.y * q.y +
                                     q.z * q.z + q.w * q.w;
            if (q_norm_sq < 1e-12) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Cannot initialize hold: invalid orientation.");
                return;
            }
            hold_position_ = p;
            hold_yaw_ = static_cast<float>(std::atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
            hold_initialized_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "Position hold initialized: x=%.2f y=%.2f z=%.2f yaw=%.2f",
                        hold_position_.x, hold_position_.y, hold_position_.z,
                        static_cast<double>(hold_yaw_));
        }

        auto real_msg = create_position_target();
        real_msg.position = hold_position_;
        real_msg.yaw = hold_yaw_;
        mavros_setpoint_pub_->publish(real_msg);
    }

    void request_guided_mode()
    {
        if (!set_mode_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting for /mavros/set_mode service...");
            return;
        }
        auto request =
            std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->base_mode = 0;
        request->custom_mode = "GUIDED";
        mode_request_sent_ = true;
        mode_response_received_ = false;
        mode_request_accepted_ = false;
        mode_request_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Requesting GUIDED mode...");

        set_mode_client_->async_send_request(
            request,
            [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
            {
                try {
                    const auto response = future.get();
                    mode_request_accepted_ = response->mode_sent;
                    mode_response_received_ = true;
                    if (response->mode_sent) {
                        RCLCPP_INFO(this->get_logger(),
                                    "GUIDED mode request sent successfully.");
                    } else {
                        RCLCPP_ERROR(this->get_logger(),
                                     "MAVROS rejected GUIDED mode request.");
                    }
                } catch (const std::exception &e) {
                    mode_request_accepted_ = false;
                    mode_response_received_ = true;
                    RCLCPP_ERROR(this->get_logger(),
                                 "SET_MODE service error: %s", e.what());
                }
            });
    }

    void request_arm()
    {
        if (!arming_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting for /mavros/cmd/arming service...");
            return;
        }
        auto request =
            std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;
        arm_request_sent_ = true;
        arm_response_received_ = false;
        arm_request_accepted_ = false;
        arm_request_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Requesting ARM...");
        arming_client_->async_send_request(
            request,
            [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
            {
                try {
                    const auto response = future.get();
                    arm_request_accepted_ = response->success;
                    arm_response_received_ = true;
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(),
                                    "ARM request accepted. Result: %u",
                                    static_cast<unsigned>(response->result));
                    } else {
                        RCLCPP_ERROR(this->get_logger(),
                                     "ARM request rejected. Result: %u",
                                     static_cast<unsigned>(response->result));
                    }
                } catch (const std::exception &e) {
                    arm_request_accepted_ = false;
                    arm_response_received_ = true;
                    RCLCPP_ERROR(this->get_logger(),
                                 "ARM service error: %s", e.what());
                }
            });
    }

    void state_machine_callback()
    {
        bool connected = false;
        bool armed = false;
        bool state_received = false;
        std::string mode;
        std::chrono::steady_clock::time_point last_state_time{};
        {
            std::lock_guard<std::mutex> lock(mavros_mutex_);
            connected = mavros_connected_;
            armed = mavros_armed_;
            state_received = mavros_state_received_;
            mode = mavros_mode_;
            if (state_received) {
                last_state_time = last_mavros_state_time_;
            }
        }
        const auto now = std::chrono::steady_clock::now();
        if (!enable_navigation_test_ && state_received &&
            current_state_ != MissionState::IDLE &&
            current_state_ != MissionState::WAIT_CONNECTION &&
            current_state_ != MissionState::ERROR &&
            current_state_ != MissionState::DONE)
        {
            if (!connected) {
                RCLCPP_ERROR(this->get_logger(), "MAVROS reports disconnected!");
                change_state(MissionState::ERROR);
                return;
            }
            if (now - last_state_time > MAVROS_TIMEOUT) {
                RCLCPP_ERROR(this->get_logger(), "MAVROS state timeout!");
                change_state(MissionState::ERROR);
                return;
            }
        }

        switch (current_state_) {
            case MissionState::IDLE:
            {
                if (enable_navigation_test_) {

                    RCLCPP_INFO(this->get_logger(),
                        "A1.2.2 diagnostics: NAVIGATE with live pose/velocity, "
                        "no arming, no MAVROS position commands.");
                    position_controller_.reset_arrival();
                    change_state(MissionState::NAVIGATE);
                } else {
                    RCLCPP_INFO(this->get_logger(),
                                "Mission ready. Waiting for MAVROS.");
                    change_state(MissionState::WAIT_CONNECTION);
                }
                break;
            }
            case MissionState::WAIT_CONNECTION:
            {
                const bool state_fresh = state_received &&
                    (now - last_state_time <= MAVROS_TIMEOUT);
                if (state_fresh && connected) {
                    RCLCPP_INFO(this->get_logger(), "MAVROS connected.");
                    change_state(MissionState::SET_MODE);
                }
                break;
            }
            case MissionState::SET_MODE:
            {
                if (mode == "GUIDED") {
                    RCLCPP_INFO(this->get_logger(), "GUIDED mode confirmed.");
                    change_state(MissionState::ARM);
                    break;
                }
                if (!mode_request_sent_) {
                    request_guided_mode();
                    break;
                }
                if (mode_response_received_ && !mode_request_accepted_) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Failed to request GUIDED mode.");
                    change_state(MissionState::ERROR);
                    break;
                }
                if (now - mode_request_time_ > MODE_TIMEOUT) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Timeout waiting for GUIDED mode confirmation.");
                    change_state(MissionState::ERROR);
                }
                break;
            }
            case MissionState::ARM:
            {

                if (navigation_diagnostics_enabled_ && !enable_navigation_flight_) {
                    process_current_waypoint();
                }
                if (enable_navigation_flight_ && armed && mode == "GUIDED") {

                    bool initial_flight_pose_ok = false;
                    {
                        std::lock_guard<std::mutex> lock(pose_mutex_);
                        const auto t = std::chrono::steady_clock::now();
                        const auto &p = current_pose_.pose.position;
                        const auto &v = current_velocity_.twist.linear;
                        const auto goal =
                            position_controller_.gazebo_to_enu(mission_.front());
                        const double horizontal_distance =
                            std::hypot(p.x - goal.x, p.y - goal.y);
                        const double speed = std::sqrt(
                            v.x * v.x + v.y * v.y + v.z * v.z);
                        initial_flight_pose_ok =
                            pose_received_ && velocity_received_ &&
                            t - last_pose_time_ <= POSE_TIMEOUT &&
                            t - last_velocity_time_ <= VELOCITY_TIMEOUT &&
                            std::isfinite(p.x) && std::isfinite(p.y) &&
                            std::isfinite(p.z) && std::isfinite(speed) &&
                            p.z >= 1.5 && p.z <= 2.5 &&
                            horizontal_distance <=
                                (enable_hard_soft_flight_test_ ? 0.25 : 0.75) &&
                            (!enable_hard_soft_flight_test_ ||
                             std::fabs(p.z - goal.z) <= 0.25) &&
                            speed <= 0.25;
                    }
                    if (initial_flight_pose_ok) {
                        if (enable_hard_soft_flight_test_) {
                            RCLCPP_WARN(this->get_logger(),
                                "A1.2.2 SITL: SOFT -> SOFT -> HARD -> SOFT, "
                                "then HOLD final WP. Land manually.");
                        } else {
                            RCLCPP_WARN(this->get_logger(),
                                "A1.2.2 controlled SITL flight: ONLY first "
                                "waypoint, then HOLD. Planner not connected. "
                                "Land manually afterwards.");
                        }
                        position_controller_.reset_arrival();
                        flight_navigation_active_.store(true);
                        change_state(MissionState::NAVIGATE);
                    }
                    break;
                }
                if (!enable_arming_) {
                    if (!arming_disabled_logged_) {
                        RCLCPP_INFO(this->get_logger(),
                                    "Automatic arming disabled. Waiting in ARM state.");
                        arming_disabled_logged_ = true;
                    }
                    break;
                }
                if (mode != "GUIDED") {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Cannot arm: GUIDED mode is not active.");
                    change_state(MissionState::ERROR);
                    break;
                }
                if (armed) {
                    RCLCPP_INFO(this->get_logger(),
                                "Drone armed successfully.");
                    change_state(MissionState::TAKEOFF);
                    break;
                }
                if (!arm_request_sent_) {
                    request_arm();
                    break;
                }
                if (arm_response_received_ && !arm_request_accepted_) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Failed to arm the drone.");
                    change_state(MissionState::ERROR);
                    break;
                }
                if (now - arm_request_time_ > ARM_TIMEOUT) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Timeout waiting for ARM confirmation.");
                    change_state(MissionState::ERROR);
                }
                break;
            }
            case MissionState::TAKEOFF:
            {

                break;
            }
            case MissionState::NAVIGATE:
            {
                if (enable_navigation_flight_ && (!armed || mode != "GUIDED")) {
                    RCLCPP_ERROR(this->get_logger(),
                        "Flight interrupted: disarmed or GUIDED lost. "
                        "Inspect SITL, land manually if needed.");
                    change_state(MissionState::ERROR);
                    break;
                }
                if (enable_navigation_flight_) {
                    bool fresh = false;
                    {
                        std::lock_guard<std::mutex> lock(pose_mutex_);
                        const auto t = std::chrono::steady_clock::now();
                        fresh = pose_received_ && velocity_received_ &&
                            t - last_pose_time_ <= POSE_TIMEOUT &&
                            t - last_velocity_time_ <= VELOCITY_TIMEOUT;
                    }
                    if (!fresh) {
                        RCLCPP_ERROR(this->get_logger(),
                            "Flight telemetry lost. ERROR: no fresh commands "
                            "will be sent, but ArduPilot may keep the last "
                            "position target! Land manually if needed.");
                        change_state(MissionState::ERROR);
                        break;
                    }
                }
                if (!enable_navigation_test_ && !enable_navigation_flight_) {

                    break;
                }
                const auto index = current_waypoint_index_.load();
                if (index >= mission_.size()) {
                    position_controller_.reset_arrival();
                    change_state(MissionState::DONE);
                    break;
                }
                if (process_current_waypoint()) {
                    RCLCPP_INFO(this->get_logger(),
                        "Waypoint %zu/%zu reached [%s]. %s",
                        index + 1, mission_.size(),
                        mission_[index].precision == Precision::HARD
                            ? "HARD settled" : "SOFT pass-through",
                        enable_navigation_flight_ &&
                          !enable_hard_soft_flight_test_ ?
                          "HOLD WP1 (flight limit)." :
                        (index + 1 == mission_.size() ? "HOLD final WP." :
                         "Advancing."));
                    position_controller_.reset_arrival();

                    if (enable_navigation_flight_ &&
                        !enable_hard_soft_flight_test_) {
                        change_state(MissionState::DONE);
                    } else if (index + 1 < mission_.size()) {
                        current_waypoint_index_.store(index + 1);
                    } else {

                        if (enable_navigation_flight_) {
                            RCLCPP_WARN(this->get_logger(),
                                "Last waypoint reached: HOLD last target. "
                                "Landing is NOT implemented here.");
                        } else {
                            current_waypoint_index_.store(mission_.size());
                        }
                        change_state(MissionState::DONE);
                    }
                }
                break;
            }
            case MissionState::TASK:
            {
                break;
            }
            case MissionState::LAND:
            {

                break;
            }
            case MissionState::DONE:
            {
                break;
            }
            case MissionState::ERROR:
            {
                // Bezpecny reakcia na chybu pred skutocnym letom.
                break;
            }
        }
    }
};

}  // namespace mission_executor

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node =
            std::make_shared<mission_executor::MissionExecutorNode>();
        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(), 3);
        executor.add_node(node);
        RCLCPP_INFO(node->get_logger(),
                    "MultiThreadedExecutor started with 3 threads.");
        executor.spin();
        executor.remove_node(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("mission_executor"),
                     "Mission executor failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}