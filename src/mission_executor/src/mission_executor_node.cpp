#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "uav_navigation_msgs/action/compute_path3_d.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"

#include "mission_executor/mission_parser.hpp"
#include "mission_executor/mission_types.hpp"
#include "mission_executor/mission_state.hpp"
#include "mission_executor/position_controller.hpp"

using namespace std::chrono_literals;

namespace mission_executor
{

class MissionExecutorNode : public rclcpp::Node
{
    using ComputePath3D = uav_navigation_msgs::action::ComputePath3D;
    using PlannerGoalHandle = rclcpp_action::ClientGoalHandle<ComputePath3D>;
public:
    MissionExecutorNode()
        : Node("mission_executor_node"),
          current_state_(MissionState::IDLE),
          position_controller_(0.15, 0.50, 0.15, 1.0)
    {
        this->declare_parameter<std::string>("mission_file", "");
        this->declare_parameter<bool>("enable_task_sequence_check", false);
        this->declare_parameter<bool>("enable_task_flight", false);
        this->declare_parameter<bool>("task_flight_confirmed", false);
        this->declare_parameter<bool>("task_slow_sitl_telemetry", false);
        this->declare_parameter<bool>("enable_task_planned_flight", false);
        this->declare_parameter<bool>("task_planner_frame_confirmed", false);
        this->declare_parameter<double>("task_path_point_tolerance_m", 0.20);
        this->declare_parameter<double>("task_planned_max_z_m", 4.0);
        this->declare_parameter<double>("task_ground_wait_s", 3.0);
        this->declare_parameter<double>("task_max_direct_leg_m", 1.25);

        const bool task_sequence_check = this->get_parameter("enable_task_sequence_check").as_bool();
        task_flight_enabled_ = this->get_parameter("enable_task_flight").as_bool();
        task_slow_sitl_telemetry_ = this->get_parameter("task_slow_sitl_telemetry").as_bool();
        enable_task_planned_flight_ = this->get_parameter("enable_task_planned_flight").as_bool();
        task_path_point_tolerance_m_ = this->get_parameter("task_path_point_tolerance_m").as_double();
        task_planned_max_z_m_ = this->get_parameter("task_planned_max_z_m").as_double();
        task_ground_wait_s_ = this->get_parameter("task_ground_wait_s").as_double();
        task_max_direct_leg_m_ = this->get_parameter("task_max_direct_leg_m").as_double();
        const bool task_flight_confirmed = this->get_parameter("task_flight_confirmed").as_bool();
        const bool frame_confirmed = this->get_parameter("task_planner_frame_confirmed").as_bool();

        if (task_flight_enabled_ == task_sequence_check) {
            throw std::runtime_error(
                "Choose exactly one: enable_task_flight OR enable_task_sequence_check.");
        }
        if (task_flight_enabled_ && !task_flight_confirmed) {
            throw std::runtime_error(
                "A1.2.3 flight DENIED. Need task_flight_confirmed=true, "
                "verified coordinate frames, safe corridor and reliable telemetry.");
        }
        if (task_slow_sitl_telemetry_ && !task_flight_enabled_) {
            throw std::runtime_error(
                "task_slow_sitl_telemetry requires enable_task_flight=true.");
        }
        if (enable_task_planned_flight_ &&
            (!task_flight_enabled_ || !task_flight_confirmed || !frame_confirmed)) {
            throw std::runtime_error(
                "A* FLIGHT DENIED: need enable_task_flight, "
                "task_flight_confirmed, and independently verified "
                "task_planner_frame_confirmed=true. Not for real UAV.");
        }
        if (!std::isfinite(task_path_point_tolerance_m_) ||
            task_path_point_tolerance_m_ < 0.10 ||
            task_path_point_tolerance_m_ > 0.25) {
            throw std::runtime_error("task_path_point_tolerance_m must be within 0.10..0.25m");
        }
        if (!std::isfinite(task_planned_max_z_m_) ||
            task_planned_max_z_m_ < 1.0 || task_planned_max_z_m_ > 6.0) {
            throw std::runtime_error("task_planned_max_z_m must be within 1.0..6.0m");
        }
        if (!std::isfinite(task_ground_wait_s_) || task_ground_wait_s_ < 2.0 ||
            task_ground_wait_s_ > 30.0 ||
            !std::isfinite(task_max_direct_leg_m_) ||
            task_max_direct_leg_m_ <= 0.0 || task_max_direct_leg_m_ > 1.5) {
            throw std::runtime_error("Invalid task wait or direct-leg safety limit.");
        }
        if (task_slow_sitl_telemetry_) {
            RCLCPP_WARN(this->get_logger(),
                "SLOW SITL telemetry mode enabled: simulation ONLY. "
                "Never use on a real UAV.");
        }
        if (enable_task_planned_flight_) {
            RCLCPP_WARN(this->get_logger(),
                "EXPERIMENTAL A* TASK FLIGHT: planning and route following "
                "enabled; no direct fallback if planning fails. "
                "Verify map->MAVROS transform and costmap inflation in Gazebo SITL.");
        }

        const std::string mission_file = this->get_parameter("mission_file").as_string();
        if (mission_file.empty()) {
            throw std::runtime_error("Parameter 'mission_file' is empty.");
        }
        mission_ = MissionParser::load_from_csv(mission_file);
        if (mission_.empty()) {
            throw std::runtime_error("Mission does not contain any waypoints.");
        }
        validate_task_mission();
        RCLCPP_INFO(this->get_logger(), "Mission loaded: %zu waypoints", mission_.size());
        for (std::size_t i = 0; i < mission_.size(); ++i) {
            const auto &w = mission_[i];
            const std::string task = w.task == TaskType::TAKEOFF ? "takeoff" :
                w.task == TaskType::LAND ? "land" :
                w.task == TaskType::LANDTAKEOFF ? "landtakeoff" :
                w.task == TaskType::YAW ? "yaw" + std::to_string(w.yaw_angle) : "none";
            RCLCPP_INFO(this->get_logger(),
                "TASK PLAN CSV[%zu/%zu] (%.2f, %.2f, %.2f) [%s] %s",
                i + 1, mission_.size(), w.x, w.y, w.z,
                w.precision == Precision::HARD ? "HARD" : "SOFT", task.c_str());
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
        mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&MissionExecutorNode::mavros_state_callback, this, std::placeholders::_1),
            state_options);
        extended_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
            "/mavros/extended_state", 10,
            std::bind(&MissionExecutorNode::extended_state_callback, this, std::placeholders::_1),
            state_options);

        rclcpp::SubscriptionOptions pose_options;
        pose_options.callback_group = pose_group_;
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
            std::bind(&MissionExecutorNode::pose_callback, this, std::placeholders::_1),
            pose_options);
        velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_local", rclcpp::SensorDataQoS(),
            std::bind(&MissionExecutorNode::velocity_callback, this, std::placeholders::_1),
            pose_options);

        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
            "/mavros/set_mode", rclcpp::ServicesQoS(), state_group_);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
            "/mavros/cmd/arming", rclcpp::ServicesQoS(), state_group_);
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(
            "/mavros/cmd/takeoff", rclcpp::ServicesQoS(), state_group_);
        land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(
            "/mavros/cmd/land", rclcpp::ServicesQoS(), state_group_);
        debug_setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mission_executor/debug_setpoint", 10);
        mavros_setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mavros/setpoint_raw/local", 10);

        if (task_sequence_check) {
            RCLCPP_WARN(this->get_logger(),
                "TASK SEQUENCE CHECK COMPLETE. No GUIDED/ARM/TAKEOFF/LAND "
                "requests, no setpoints, no state-machine execution.");
        } else {
            if (enable_task_planned_flight_) {
                planner_client_ = rclcpp_action::create_client<ComputePath3D>(
                    this, "/compute_path_3d");
            }
            state_timer_ = this->create_wall_timer(
                100ms, std::bind(&MissionExecutorNode::state_machine_callback, this),
                state_group_);
            setpoint_timer_ = this->create_wall_timer(
                50ms, std::bind(&MissionExecutorNode::publish_setpoint, this),
                setpoint_group_);
        }
        RCLCPP_INFO(this->get_logger(),
            "Callback groups initialized: state, setpoint, telemetry.");
        RCLCPP_INFO(this->get_logger(),
            "Real MAVROS setpoints: %s",
            task_flight_enabled_ ? "ENABLED (guarded SITL only)" : "DISABLED");
    }

private:
    Mission mission_;
    MissionState current_state_;
    PositionController position_controller_;
    std::atomic<std::size_t> current_waypoint_index_{0};

    bool task_flight_enabled_ = false;
    bool task_slow_sitl_telemetry_ = false;
    bool enable_task_planned_flight_ = false;
    double task_path_point_tolerance_m_ = 0.20;
    double task_planned_max_z_m_ = 4.0;
    double task_ground_wait_s_ = 3.0;
    double task_max_direct_leg_m_ = 1.25;
    std::atomic<bool> task_navigation_active_{false};
    enum class TaskHeadingPhase { PREPARE, ALIGN, FLY, YAW_TASK };
    std::mutex task_heading_mutex_;
    TaskHeadingPhase task_heading_phase_ = TaskHeadingPhase::PREPARE;
    geometry_msgs::msg::Point task_heading_hold_position_{};
    double task_heading_target_rad_ = 0.0;
    std::chrono::steady_clock::time_point task_heading_phase_since_{};
    std::chrono::steady_clock::time_point task_heading_stable_since_{};
    bool task_heading_stable_started_ = false;
    bool task_landtakeoff_ = false;
    bool task_takeoff_sent_ = false;
    bool task_takeoff_response_received_ = false;
    bool task_takeoff_accepted_ = false;
    bool task_land_sent_ = false;
    bool task_land_response_received_ = false;
    bool task_land_accepted_ = false;
    bool task_disarm_sent_ = false;
    bool task_disarm_response_received_ = false;
    bool task_disarm_accepted_ = false;
    bool task_ground_confirmed_ = false;
    bool task_land_telemetry_missing_ = false;
    std::chrono::steady_clock::time_point task_land_telemetry_missing_since_{};
    bool task_stable_started_ = false;
    double task_takeoff_target_z_ = 0.0;
    std::chrono::steady_clock::time_point task_operation_started_{};
    std::chrono::steady_clock::time_point task_stable_since_{};
    std::chrono::steady_clock::time_point task_ground_wait_since_{};
    std::chrono::steady_clock::time_point task_preflight_started_{};
    rclcpp_action::Client<ComputePath3D>::SharedPtr planner_client_;
    enum class TaskRoutePhase { NEED_PLAN, WAIT_RESULT, FOLLOW_PATH, FAILURE };
    std::mutex task_route_mutex_;
    TaskRoutePhase task_route_phase_ = TaskRoutePhase::NEED_PLAN;
    std::vector<geometry_msgs::msg::Point> task_route_enu_;
    std::vector<uint8_t> task_route_corners_;
    std::size_t task_route_point_index_ = 0;
    std::size_t task_route_waypoint_index_ = 0;
    std::size_t task_route_sequence_ = 0;
    std::string task_route_failure_;
    std::chrono::steady_clock::time_point task_route_started_{};

    std::mutex mavros_mutex_;
    bool mavros_connected_ = false;
    bool mavros_armed_ = false;
    bool mavros_state_received_ = false;
    std::string mavros_mode_;
    std::chrono::steady_clock::time_point last_mavros_state_time_{};
    bool extended_state_received_ = false;
    uint8_t landed_state_ = mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED;
    std::chrono::steady_clock::time_point last_extended_state_time_{};

    static constexpr auto MAVROS_TIMEOUT = 10s;
    static constexpr auto MODE_TIMEOUT = 10s;
    static constexpr auto ARM_TIMEOUT = 12s;
    static constexpr auto MOTION_TIMEOUT = 2500ms;

    std::mutex pose_mutex_;
    geometry_msgs::msg::PoseStamped current_pose_;
    bool pose_received_ = false;
    std::chrono::steady_clock::time_point last_pose_time_{};
    std::deque<std::chrono::steady_clock::time_point> pose_samples_;


    geometry_msgs::msg::TwistStamped current_velocity_;
    bool velocity_received_ = false;
    std::chrono::steady_clock::time_point last_velocity_time_{};
    std::deque<std::chrono::steady_clock::time_point> velocity_samples_;

    rclcpp::CallbackGroup::SharedPtr state_group_;
    rclcpp::CallbackGroup::SharedPtr setpoint_group_;
    rclcpp::CallbackGroup::SharedPtr pose_group_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr
        mavros_state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr
        extended_state_sub_;
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
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr
        takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr
        land_client_;

    bool mode_request_sent_ = false;
    bool mode_response_received_ = false;
    bool mode_request_accepted_ = false;
    std::chrono::steady_clock::time_point mode_request_time_{};
    bool arm_request_sent_ = false;
    bool arm_response_received_ = false;
    bool arm_request_accepted_ = false;
    std::chrono::steady_clock::time_point arm_request_time_{};

    void validate_task_mission() const
    {
        if (mission_.size() < 3 || mission_.front().task != TaskType::TAKEOFF ||
            mission_.back().task != TaskType::LAND) {
            throw std::runtime_error(
                "A1.2.3 test CSV must begin with takeoff and end with land "
                "and contain at least three waypoints.");
        }
        for (std::size_t i = 0; i < mission_.size(); ++i) {
            const auto &w = mission_[i];
            if (!std::isfinite(w.x) || !std::isfinite(w.y) ||
                !std::isfinite(w.z) || w.z < 0.75 || w.z > 3.0) {
                throw std::runtime_error(
                    "A1.2.3 invalid waypoint coordinate or altitude; "
                    "test altitudes must be within 0.75..3.0 m.");
            }
            if ((w.task == TaskType::YAW &&
                 !std::isfinite(w.yaw_angle)) ||
                (w.task == TaskType::TAKEOFF && i != 0) ||
                (w.task == TaskType::LAND && i + 1 != mission_.size()) ||
                (w.task == TaskType::LANDTAKEOFF &&
                    (i == 0 || i + 1 == mission_.size()))) {
                throw std::runtime_error(
                    "A1.2 invalid task placement or nonfinite yaw angle.");
            }
            if (i != 0) {
                const auto &prev = mission_[i - 1];
                if (!enable_task_planned_flight_ &&
                    std::hypot(w.x - prev.x, w.y - prev.y) >
                        task_max_direct_leg_m_) {
                    throw std::runtime_error(
                        "A1.2.3 direct XY leg too long for this guarded SITL "
                        "test; use a short inspected obstacle-free corridor.");
                }
            }
        }
    }

    static void keep_recent_samples(
        std::deque<std::chrono::steady_clock::time_point> &times,
        std::chrono::steady_clock::time_point now)
    {
        times.push_back(now);
        while (times.size() > 8) times.pop_front();
    }

    static bool sample_rate_ok(
        const std::deque<std::chrono::steady_clock::time_point> &samples,
        bool slow_sitl)
    {
        return samples.size() == 8 &&
            samples.back() - samples.front() <= (slow_sitl ? 9s : 1400ms);
    }

    bool task_telemetry_ok(bool require_ground)
    {
        const auto now = std::chrono::steady_clock::now();
        const auto state_timeout = task_slow_sitl_telemetry_ ? 6s : 2s;
        const auto extended_timeout = task_slow_sitl_telemetry_ ? 4s : 2s;
        const auto motion_timeout = task_slow_sitl_telemetry_ ? 2500ms : 500ms;
        {
            std::lock_guard<std::mutex> lock(mavros_mutex_);
            if (!mavros_state_received_ || !mavros_connected_ ||
                now - last_mavros_state_time_ > state_timeout ||
                !extended_state_received_ ||
                now - last_extended_state_time_ > extended_timeout ||
                (require_ground && (
                    mavros_armed_ ||
                    landed_state_ !=
                        mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND))) {
                return false;
            }
        }
        std::lock_guard<std::mutex> lock(pose_mutex_);
        const auto &p = current_pose_.pose.position;
        const auto &v = current_velocity_.twist.linear;
        return pose_received_ && velocity_received_ &&
            now - last_pose_time_ <= motion_timeout &&
            now - last_velocity_time_ <= motion_timeout &&
            sample_rate_ok(pose_samples_, task_slow_sitl_telemetry_) &&
            sample_rate_ok(velocity_samples_, task_slow_sitl_telemetry_) &&
            current_pose_.header.frame_id == "map" &&
            finite_point(p) &&
            std::isfinite(v.x) && std::isfinite(v.y) &&
            std::isfinite(v.z);
    }

    bool task_airborne()
    {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mavros_mutex_);
        return extended_state_received_ &&
            now - last_extended_state_time_ <=
                (task_slow_sitl_telemetry_ ? 4s : 2s) &&
            landed_state_ ==
                mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR;
    }

    bool task_snapshot(
        geometry_msgs::msg::PoseStamped &pose,
        geometry_msgs::msg::TwistStamped &velocity)
    {
        if (!task_telemetry_ok(false)) return false;
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pose = current_pose_;
        velocity = current_velocity_;
        return true;
    }

    bool task_stable_for(
        bool condition, std::chrono::steady_clock::time_point now,
        std::chrono::steady_clock::duration duration)
    {
        if (!condition) {
            task_stable_started_ = false;
            return false;
        }
        if (!task_stable_started_) {
            task_stable_started_ = true;
            task_stable_since_ = now;
        }
        return now - task_stable_since_ >= duration;
    }

    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kYawToleranceRad = 8.0 * kPi / 180.0;

    static double angular_error(double target, double measured)
    {
        return std::remainder(target - measured, 2.0 * kPi);
    }

    static double unwrap_near(double target, double measured)
    {
        return measured + angular_error(target, measured);
    }

    static bool yaw_from_enu_pose(
        const geometry_msgs::msg::PoseStamped &pose, double &yaw)
    {
        if (pose.header.frame_id != "map") return false;
        const auto &q = pose.pose.orientation;
        if (!std::isfinite(q.x) || !std::isfinite(q.y) ||
            !std::isfinite(q.z) || !std::isfinite(q.w)) return false;
        const double n = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (!std::isfinite(n) || n < 1e-12) return false;
        yaw = std::atan2(2.0 * (q.w*q.z + q.x*q.y) / n,
            (q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z) / n);
        return std::isfinite(yaw);
    }

    TaskHeadingPhase heading_phase()
    {
        std::lock_guard<std::mutex> lock(task_heading_mutex_);
        return task_heading_phase_;
    }

    void set_heading_phase(
        TaskHeadingPhase phase, double yaw,
        const geometry_msgs::msg::Point &hold_position)
    {
        std::lock_guard<std::mutex> lock(task_heading_mutex_);
        task_heading_phase_ = phase;
        task_heading_target_rad_ = yaw;
        task_heading_hold_position_ = hold_position;
        task_heading_phase_since_ = std::chrono::steady_clock::now();
        task_heading_stable_started_ = false;
    }

    void reset_heading_for_next_leg()
    {
        std::lock_guard<std::mutex> lock(task_heading_mutex_);
        task_heading_phase_ = TaskHeadingPhase::PREPARE;
        task_heading_stable_started_ = false;
    }

    void reset_task_route()
    {
        if (!enable_task_planned_flight_) return;
        std::lock_guard<std::mutex> lock(task_route_mutex_);
        ++task_route_sequence_;  
        task_route_phase_ = TaskRoutePhase::NEED_PLAN;
        task_route_enu_.clear();
        task_route_corners_.clear();
        task_route_point_index_ = 0;
        task_route_waypoint_index_ = current_waypoint_index_.load();
        task_route_failure_.clear();
        task_route_started_ = std::chrono::steady_clock::now();
    }

    bool task_route_target(geometry_msgs::msg::Point &target,
                           bool &final_point, std::size_t &point_index,
                           std::size_t &point_count)
    {
        std::lock_guard<std::mutex> lock(task_route_mutex_);
        if (task_route_phase_ != TaskRoutePhase::FOLLOW_PATH ||
            task_route_waypoint_index_ != current_waypoint_index_.load() ||
            task_route_point_index_ >= task_route_enu_.size()) return false;
        point_index = task_route_point_index_;
        point_count = task_route_enu_.size();
        final_point = point_index + 1 == point_count;
        target = task_route_enu_[point_index];
        return true;
    }

    void request_task_route()
    {
        if (!planner_client_ || !planner_client_->action_server_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "A* TASK: waiting for /compute_path_3d; holding current position.");
            return;
        }
        geometry_msgs::msg::PoseStamped pose;
        geometry_msgs::msg::TwistStamped vel;
        if (!task_snapshot(pose, vel) || !task_airborne()) return;
        const auto index = current_waypoint_index_.load();
        if (index >= mission_.size()) return;
        const auto &p = pose.pose.position;
        const std::vector<double> start{p.y, -p.x, p.z};
        const auto &wp = mission_[index];
        const std::vector<double> goal{wp.x, wp.y, wp.z};
        auto request = make_planner_goal(start, goal);
        std::size_t sequence = 0;
        {
            std::lock_guard<std::mutex> lock(task_route_mutex_);
            if (task_route_phase_ != TaskRoutePhase::NEED_PLAN ||
                task_route_waypoint_index_ != index) return;
            sequence = ++task_route_sequence_;
            task_route_phase_ = TaskRoutePhase::WAIT_RESULT;
            task_route_started_ = std::chrono::steady_clock::now();
        }
        RCLCPP_INFO(this->get_logger(),
            "A* TASK: planning WP%zu map (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)",
            index + 1, start[0], start[1], start[2], goal[0], goal[1], goal[2]);
        rclcpp_action::Client<ComputePath3D>::SendGoalOptions options;
        options.goal_response_callback = [this, sequence](
            const PlannerGoalHandle::SharedPtr &handle) {
            if (handle) return;
            std::lock_guard<std::mutex> lock(task_route_mutex_);
            if (sequence == task_route_sequence_ &&
                task_route_phase_ == TaskRoutePhase::WAIT_RESULT) {
                task_route_phase_ = TaskRoutePhase::FAILURE;
                task_route_failure_ = "Planner rejected the goal";
            }
        };
        options.result_callback = [this, sequence, index, start, goal](
            const PlannerGoalHandle::WrappedResult &wrapped) {
            bool success = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
                wrapped.result &&
                wrapped.result->status == ComputePath3D::Result::SUCCESS &&
                validate_planner_path(wrapped.result->path, start, goal);
            std::vector<geometry_msgs::msg::Point> route;
            std::vector<uint8_t> corners;
            if (success) {
                const auto &poses = wrapped.result->path.poses;
                route.push_back(provisional_map_to_mavros(poses.front().pose.position));
                corners.push_back(1);
                for (std::size_t j = 1; j < poses.size(); ++j) {
                    const auto a = provisional_map_to_mavros(poses[j-1].pose.position);
                    const auto b = provisional_map_to_mavros(poses[j].pose.position);
                    const double len = distance_between(a, b);
                    if (!std::isfinite(a.z) || !std::isfinite(b.z) ||
                        a.z < 0.50 || b.z < 0.50 ||
                        a.z > task_planned_max_z_m_ ||
                        b.z > task_planned_max_z_m_) {
                        success = false; break;
                    }
                    if (!std::isfinite(len)) { success = false; break; }
                    const auto parts = static_cast<std::size_t>(std::ceil(len / 0.35));
                    if (parts > 3000 || route.size() + parts > 3000) {
                        success = false; break;
                    }
                    for (std::size_t k = 1; k <= parts; ++k) {
                        const double t = static_cast<double>(k) / parts;
                        geometry_msgs::msg::Point next;
                        next.x = a.x + t * (b.x - a.x);
                        next.y = a.y + t * (b.y - a.y);
                        next.z = a.z + t * (b.z - a.z);
                        route.push_back(next);
                        corners.push_back(k == parts ? 1 : 0);
                    }
                }
                if (route.empty() || corners.size() != route.size()) success = false;
                if (route.front().z < 0.50 ||
                    route.front().z > task_planned_max_z_m_) success = false;
                if (success && distance_between(route.back(),
                    position_controller_.gazebo_to_enu(mission_[index])) > 0.05) {
                    success = false;
                }
            }
            std::lock_guard<std::mutex> lock(task_route_mutex_);
            if (sequence != task_route_sequence_ ||
                task_route_phase_ != TaskRoutePhase::WAIT_RESULT ||
                task_route_waypoint_index_ != index) return;
            if (!success) {
                task_route_phase_ = TaskRoutePhase::FAILURE;
                task_route_failure_ = "A* failed or route invalid/too long";
                return;
            }
            task_route_enu_ = std::move(route);
            task_route_corners_ = std::move(corners);
            task_route_point_index_ = task_route_enu_.size() > 1 ? 1 : 0;
            task_route_phase_ = TaskRoutePhase::FOLLOW_PATH;
            RCLCPP_INFO(this->get_logger(),
                "A* TASK: WP%zu route received, %zu position targets; "
                "flight Z ceiling %.2fm; no direct fallback.",
                index + 1, task_route_enu_.size(), task_planned_max_z_m_);
        };
        planner_client_->async_send_goal(request, options);
    }

    bool task_route_ready(std::chrono::steady_clock::time_point now)
    {
        TaskRoutePhase phase;
        std::string failure;
        bool expired = false;
        {
            std::lock_guard<std::mutex> lock(task_route_mutex_);
            phase = task_route_phase_;
            failure = task_route_failure_;
            expired = now - task_route_started_ > 30s &&
                (phase == TaskRoutePhase::NEED_PLAN ||
                 phase == TaskRoutePhase::WAIT_RESULT);
        }
        if (expired || phase == TaskRoutePhase::FAILURE) {
            RCLCPP_ERROR(this->get_logger(), "A* TASK planning failed: %s",
                expired ? "planner unavailable/result timed out" : failure.c_str());
            change_state(MissionState::ERROR);
            return false;
        }
        if (phase == TaskRoutePhase::NEED_PLAN) {
            request_task_route();
            return false;
        }
        return phase == TaskRoutePhase::FOLLOW_PATH;
    }

    bool advance_task_route_point()
    {
        geometry_msgs::msg::Point target;
        bool final_point;
        std::size_t point_index, point_count;
        if (!task_route_target(target, final_point, point_index, point_count)) {
            change_state(MissionState::ERROR);
            return false;
        }
        if (final_point) return false;
        geometry_msgs::msg::PoseStamped pose;
        geometry_msgs::msg::TwistStamped velocity;
        if (!task_snapshot(pose, velocity)) {
            change_state(MissionState::ERROR);
            return false;
        }
        const double distance = distance_between(pose.pose.position, target);
        bool is_bend = false;
        {
            std::lock_guard<std::mutex> lock(task_route_mutex_);
            if (point_index >= task_route_corners_.size()) {
                change_state(MissionState::ERROR);
                return false;
            }
            is_bend = task_route_corners_[point_index] != 0;
        }
        const double tolerance = is_bend ?
            std::min(task_path_point_tolerance_m_, 0.10) :
            task_path_point_tolerance_m_;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "A* TASK path[%zu/%zu] dist=%.3f m (limit %.2f m)",
            point_index + 1, point_count, distance, tolerance);
        if (distance > tolerance) return false;
        {
            std::lock_guard<std::mutex> lock(task_route_mutex_);
            if (task_route_phase_ != TaskRoutePhase::FOLLOW_PATH ||
                task_route_point_index_ != point_index) return false;
            ++task_route_point_index_;
        }
        RCLCPP_INFO(this->get_logger(),
            "A* TASK: reached path point %zu/%zu, advance.",
            point_index + 1, point_count);
        if (is_bend) {
            reset_heading_for_next_leg();
        }
        return true;
    }

    bool prepare_task_leg(std::size_t index)
    {
        if (index == 0 || index >= mission_.size()) return false;
        geometry_msgs::msg::PoseStamped pose;
        geometry_msgs::msg::TwistStamped velocity;
        if (!task_snapshot(pose, velocity)) return false;
        double current_yaw = 0.0;
        if (!yaw_from_enu_pose(pose, current_yaw)) return false;
        auto from = position_controller_.gazebo_to_enu(mission_[index - 1]);
        auto to = position_controller_.gazebo_to_enu(mission_[index]);
        if (enable_task_planned_flight_) {
            bool final_point;
            std::size_t point_index, point_count;
            if (!task_route_target(to, final_point, point_index, point_count))
                return false;
            from = pose.pose.position;
        }
        const double dx = to.x - from.x;
        const double dy = to.y - from.y;
        const double desired_yaw = std::hypot(dx, dy) > 0.05 ?
            std::atan2(dy, dx) : current_yaw;
        const double commanded_yaw = unwrap_near(desired_yaw, current_yaw);
        set_heading_phase(TaskHeadingPhase::ALIGN, commanded_yaw,
            pose.pose.position);
        RCLCPP_INFO(this->get_logger(),
            "WP%zu: ALIGN BEFORE MOVING. ENU segment (%.2f,%.2f) -> "
            "(%.2f,%.2f); travel heading %.1f deg ENU.",
            index + 1, from.x, from.y, to.x, to.y,
            desired_yaw * 180.0 / kPi);
        return true;
    }

    bool begin_waypoint_yaw(std::size_t index)
    {
        geometry_msgs::msg::PoseStamped pose;
        geometry_msgs::msg::TwistStamped velocity;
        if (!task_snapshot(pose, velocity)) return false;
        double measured_yaw = 0.0;
        if (!yaw_from_enu_pose(pose, measured_yaw)) return false;
        const auto goal = position_controller_.gazebo_to_enu(mission_[index]);
        const double target = unwrap_near(
            std::remainder(mission_[index].yaw_angle, 360.0) *
                kPi / 180.0, measured_yaw);
        set_heading_phase(TaskHeadingPhase::YAW_TASK, target, goal);
        RCLCPP_INFO(this->get_logger(),
            "WP%zu: YAW TASK %.1f degrees absolute ENU; hold at waypoint.",
            index + 1, mission_[index].yaw_angle);
        return true;
    }

    bool task_heading_held(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::TwistStamped &velocity,
        std::chrono::steady_clock::time_point now)
    {
        double measured_yaw = 0.0;
        if (!yaw_from_enu_pose(pose, measured_yaw)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid ENU quaternion during yaw.");
            change_state(MissionState::ERROR);
            return false;
        }
        double commanded_yaw = 0.0;
        geometry_msgs::msg::Point hold_position;
        std::chrono::steady_clock::time_point since;
        TaskHeadingPhase phase;
        {
            std::lock_guard<std::mutex> lock(task_heading_mutex_);
            phase = task_heading_phase_;
            commanded_yaw = task_heading_target_rad_;
            hold_position = task_heading_hold_position_;
            since = task_heading_phase_since_;
        }
        if (phase != TaskHeadingPhase::ALIGN &&
            phase != TaskHeadingPhase::YAW_TASK) return false;
        if (now - since > 60s) {
            RCLCPP_ERROR(this->get_logger(),
                "Yaw/segment alignment timed out (60 s). Setpoints stopped.");
            change_state(MissionState::ERROR);
            return false;
        }
        const double err = std::fabs(angular_error(commanded_yaw, measured_yaw));
        const auto &v = velocity.twist.linear;
        const double speed = std::hypot(std::hypot(v.x, v.y), v.z);
        const double position_err = distance_between(
            pose.pose.position, hold_position);
        const bool within_limits = err <= kYawToleranceRad &&
            speed <= 0.25 && position_err <= 0.25;
        if (!within_limits) {
            task_heading_stable_started_ = false;
        } else if (!task_heading_stable_started_) {
            task_heading_stable_started_ = true;
            task_heading_stable_since_ = now;
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "%s: target=%.1f deg ENU measured=%.1f deg ENU "
            "error=%.1f deg drift=%.2f m stable=%s",
            phase == TaskHeadingPhase::ALIGN ? "LEG ALIGN" : "WAYPOINT YAW",
            commanded_yaw * 180.0 / kPi, measured_yaw * 180.0 / kPi,
            err * 180.0 / kPi, position_err, within_limits ? "YES" : "NO");
        return task_heading_stable_started_ &&
            now - task_heading_stable_since_ >= 1s;
    }

    void request_takeoff_task(double altitude_above_ground)
    {
        if (!takeoff_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for /mavros/cmd/takeoff...");
            return;
        }
        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->altitude = altitude_above_ground;
        task_takeoff_sent_ = true;
        task_takeoff_response_received_ = false;
        task_takeoff_accepted_ = false;
        task_operation_started_ = std::chrono::steady_clock::now();
        RCLCPP_WARN(this->get_logger(),
            "SITL TAKEOFF command, relative climb %.2fm. "
            "Waiting for actual airborne state and Z feedback.",
            altitude_above_ground);
        takeoff_client_->async_send_request(request,
            [this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
                try {
                    task_takeoff_accepted_ = future.get()->success;
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "TAKEOFF call: %s", e.what());
                    task_takeoff_accepted_ = false;
                }
                task_takeoff_response_received_ = true;
            });
    }

    void request_land_task()
    {
        if (!land_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for /mavros/cmd/land...");
            return;
        }
        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        task_land_sent_ = true;
        task_land_response_received_ = false;
        task_land_accepted_ = false;
        task_operation_started_ = std::chrono::steady_clock::now();
        RCLCPP_WARN(this->get_logger(),
            "SITL LAND command at current waypoint; stop position setpoints.");
        land_client_->async_send_request(request,
            [this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
                try {
                    task_land_accepted_ = future.get()->success;
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "LAND call: %s", e.what());
                    task_land_accepted_ = false;
                }
                task_land_response_received_ = true;
            });
    }

    void request_disarm_task()
    {
        if (!arming_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for /mavros/cmd/arming to disarm...");
            return;
        }
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = false;
        task_disarm_sent_ = true;
        task_disarm_response_received_ = false;
        task_disarm_accepted_ = false;
        RCLCPP_INFO(this->get_logger(),
            "Ground confirmed, requesting disarm if not already disarmed.");
        arming_client_->async_send_request(request,
            [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                try {
                    task_disarm_accepted_ = future.get()->success;
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "DISARM call: %s", e.what());
                    task_disarm_accepted_ = false;
                }
                task_disarm_response_received_ = true;
            });
    }

    void task_takeoff_tick(
        bool armed, const std::string &mode,
        std::chrono::steady_clock::time_point now)
    {
        if (!armed || mode != "GUIDED") {
            RCLCPP_ERROR(this->get_logger(),
                "TAKEOFF interrupted: arm/GUIDED confirmation lost.");
            change_state(MissionState::ERROR);
            return;
        }
        geometry_msgs::msg::PoseStamped pose;
        geometry_msgs::msg::TwistStamped velocity;
        if (!task_snapshot(pose, velocity)) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TAKEOFF waiting for fresh pose/velocity and MAVROS state.");
            if ((task_takeoff_sent_ && now - task_operation_started_ > 8s) ||
                (!task_takeoff_sent_ && now - task_operation_started_ > 20s)) {
                change_state(MissionState::ERROR);
            }
            return;
        }
        const std::size_t idx = current_waypoint_index_.load();
        if (idx >= mission_.size()) {
            change_state(MissionState::ERROR);
            return;
        }
        const auto goal = position_controller_.gazebo_to_enu(mission_[idx]);
        const auto &p = pose.pose.position;
        const double xy_error = std::hypot(p.x - goal.x, p.y - goal.y);
        if (!task_takeoff_sent_) {
            if (now - task_operation_started_ > 20s) {
                RCLCPP_ERROR(this->get_logger(),
                    "TAKEOFF preparation timeout; no command sent.");
                change_state(MissionState::ERROR);
                return;
            }
            bool on_ground = false;
            {
                std::lock_guard<std::mutex> lock(mavros_mutex_);
                on_ground = landed_state_ ==
                    mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;
            }
            if (!on_ground) {
                RCLCPP_ERROR(this->get_logger(),
                    "TAKEOFF blocked: vehicle no longer reports ON_GROUND.");
                change_state(MissionState::ERROR);
                return;
            }
            if (xy_error > 0.45) {
                RCLCPP_ERROR(this->get_logger(),
                    "TAKEOFF blocked: current XY is %.2fm from the takeoff "
                    "waypoint. No lateral flight allowed during takeoff.", xy_error);
                change_state(MissionState::ERROR);
                return;
            }
            if (task_snapshot(pose, velocity) &&
                mission_[idx].z - p.z >= 0.75 &&
                mission_[idx].z - p.z <= 3.0) {
                task_takeoff_target_z_ = goal.z;
                request_takeoff_task(goal.z - p.z);
            } else {
                RCLCPP_ERROR(this->get_logger(),
                    "TAKEOFF invalid requested climb/ground Z.");
                change_state(MissionState::ERROR);
            }
            return;
        }
        if (task_takeoff_response_received_ && !task_takeoff_accepted_) {
            RCLCPP_ERROR(this->get_logger(), "Autopilot rejected TAKEOFF request.");
            change_state(MissionState::ERROR);
            return;
        }
        if (now - task_operation_started_ > 50s) {
            RCLCPP_ERROR(this->get_logger(), "TAKEOFF timed out.");
            change_state(MissionState::ERROR);
            return;
        }
        bool airborne = false;
        {
            std::lock_guard<std::mutex> lock(mavros_mutex_);
            airborne = landed_state_ ==
                mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR;
        }
        const double speed = std::hypot(std::hypot(velocity.twist.linear.x,
            velocity.twist.linear.y), velocity.twist.linear.z);
        const bool ready = task_takeoff_response_received_ &&
            task_takeoff_accepted_ && airborne &&
            std::fabs(p.z - task_takeoff_target_z_) <= 0.20 &&
            speed <= 0.25 && xy_error <= 0.45;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "TAKEOFF monitor: measured Z=%.2f target=%.2f airborne=%s "
            "speed=%.2f stable=%s", p.z, task_takeoff_target_z_,
            airborne ? "YES" : "NO", speed, ready ? "YES" : "NO");
        if (task_stable_for(ready, now, 1s)) {
            RCLCPP_INFO(this->get_logger(),
                "TAKEOFF verified: airborne and target height stable. "
                "WP%zu complete.", idx + 1);
            current_waypoint_index_.store(idx + 1);
            position_controller_.reset_arrival();
            change_state(MissionState::NAVIGATE);
        }
    }

    void task_land_tick(
        bool armed, std::chrono::steady_clock::time_point now)
    {
        if (!task_land_sent_) {
            if (now - task_operation_started_ > 20s) {
                RCLCPP_ERROR(this->get_logger(),
                    "LAND preparation timeout; no command sent.");
                change_state(MissionState::ERROR);
                return;
            }
            if (!task_telemetry_ok(false)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "LAND waiting for fresh telemetry.");
                return;
            }
            request_land_task();
            return;
        }

        if (!task_telemetry_ok(false)) {
            if (!task_land_telemetry_missing_) {
                task_land_telemetry_missing_ = true;
                task_land_telemetry_missing_since_ = now;
                RCLCPP_WARN(this->get_logger(),
                    "LAND: lost fresh telemetry; autonomous landing may continue.");
            }
            if (now - task_land_telemetry_missing_since_ > 3s) {
                RCLCPP_ERROR(this->get_logger(),
                    "LAND telemetry stale for >3s: ERROR. Do not issue another "
                    "takeoff; inspect vehicle and intervene manually if needed. "
                    "Autopilot may still be landing.");
                change_state(MissionState::ERROR);
            }
            return;
        }
        task_land_telemetry_missing_ = false;
        if (task_land_response_received_ && !task_land_accepted_) {
            RCLCPP_ERROR(this->get_logger(), "Autopilot rejected LAND request.");
            change_state(MissionState::ERROR);
            return;
        }
        if (now - task_operation_started_ > 90s) {
            RCLCPP_ERROR(this->get_logger(),
                "LAND timeout. Autopilot might still be landing; inspect SITL.");
            change_state(MissionState::ERROR);
            return;
        }
        if (!task_land_response_received_ || !task_land_accepted_) return;
        bool ground = false;
        {
            std::lock_guard<std::mutex> lock(mavros_mutex_);
            ground = extended_state_received_ &&
                now - last_extended_state_time_ <=
                    (task_slow_sitl_telemetry_ ? 4s : 2s) &&
                landed_state_ ==
                    mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;
        }
        if (!ground) {
            task_stable_started_ = false;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "LAND in progress: waiting for ON_GROUND extended state.");
            return;
        }
        geometry_msgs::msg::PoseStamped pose;
        geometry_msgs::msg::TwistStamped vel;
        if (!task_snapshot(pose, vel)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "LAND ground reported, waiting for fresh telemetry before disarm.");
            return;
        }
        if (std::fabs(vel.twist.linear.z) > 0.20) {
            task_stable_started_ = false;
            return;
        }
        if (armed) {
            if (!task_disarm_sent_) request_disarm_task();
            if (task_disarm_response_received_ && !task_disarm_accepted_) {
                RCLCPP_ERROR(this->get_logger(), "Autopilot rejected DISARM.");
                change_state(MissionState::ERROR);
            }
            return;
        }
        if (!task_ground_confirmed_) {
            if (!task_stable_for(true, now, 1s)) return;
            task_ground_confirmed_ = true;
            task_ground_wait_since_ = now;
            RCLCPP_INFO(this->get_logger(),
                "LAND verified: ON_GROUND and DISARMED. Waiting %.1fs.",
                task_landtakeoff_ ? task_ground_wait_s_ : 0.0);
        }
        if (!task_landtakeoff_) {
            current_waypoint_index_.store(mission_.size());
            change_state(MissionState::DONE);
            return;
        }
        if (now - task_ground_wait_since_ <
            std::chrono::duration<double>(task_ground_wait_s_)) return;
        task_landtakeoff_ = false;
        RCLCPP_WARN(this->get_logger(),
            "LANDTAKEOFF: ground wait complete, re-enter GUIDED then ARM "
            "before attempting a new takeoff from waypoint %zu.",
            current_waypoint_index_.load() + 1);
        change_state(MissionState::SET_MODE);
    }

    static geometry_msgs::msg::Point provisional_map_to_mavros(
        const geometry_msgs::msg::Point &map_point)
    {
        geometry_msgs::msg::Point point;
        point.x = -map_point.y;
        point.y = map_point.x;
        point.z = map_point.z;
        return point;
    }

    static bool finite_point(const geometry_msgs::msg::Point &p)
    {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    }

    static double distance_between(
        const geometry_msgs::msg::Point &a,
        const geometry_msgs::msg::Point &b)
    {
        return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
    }

    ComputePath3D::Goal make_planner_goal(
        const std::vector<double> &start,
        const std::vector<double> &goal)
    {
        ComputePath3D::Goal request;
        request.start.header.frame_id = "map";
        request.start.header.stamp = this->now();
        request.start.pose.position.x = start[0];
        request.start.pose.position.y = start[1];
        request.start.pose.position.z = start[2];
        request.start.pose.orientation.w = 1.0;
        request.goal.header.frame_id = "map";
        request.goal.header.stamp = this->now();
        request.goal.pose.position.x = goal[0];
        request.goal.pose.position.y = goal[1];
        request.goal.pose.position.z = goal[2];
        request.goal.pose.orientation.w = 1.0;
        return request;
    }

    bool validate_planner_path(
        const nav_msgs::msg::Path &path,
        const std::vector<double> &requested_start,
        const std::vector<double> &requested_goal) const
    {
        if (path.header.frame_id != "map" || path.poses.empty() ||
            path.poses.size() > 10000) {
            RCLCPP_ERROR(this->get_logger(),
                "Planner result rejected: invalid frame/empty/oversized path.");
            return false;
        }
        for (std::size_t i = 0; i < path.poses.size(); ++i) {
            const auto &pose = path.poses[i];
            if ((!pose.header.frame_id.empty() &&
                 pose.header.frame_id != "map") ||
                !finite_point(pose.pose.position)) {
                RCLCPP_ERROR(this->get_logger(),
                    "Planner result rejected: invalid point/frame at index %zu.", i);
                return false;
            }
        }
        geometry_msgs::msg::Point expected_start;
        expected_start.x = requested_start.at(0);
        expected_start.y = requested_start.at(1);
        expected_start.z = requested_start.at(2);
        geometry_msgs::msg::Point expected_goal;
        expected_goal.x = requested_goal.at(0);
        expected_goal.y = requested_goal.at(1);
        expected_goal.z = requested_goal.at(2);
        if (distance_between(path.poses.front().pose.position, expected_start) >
                0.25 ||
            distance_between(path.poses.back().pose.position, expected_goal) >
                0.25) {
            RCLCPP_ERROR(this->get_logger(),
                "Planner result rejected: endpoints do not match request.");
            return false;
        }
        return true;
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

    void extended_state_callback(
        const mavros_msgs::msg::ExtendedState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mavros_mutex_);
        extended_state_received_ = true;
        landed_state_ = msg->landed_state;
        last_extended_state_time_ = std::chrono::steady_clock::now();
    }

    void pose_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_pose_ = *msg;
            pose_received_ = true;
            last_pose_time_ = std::chrono::steady_clock::now();
            keep_recent_samples(pose_samples_, last_pose_time_);
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
            keep_recent_samples(velocity_samples_, last_velocity_time_);
        }
        const auto &v = msg->twist.linear;
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Current velocity: vx=%.3f, vy=%.3f, vz=%.3f m/s",
            v.x, v.y, v.z);
    }

    bool process_current_waypoint()
    {
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
                now - last_pose_time_ <= MOTION_TIMEOUT;
            velocity_ok = velocity_received_ &&
                now - last_velocity_time_ <= MOTION_TIMEOUT;
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
                "Navigation waiting for fresh %s%s%s",
                !pose_ok ? "pose" : "",
                !pose_ok && !velocity_ok ? " and " : "",
                !velocity_ok ? "velocity" : "");
            return false;
        }

        MissionWaypoint waypoint = mission_[index];
        if (waypoint.task == TaskType::LAND ||
            waypoint.task == TaskType::LANDTAKEOFF ||
            waypoint.task == TaskType::YAW) {
            waypoint.precision = Precision::HARD;
        }
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
        if (task_flight_enabled_) {
            const std::size_t idx = current_waypoint_index_.load();
            const bool is_yaw_task = idx < mission_.size() &&
                mission_[idx].task == TaskType::YAW;
            if (new_state == MissionState::NAVIGATE) {
                reset_heading_for_next_leg();
                reset_task_route();
            }
            task_navigation_active_.store(
                new_state == MissionState::NAVIGATE ||
                (new_state == MissionState::TASK && is_yaw_task));
            if (new_state == MissionState::SET_MODE) {
                mode_request_sent_ = false;
                mode_response_received_ = false;
                mode_request_accepted_ = false;
                task_preflight_started_ = std::chrono::steady_clock::now();
            }
            if (new_state == MissionState::ARM) {
                arm_request_sent_ = false;
                arm_response_received_ = false;
                arm_request_accepted_ = false;
            }
            if (new_state == MissionState::TAKEOFF) {
                task_operation_started_ = std::chrono::steady_clock::now();
                task_takeoff_sent_ = false;
                task_takeoff_response_received_ = false;
                task_takeoff_accepted_ = false;
                task_stable_started_ = false;
            }
            if (new_state == MissionState::LAND) {
                task_operation_started_ = std::chrono::steady_clock::now();
                task_land_sent_ = false;
                task_land_response_received_ = false;
                task_land_accepted_ = false;
                task_disarm_sent_ = false;
                task_disarm_response_received_ = false;
                task_disarm_accepted_ = false;
                task_ground_confirmed_ = false;
                task_land_telemetry_missing_ = false;
                task_stable_started_ = false;
            }
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
        const auto index = current_waypoint_index_.load();
        if (index >= mission_.size()) return;
        debug_setpoint_pub_->publish(create_position_target());

        bool state_ok = false;
        bool armed = false;
        std::string mode;
        {
            std::lock_guard<std::mutex> lock(mavros_mutex_);
            state_ok = mavros_state_received_ && mavros_connected_ &&
                (std::chrono::steady_clock::now() - last_mavros_state_time_
                 <= MAVROS_TIMEOUT);
            armed = mavros_armed_;
            mode = mavros_mode_;
        }
        if (!task_navigation_active_.load() || !state_ok || !armed ||
            mode != "GUIDED" || !task_telemetry_ok(false) || !task_airborne())
            return;
        const auto goal_idx = current_waypoint_index_.load();
        if (goal_idx == 0 || goal_idx >= mission_.size()) return;
        TaskHeadingPhase phase;
        double commanded_yaw;
        geometry_msgs::msg::Point hold_position;
        {
            std::lock_guard<std::mutex> lock(task_heading_mutex_);
            phase = task_heading_phase_;
            commanded_yaw = task_heading_target_rad_;
            hold_position = task_heading_hold_position_;
        }
        if (phase == TaskHeadingPhase::PREPARE) {
            if (!enable_task_planned_flight_) return;
            geometry_msgs::msg::PoseStamped p;
            geometry_msgs::msg::TwistStamped v;
            double current_yaw = 0.0;
            if (!task_snapshot(p, v) || !yaw_from_enu_pose(p, current_yaw)) return;
            auto hold_target = create_position_target();
            hold_target.position = p.pose.position;
            hold_target.yaw = static_cast<float>(current_yaw);
            mavros_setpoint_pub_->publish(hold_target);
            return;
        }
        auto target = create_position_target();
        if (phase == TaskHeadingPhase::FLY && enable_task_planned_flight_) {
            bool final_point;
            std::size_t point_index, point_count;
            if (!task_route_target(target.position, final_point,
                                   point_index, point_count)) return;
        } else {
            target.position = phase == TaskHeadingPhase::FLY ?
                position_controller_.gazebo_to_enu(mission_[goal_idx]) :
                hold_position;
        }
        target.yaw = static_cast<float>(commanded_yaw);
        mavros_setpoint_pub_->publish(target);

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
            last_state_time = last_mavros_state_time_;
        }
        const auto now = std::chrono::steady_clock::now();
        if (state_received &&
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
                RCLCPP_INFO(this->get_logger(), "Mission ready. Waiting for MAVROS.");
                change_state(MissionState::WAIT_CONNECTION);
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
                if (!task_telemetry_ok(true)) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                        3000, "A1.2 preflight blocked: need ON_GROUND, "
                        "DISARMED, connected state and fresh pose/velocity "
                        "matching selected telemetry profile.");
                    if (now - task_preflight_started_ > 30s) {
                        RCLCPP_ERROR(this->get_logger(),
                            "A1.2.3 preflight timeout. No arming attempted.");
                        change_state(MissionState::ERROR);
                    }
                    break;
                }
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
                if (mode != "GUIDED") {
                    RCLCPP_ERROR(this->get_logger(),
                        "A1.2.3 GUIDED lost before arming.");
                    change_state(MissionState::ERROR);
                    break;
                }
                if (armed) {
                    RCLCPP_INFO(this->get_logger(),
                        "A1.2.3 ARM confirmed, entering TAKEOFF.");
                    change_state(MissionState::TAKEOFF);
                    break;
                }
                if (!task_telemetry_ok(true)) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(),
                        *this->get_clock(), 2000,
                        "A1.2.3 ARM waiting for healthy ground telemetry.");
                    break;
                }
                if (!arm_request_sent_) {
                    geometry_msgs::msg::PoseStamped check_pose;
                    geometry_msgs::msg::TwistStamped check_velocity;
                    if (!task_snapshot(check_pose, check_velocity)) break;
                    const auto index = current_waypoint_index_.load();
                    const auto target =
                        position_controller_.gazebo_to_enu(mission_[index]);
                    const auto &p = check_pose.pose.position;
                    if (std::hypot(p.x - target.x, p.y - target.y) > 0.45) {
                        RCLCPP_ERROR(this->get_logger(),
                            "A1.2.3 ARM refused: not above planned takeoff XY.");
                        change_state(MissionState::ERROR);
                        break;
                    }
                    request_arm();
                }
                if (arm_response_received_ && !arm_request_accepted_) {
                    RCLCPP_ERROR(this->get_logger(),
                        "A1.2.3 ARM request rejected.");
                    change_state(MissionState::ERROR);
                } else if (arm_request_sent_ &&
                    now - arm_request_time_ > ARM_TIMEOUT) {
                    RCLCPP_ERROR(this->get_logger(),
                        "A1.2.3 ARM acknowledgement timeout.");
                    change_state(MissionState::ERROR);
                }
                break;
            }
            case MissionState::TAKEOFF:
            {
                task_takeoff_tick(armed, mode, now);
                break;
            }
            case MissionState::NAVIGATE:
            {
                if (!armed || mode != "GUIDED") {
                    RCLCPP_ERROR(this->get_logger(),
                        "A1.2.3 NAVIGATE: arm/GUIDED lost.");
                    change_state(MissionState::ERROR);
                    break;
                }
                if (!task_telemetry_ok(false) || !task_airborne()) {
                    RCLCPP_ERROR(this->get_logger(),
                        "A1.2 NAVIGATE: not confirmed IN_AIR or telemetry "
                        "became stale/too sparse for configured profile. "
                        "Setpoints stopped; ArduPilot "
                        "may retain last target. Manual intervention needed.");
                    change_state(MissionState::ERROR);
                    break;
                }
                const auto i = current_waypoint_index_.load();
                if (i >= mission_.size()) {
                    change_state(MissionState::DONE);
                    break;
                }
                if (enable_task_planned_flight_ && !task_route_ready(now)) {
                    break; 
                }
                const auto phase = heading_phase();
                if (phase == TaskHeadingPhase::PREPARE) {
                    if (!prepare_task_leg(i)) {
                        RCLCPP_ERROR(this->get_logger(),
                            "Cannot prepare heading: invalid/stale ENU pose.");
                        change_state(MissionState::ERROR);
                    }
                    break;
                }
                if (phase == TaskHeadingPhase::ALIGN) {
                    geometry_msgs::msg::PoseStamped pose;
                    geometry_msgs::msg::TwistStamped velocity;
                    if (!task_snapshot(pose, velocity)) {
                        change_state(MissionState::ERROR);
                        break;
                    }
                    if (task_heading_held(pose, velocity, now)) {
                        double yaw;
                        geometry_msgs::msg::Point hold;
                        {
                            std::lock_guard<std::mutex> lock(task_heading_mutex_);
                            yaw = task_heading_target_rad_;
                            hold = task_heading_hold_position_;
                        }
                        set_heading_phase(TaskHeadingPhase::FLY, yaw, hold);
                        RCLCPP_INFO(this->get_logger(),
                            "WP%zu: leg aligned; position travel begins.", i + 1);
                    }
                    break;
                }
                if (phase != TaskHeadingPhase::FLY) {
                    change_state(MissionState::ERROR);
                    break;
                }
                if (enable_task_planned_flight_) {
                    geometry_msgs::msg::Point route_target;
                    bool final_point;
                    std::size_t point_index, point_count;
                    if (!task_route_target(route_target, final_point,
                                           point_index, point_count)) {
                        change_state(MissionState::ERROR);
                        break;
                    }
                    if (!final_point) {
                        advance_task_route_point();
                        break;
                    }
                }
                if (process_current_waypoint()) {
                    position_controller_.reset_arrival();
                    const auto task = mission_[i].task;
                    if (task == TaskType::LAND ||
                        task == TaskType::LANDTAKEOFF) {
                        task_landtakeoff_ = task == TaskType::LANDTAKEOFF;
                        RCLCPP_INFO(this->get_logger(),
                            "WP%zu reached: entering TASK (%s).", i + 1,
                            task_landtakeoff_ ? "LANDTAKEOFF" : "LAND");
                        change_state(MissionState::TASK);
                    } else if (task == TaskType::YAW) {
                        if (!begin_waypoint_yaw(i)) {
                            RCLCPP_ERROR(this->get_logger(),
                                "Cannot begin yaw task: invalid ENU pose.");
                            change_state(MissionState::ERROR);
                        } else {
                            change_state(MissionState::TASK);
                        }
                    } else if (task == TaskType::NONE) {
                        reset_heading_for_next_leg();
                        current_waypoint_index_.store(i + 1);
                        reset_task_route();
                        if (i + 1 < mission_.size()) {
                            RCLCPP_INFO(this->get_logger(),
                                "WP%zu reached: advance to WP%zu.", i + 1, i + 2);
                        } else {
                            change_state(MissionState::DONE);
                        }
                    } else {
                        RCLCPP_ERROR(this->get_logger(),
                            "Unexpected task type during NAVIGATE.");
                        change_state(MissionState::ERROR);
                    }
                }
                break;
            }
            case MissionState::TASK:
            {
                const auto idx = current_waypoint_index_.load();
                if (idx >= mission_.size()) {
                    RCLCPP_ERROR(this->get_logger(), "TASK: invalid waypoint index.");
                    change_state(MissionState::ERROR);
                } else if (mission_[idx].task == TaskType::YAW) {
                    if (!armed || mode != "GUIDED" ||
                        !task_telemetry_ok(false) || !task_airborne()) {
                        RCLCPP_ERROR(this->get_logger(),
                            "YAW TASK aborted: arm/GUIDED or telemetry lost.");
                        change_state(MissionState::ERROR);
                        break;
                    }
                    geometry_msgs::msg::PoseStamped pose;
                    geometry_msgs::msg::TwistStamped velocity;
                    if (!task_snapshot(pose, velocity) ||
                        heading_phase() != TaskHeadingPhase::YAW_TASK) {
                        change_state(MissionState::ERROR);
                        break;
                    }
                    if (task_heading_held(pose, velocity, now)) {
                        RCLCPP_INFO(this->get_logger(),
                            "WP%zu YAW completed and held. Continue next leg.",
                            idx + 1);
                        reset_heading_for_next_leg();
                        current_waypoint_index_.store(idx + 1);
                        change_state(MissionState::NAVIGATE);
                    }
                } else if (mission_[idx].task == TaskType::LAND ||
                           mission_[idx].task == TaskType::LANDTAKEOFF) {
                    change_state(MissionState::LAND);
                } else {
                    RCLCPP_ERROR(this->get_logger(),
                        "TASK: unsupported task type.");
                    change_state(MissionState::ERROR);
                }
                break;
            }
            case MissionState::LAND:
            {
                task_land_tick(armed, now);
                break;
            }
            case MissionState::DONE:
            case MissionState::ERROR:
                break;
        }
    }

};

}

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