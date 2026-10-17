#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"

#include "mission_executor/mission_parser.hpp"
#include "mission_executor/mission_types.hpp"
#include "mission_executor/mission_state.hpp"

using namespace std::chrono_literals;

namespace mission_executor
{

class MissionExecutorNode : public rclcpp::Node
{
public:
    MissionExecutorNode()
        : Node("mission_executor_node"),
          current_state_(MissionState::IDLE)
    {
        this->declare_parameter<std::string>("mission_file", "");
        this->declare_parameter<bool>("enable_arming", false);
        this->declare_parameter<bool>("enable_mavros_setpoints", false);

        enable_arming_ =
            this->get_parameter("enable_arming").as_bool();

        enable_mavros_setpoints_ =
            this->get_parameter("enable_mavros_setpoints").as_bool();

        
        if (enable_arming_ && enable_mavros_setpoints_) {
            throw std::runtime_error(
                "enable_arming and enable_mavros_setpoints "
                "cannot both be true in this test version."
            );
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Automatic arming: %s",
            enable_arming_ ? "ENABLED" : "DISABLED"
        );

        // Nacitanie misie
        const std::string mission_file =
            this->get_parameter("mission_file").as_string();

        if (mission_file.empty()) {
            throw std::runtime_error(
                "Parameter 'mission_file' is empty."
            );
        }

        mission_ = MissionParser::load_from_csv(mission_file);

        RCLCPP_INFO(
            this->get_logger(),
            "Mission loaded: %zu waypoints",
            mission_.size()
        );

        // Callback groups
        state_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );

        setpoint_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );

        pose_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );

        // Subscriber na MAVROS state
        rclcpp::SubscriptionOptions state_options;
        state_options.callback_group = state_group_;

        mavros_state_sub_ =
            this->create_subscription<mavros_msgs::msg::State>(
                "/mavros/state",
                10,
                std::bind(
                    &MissionExecutorNode::mavros_state_callback,
                    this,
                    std::placeholders::_1
                ),
                state_options
            );

        // Subscriber na aktualnu polohu
        rclcpp::SubscriptionOptions pose_options;
        pose_options.callback_group = pose_group_;

        pose_sub_ =
            this->create_subscription<
                geometry_msgs::msg::PoseStamped
            >(
                "/mavros/local_position/pose",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &MissionExecutorNode::pose_callback,
                    this,
                    std::placeholders::_1
                ),
                pose_options
            );

        // MAVROS sluzby
        set_mode_client_ =
            this->create_client<mavros_msgs::srv::SetMode>(
                "/mavros/set_mode",
                rclcpp::ServicesQoS(),
                state_group_
            );

        arming_client_ =
            this->create_client<mavros_msgs::srv::CommandBool>(
                "/mavros/cmd/arming",
                rclcpp::ServicesQoS(),
                state_group_
            );

        // Diagnosticky publisher
        debug_setpoint_pub_ =
            this->create_publisher<
                mavros_msgs::msg::PositionTarget
            >(
                "/mission_executor/debug_setpoint",
                10
            );

        // Publisher skutocnych MAVROS setpointov
        mavros_setpoint_pub_ =
            this->create_publisher<
                mavros_msgs::msg::PositionTarget
            >(
                "/mavros/setpoint_raw/local",
                10
            );

        //  Timer stavoveho automatu - 10 Hz
        state_timer_ = this->create_wall_timer(
            100ms,
            std::bind(
                &MissionExecutorNode::state_machine_callback,
                this
            ),
            state_group_
        );

        // Timer setpointov - 20 Hz
        setpoint_timer_ = this->create_wall_timer(
            50ms,
            std::bind(
                &MissionExecutorNode::publish_setpoint,
                this
            ),
            setpoint_group_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Callback groups initialized: state, setpoint, pose."
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Debug setpoint publisher initialized at 20 Hz."
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Real MAVROS setpoint test: %s (disarmed only)",
            enable_mavros_setpoints_ ? "ENABLED" : "DISABLED"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Mission executor initialized."
        );
    }

private:
    
    Mission mission_;
    MissionState current_state_;

    bool enable_arming_ = false;
    bool enable_mavros_setpoints_ = false;

    
    std::mutex mavros_mutex_;

    bool mavros_connected_ = false;
    bool mavros_armed_ = false;
    bool mavros_state_received_ = false;

    bool arming_disabled_logged_ = false;

    std::string mavros_mode_;

    std::chrono::steady_clock::time_point
        last_mavros_state_time_;

    //diagnosticke timeouty
    static constexpr auto MAVROS_TIMEOUT = 10s;
    static constexpr auto MODE_TIMEOUT = 10s;
    static constexpr auto ARM_TIMEOUT = 12s;
    static constexpr auto POSE_TIMEOUT = 2500ms;

    
    std::mutex pose_mutex_;

    geometry_msgs::msg::PoseStamped current_pose_;

    bool pose_received_ = false;

    std::chrono::steady_clock::time_point last_pose_time_;

    
    bool hold_initialized_ = false;

    geometry_msgs::msg::Point hold_position_;

    float hold_yaw_ = 0.0f;

    
    rclcpp::CallbackGroup::SharedPtr state_group_;
    rclcpp::CallbackGroup::SharedPtr setpoint_group_;
    rclcpp::CallbackGroup::SharedPtr pose_group_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr
        mavros_state_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped
    >::SharedPtr pose_sub_;

    rclcpp::Publisher<
        mavros_msgs::msg::PositionTarget
    >::SharedPtr debug_setpoint_pub_;

    rclcpp::Publisher<
        mavros_msgs::msg::PositionTarget
    >::SharedPtr mavros_setpoint_pub_;

    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr setpoint_timer_;

   
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr
        set_mode_client_;

    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr
        arming_client_;

    bool mode_request_sent_ = false;
    bool mode_response_received_ = false;
    bool mode_request_accepted_ = false;

    std::chrono::steady_clock::time_point mode_request_time_;

    bool arm_request_sent_ = false;
    bool arm_response_received_ = false;
    bool arm_request_accepted_ = false;

    std::chrono::steady_clock::time_point arm_request_time_;

    void mavros_state_callback(
        const mavros_msgs::msg::State::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mavros_mutex_);

        mavros_state_received_ = true;
        mavros_connected_ = msg->connected;
        mavros_armed_ = msg->armed;
        mavros_mode_ = msg->mode;

        last_mavros_state_time_ =
            std::chrono::steady_clock::now();
    }

    
    void pose_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);

            current_pose_ = *msg;
            pose_received_ = true;

            last_pose_time_ =
                std::chrono::steady_clock::now();
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Current position: x=%.2f, y=%.2f, z=%.2f",
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        );
    }

    void change_state(MissionState new_state)
    {
        if (current_state_ == new_state) {
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "State transition: %s -> %s",
            state_to_string(current_state_),
            state_to_string(new_state)
        );

        current_state_ = new_state;
    }

    mavros_msgs::msg::PositionTarget create_position_target()
    {
        mavros_msgs::msg::PositionTarget msg;

        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        msg.coordinate_frame =
            mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

        // Aktivne: poloha a yaw.
        // Ignorovane: rychlost, zrychlenie, yaw_rate.
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
        auto msg = create_position_target();

        if (hold_initialized_) {
            msg.position = hold_position_;
            msg.yaw = hold_yaw_;
        }

        debug_setpoint_pub_->publish(msg);

        // Skutocne setpointy su predvolene vypnute.
        if (!enable_mavros_setpoints_) {
            return;
        }

        bool connected = false;
        bool armed = false;
        bool state_fresh = false;

        {
            std::lock_guard<std::mutex> lock(mavros_mutex_);

            connected =
                mavros_state_received_ && mavros_connected_;

            armed = mavros_armed_;

            state_fresh =
                mavros_state_received_ &&
                (std::chrono::steady_clock::now() -
                 last_mavros_state_time_ <= MAVROS_TIMEOUT);
        }

        if (!connected || !state_fresh || armed) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Real setpoint paused: disconnected, "
                "stale MAVROS state, or armed."
            );

            return;
        }

        geometry_msgs::msg::PoseStamped pose_snapshot;

        {
            std::lock_guard<std::mutex> lock(pose_mutex_);

            if (!pose_received_ ||
                std::chrono::steady_clock::now() -
                    last_pose_time_ > POSE_TIMEOUT)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "Real setpoint paused: missing or stale pose."
                );

                return;
            }

            pose_snapshot = current_pose_;
        }

        
        if (!hold_initialized_) {
            const auto &p = pose_snapshot.pose.position;
            const auto &q = pose_snapshot.pose.orientation;

            // Kontrola platnych cisel
            if (!std::isfinite(p.x) ||
                !std::isfinite(p.y) ||
                !std::isfinite(p.z) ||
                !std::isfinite(q.x) ||
                !std::isfinite(q.y) ||
                !std::isfinite(q.z) ||
                !std::isfinite(q.w))
            {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "Cannot initialize hold: invalid pose values."
                );

                return;
            }

            // Kontrola platneho quaternionu
            const double q_norm_sq =
                q.x * q.x + q.y * q.y +
                q.z * q.z + q.w * q.w;

            if (q_norm_sq < 1e-12) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "Cannot initialize hold: invalid orientation."
                );

                return;
            }

            hold_position_ = p;

            // Quaternion -> yaw (ROS ENU)
            hold_yaw_ = static_cast<float>(
                std::atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
            );

            hold_initialized_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "Position hold initialized: "
                "x=%.2f y=%.2f z=%.2f yaw=%.2f",
                hold_position_.x,
                hold_position_.y,
                hold_position_.z,
                static_cast<double>(hold_yaw_)
            );
        }

        // ROS hodnoty su ENU.
        // MAVROS ich interne prevedie do MAVLink NED.
        msg.position = hold_position_;
        msg.yaw = hold_yaw_;

        mavros_setpoint_pub_->publish(msg);
    }

   
    void request_guided_mode()
    {
        if (!set_mode_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for /mavros/set_mode service..."
            );

            return;
        }

        auto request =
            std::make_shared<mavros_msgs::srv::SetMode::Request>();

        request->base_mode = 0;
        request->custom_mode = "GUIDED";

        mode_request_sent_ = true;
        mode_response_received_ = false;
        mode_request_accepted_ = false;

        mode_request_time_ =
            std::chrono::steady_clock::now();

        RCLCPP_INFO(
            this->get_logger(),
            "Requesting GUIDED mode..."
        );

        set_mode_client_->async_send_request(
            request,
            [this](
                rclcpp::Client<
                    mavros_msgs::srv::SetMode
                >::SharedFuture future)
            {
                try {
                    const auto response = future.get();

                    mode_request_accepted_ = response->mode_sent;
                    mode_response_received_ = true;

                    if (response->mode_sent) {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "GUIDED mode request sent successfully."
                        );
                    } else {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "MAVROS rejected GUIDED mode request."
                        );
                    }
                }
                catch (const std::exception &e) {
                    mode_request_accepted_ = false;
                    mode_response_received_ = true;

                    RCLCPP_ERROR(
                        this->get_logger(),
                        "SET_MODE service error: %s",
                        e.what()
                    );
                }
            }
        );
    }

    
    void request_arm()
    {
        if (!arming_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for /mavros/cmd/arming service..."
            );

            return;
        }

        auto request =
            std::make_shared<mavros_msgs::srv::CommandBool::Request>();

        request->value = true;

        arm_request_sent_ = true;
        arm_response_received_ = false;
        arm_request_accepted_ = false;

        arm_request_time_ =
            std::chrono::steady_clock::now();

        RCLCPP_INFO(
            this->get_logger(),
            "Requesting ARM..."
        );

        arming_client_->async_send_request(
            request,
            [this](
                rclcpp::Client<
                    mavros_msgs::srv::CommandBool
                >::SharedFuture future)
            {
                try {
                    const auto response = future.get();

                    arm_request_accepted_ = response->success;
                    arm_response_received_ = true;

                    if (response->success) {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "ARM request accepted. Result: %u",
                            static_cast<unsigned>(
                                response->result
                            )
                        );
                    } else {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "ARM request rejected. Result: %u",
                            static_cast<unsigned>(
                                response->result
                            )
                        );
                    }
                }
                catch (const std::exception &e) {
                    arm_request_accepted_ = false;
                    arm_response_received_ = true;

                    RCLCPP_ERROR(
                        this->get_logger(),
                        "ARM service error: %s",
                        e.what()
                    );
                }
            }
        );
    }

    void state_machine_callback()
    {
        // 1. Kontrola MAVROS spojenia
        if (mavros_state_received_ &&
            current_state_ != MissionState::IDLE &&
            current_state_ != MissionState::WAIT_CONNECTION &&
            current_state_ != MissionState::ERROR &&
            current_state_ != MissionState::DONE)
        {
            const auto elapsed =
                std::chrono::steady_clock::now() -
                last_mavros_state_time_;

            if (!mavros_connected_) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "MAVROS reports disconnected!"
                );

                change_state(MissionState::ERROR);
                return;
            }

            if (elapsed > MAVROS_TIMEOUT) {
                const double elapsed_seconds =
                    std::chrono::duration<double>(
                        elapsed
                    ).count();

                RCLCPP_ERROR(
                    this->get_logger(),
                    "MAVROS state timeout: "
                    "no message for %.2f seconds!",
                    elapsed_seconds
                );

                change_state(MissionState::ERROR);
                return;
            }
        }

        switch (current_state_)
        {
 
            case MissionState::IDLE:
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Mission ready. Waiting for MAVROS."
                );

                change_state(MissionState::WAIT_CONNECTION);
                break;
            }

            case MissionState::WAIT_CONNECTION:
            {
                const bool state_is_fresh =
                    mavros_state_received_ &&
                    (std::chrono::steady_clock::now() -
                     last_mavros_state_time_ <= MAVROS_TIMEOUT);

                if (state_is_fresh && mavros_connected_) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "MAVROS connected."
                    );

                    change_state(MissionState::SET_MODE);
                }

                break;
            }

            case MissionState::SET_MODE:
            {
                if (mavros_mode_ == "GUIDED") {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "GUIDED mode confirmed."
                    );

                    change_state(MissionState::ARM);
                    break;
                }

                if (!mode_request_sent_) {
                    request_guided_mode();
                    break;
                }

                if (mode_response_received_ &&
                    !mode_request_accepted_)
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Failed to request GUIDED mode."
                    );

                    change_state(MissionState::ERROR);
                    break;
                }

                const auto elapsed =
                    std::chrono::steady_clock::now() -
                    mode_request_time_;

                if (elapsed > MODE_TIMEOUT) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Timeout waiting for GUIDED mode confirmation."
                    );

                    change_state(MissionState::ERROR);
                }

                break;
            }

            case MissionState::ARM:
            {
                if (!enable_arming_) {
                    if (!arming_disabled_logged_) {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "Automatic arming disabled. "
                            "Waiting in ARM state."
                        );

                        arming_disabled_logged_ = true;
                    }

                    break;
                }

                if (mavros_mode_ != "GUIDED") {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Cannot arm: GUIDED mode is not active."
                    );

                    change_state(MissionState::ERROR);
                    break;
                }

                if (mavros_armed_) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Drone armed successfully."
                    );

                    change_state(MissionState::TAKEOFF);
                    break;
                }

                if (!arm_request_sent_) {
                    request_arm();
                    break;
                }

                if (arm_response_received_ &&
                    !arm_request_accepted_)
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Failed to arm the drone."
                    );

                    change_state(MissionState::ERROR);
                    break;
                }

                const auto elapsed =
                    std::chrono::steady_clock::now() -
                    arm_request_time_;

                if (elapsed > ARM_TIMEOUT) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Timeout waiting for ARM confirmation."
                    );

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
                break;
            }
        }
    }
};

}  

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node =
            std::make_shared<
                mission_executor::MissionExecutorNode
            >();

        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(),
            3
        );

        executor.add_node(node);

        RCLCPP_INFO(
            node->get_logger(),
            "MultiThreadedExecutor started with 3 threads."
        );

        executor.spin();
        executor.remove_node(node);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(
            rclcpp::get_logger("mission_executor"),
            "Mission executor failed: %s",
            e.what()
        );

        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
