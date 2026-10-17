#ifndef MISSION_EXECUTOR_POSITION_CONTROLLER_HPP
#define MISSION_EXECUTOR_POSITION_CONTROLLER_HPP

#include <chrono>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "mission_executor/mission_types.hpp"

namespace mission_executor
{

// Informacie o dosiahnuti waypointu
struct ArrivalStatus
{
    double distance_m = 0.0;
    double speed_mps = 0.0;
    double stable_time_s = 0.0;

    bool within_position_tolerance = false;
    bool stable = false;
    bool reached = false;
};

class PositionController
{
public:
    PositionController(
        double hard_tolerance,
        double soft_tolerance,
        double hard_speed_limit = 0.15,
        double hard_stable_time = 1.0
    );

    // Transformacia Gazebo -> ROS ENU
    geometry_msgs::msg::Point gazebo_to_enu(
        const MissionWaypoint &waypoint
    ) const;

    // Vypocet vzdialenosti od waypointu
    double calculate_distance(
        const geometry_msgs::msg::Point &current_position,
        const MissionWaypoint &waypoint
    ) const;

    // Povodna geometricka kontrola tolerancie.
    // Pri HARD nezarucuje stabilizaciu!
    bool is_waypoint_reached(
        const geometry_msgs::msg::Point &current_position,
        const MissionWaypoint &waypoint
    ) const;

    // NOVINKA:
    // Kompletne vyhodnotenie HARD / SOFT
    ArrivalStatus update_arrival(
        const geometry_msgs::msg::Point &current_position,
        const geometry_msgs::msg::Vector3 &current_velocity,
        const MissionWaypoint &waypoint,
        std::chrono::steady_clock::time_point now
    );

    // Reset pri prechode na dalsi waypoint
    void reset_arrival();

private:
    double hard_tolerance_;
    double soft_tolerance_;

    double hard_speed_limit_;
    double hard_stable_time_;

    bool hard_timer_running_ = false;

    std::chrono::steady_clock::time_point
        hard_stable_since_;
};

}  // namespace mission_executor

#endif