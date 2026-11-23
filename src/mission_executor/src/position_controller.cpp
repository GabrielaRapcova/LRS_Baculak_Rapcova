#include "mission_executor/position_controller.hpp"

#include <cmath>
#include <stdexcept>

namespace mission_executor
{

PositionController::PositionController(
    double hard_tolerance,
    double soft_tolerance,
    double hard_speed_limit,
    double hard_stable_time
)
    : hard_tolerance_(hard_tolerance),
      soft_tolerance_(soft_tolerance),
      hard_speed_limit_(hard_speed_limit),
      hard_stable_time_(hard_stable_time)
{
    if (!std::isfinite(hard_tolerance_) ||
        !std::isfinite(soft_tolerance_) ||
        !std::isfinite(hard_speed_limit_) ||
        !std::isfinite(hard_stable_time_) ||
        hard_tolerance_ <= 0.0 ||
        soft_tolerance_ <= 0.0 ||
        hard_speed_limit_ <= 0.0 ||
        hard_stable_time_ <= 0.0)
    {
        throw std::invalid_argument(
            "Position controller parameters must "
            "be finite and positive."
        );
    }

    if (hard_tolerance_ > soft_tolerance_)
    {
        throw std::invalid_argument(
            "Hard tolerance must not exceed soft tolerance."
        );
    }
}

geometry_msgs::msg::Point PositionController::gazebo_to_enu(
    const MissionWaypoint &waypoint
) const
{
    geometry_msgs::msg::Point target;

    target.x = -waypoint.y;
    target.y = waypoint.x;
    target.z = waypoint.z;

    return target;
}

double PositionController::calculate_distance(
    const geometry_msgs::msg::Point &current_position,
    const MissionWaypoint &waypoint
) const
{
    const auto target = gazebo_to_enu(waypoint);

    const double dx =
        target.x - current_position.x;

    const double dy =
        target.y - current_position.y;

    const double dz =
        target.z - current_position.z;

    return std::sqrt(
        dx * dx +
        dy * dy +
        dz * dz
    );
}

bool PositionController::is_waypoint_reached(
    const geometry_msgs::msg::Point &current_position,
    const MissionWaypoint &waypoint
) const
{
    const double distance =
        calculate_distance(current_position, waypoint);

    if (!std::isfinite(distance))
    {
        return false;
    }

    switch (waypoint.precision)
    {
        case Precision::HARD:
            return distance <= hard_tolerance_;

        case Precision::SOFT:
            return distance <= soft_tolerance_;
    }

    return false;
}

void PositionController::reset_arrival()
{
    hard_timer_running_ = false;

    hard_stable_since_ =
        std::chrono::steady_clock::time_point{};
}

ArrivalStatus PositionController::update_arrival(
    const geometry_msgs::msg::Point &current_position,
    const geometry_msgs::msg::Vector3 &current_velocity,
    const MissionWaypoint &waypoint,
    std::chrono::steady_clock::time_point now
)
{
    ArrivalStatus status;

    if (!std::isfinite(current_position.x) ||
        !std::isfinite(current_position.y) ||
        !std::isfinite(current_position.z) ||
        !std::isfinite(current_velocity.x) ||
        !std::isfinite(current_velocity.y) ||
        !std::isfinite(current_velocity.z) ||
        !std::isfinite(waypoint.x) ||
        !std::isfinite(waypoint.y) ||
        !std::isfinite(waypoint.z))
    {
        reset_arrival();
        return status;
    }

    status.distance_m =
        calculate_distance(
            current_position,
            waypoint
        );

    status.speed_mps = std::sqrt(
        current_velocity.x * current_velocity.x +
        current_velocity.y * current_velocity.y +
        current_velocity.z * current_velocity.z
    );

    if (!std::isfinite(status.distance_m) ||
        !std::isfinite(status.speed_mps))
    {
        reset_arrival();
        return status;
    }

    if (waypoint.precision == Precision::SOFT)
    {
        reset_arrival();

        status.within_position_tolerance =
            status.distance_m <= soft_tolerance_;

        status.reached =
            status.within_position_tolerance;

        return status;
    }

    if (waypoint.precision == Precision::HARD)
    {
        const bool position_ok =
            status.distance_m <= hard_tolerance_;

        const bool velocity_ok =
            status.speed_mps <= hard_speed_limit_;

        status.within_position_tolerance = position_ok;

        if (!position_ok || !velocity_ok)
        {
            reset_arrival();
            return status;
        }

        if (!hard_timer_running_ ||
            now < hard_stable_since_)
        {
            hard_timer_running_ = true;
            hard_stable_since_ = now;
        }

        status.stable_time_s =
            std::chrono::duration<double>(
                now - hard_stable_since_
            ).count();

        if (status.stable_time_s >= hard_stable_time_)
        {
            status.stable = true;
            status.reached = true;
        }

        return status;
    }

    reset_arrival();
    return status;
}

}