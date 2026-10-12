#ifndef MISSION_EXECUTOR__MISSION_TYPES_HPP_
#define MISSION_EXECUTOR__MISSION_TYPES_HPP_

#include <vector>

namespace mission_executor
{

enum class Precision
{
    SOFT,
    HARD
};

enum class TaskType
{
    NONE,
    TAKEOFF,
    LAND,
    LANDTAKEOFF,
    YAW
};

struct MissionWaypoint
{
    double x;
    double y;
    double z;

    Precision precision;
    TaskType task;

    double yaw_angle = 0.0;
};

using Mission = std::vector<MissionWaypoint>;

}  // namespace mission_executor

#endif