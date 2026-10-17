#ifndef MISSION_EXECUTOR__MISSION_STATE_HPP_
#define MISSION_EXECUTOR__MISSION_STATE_HPP_

namespace mission_executor
{

enum class MissionState
{
    IDLE,
    WAIT_CONNECTION,
    SET_MODE,
    ARM,
    TAKEOFF,
    NAVIGATE,
    TASK,
    LAND,
    DONE,
    ERROR
};

inline const char * state_to_string(MissionState state)
{
    switch (state)
    {
        case MissionState::IDLE:
            return "IDLE";

        case MissionState::WAIT_CONNECTION:
            return "WAIT_CONNECTION";

        case MissionState::SET_MODE:
            return "SET_MODE";

        case MissionState::ARM:
            return "ARM";

        case MissionState::TAKEOFF:
            return "TAKEOFF";

        case MissionState::NAVIGATE:
            return "NAVIGATE";

        case MissionState::TASK:
            return "TASK";

        case MissionState::LAND:
            return "LAND";

        case MissionState::DONE:
            return "DONE";

        case MissionState::ERROR:
            return "ERROR";
    }

    return "UNKNOWN";
}

}  // namespace mission_executor

#endif