#ifndef MISSION_EXECUTOR__MISSION_PARSER_HPP_
#define MISSION_EXECUTOR__MISSION_PARSER_HPP_

#include <string>

#include "mission_executor/mission_types.hpp"

namespace mission_executor
{

class MissionParser
{
public:
    static Mission load_from_csv(const std::string & file_path);
};

}  // namespace mission_executor

#endif