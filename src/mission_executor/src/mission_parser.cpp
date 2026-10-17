#include "mission_executor/mission_parser.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace mission_executor
{

static std::string trim(const std::string & text)
{
    const auto first = text.find_first_not_of(" \t\r\n");

    if (first == std::string::npos) {
        return "";
    }

    const auto last = text.find_last_not_of(" \t\r\n");

    return text.substr(first, last - first + 1);
}

static std::vector<std::string> split_csv(const std::string & line)
{
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;

    while (std::getline(ss, field, ',')) {
        fields.push_back(trim(field));
    }

    if (!line.empty() && line.back() == ',') {
        fields.push_back("");
    }

    return fields;
}

static double parse_number(
    const std::string & text,
    std::size_t line_number)
{
    try {
        std::size_t processed = 0;

        double value = std::stod(text, &processed);

        if (processed != text.size() || !std::isfinite(value)) {
            throw std::invalid_argument("Invalid number");
        }

        return value;
    }
    catch (const std::exception &) {
        throw std::runtime_error(
            "Invalid numeric value on line " +
            std::to_string(line_number) + ": " + text
        );
    }
}

static Precision parse_precision(
    const std::string & text,
    std::size_t line_number)
{
    if (text == "soft") {
        return Precision::SOFT;
    }

    if (text == "hard") {
        return Precision::HARD;
    }

    throw std::runtime_error(
        "Invalid precision on line " +
        std::to_string(line_number) + ": " + text
    );
}

static void parse_task(
    const std::string & text,
    MissionWaypoint & waypoint,
    std::size_t line_number)
{
    if (text == "-") {
        waypoint.task = TaskType::NONE;
        return;
    }

    if (text == "takeoff") {
        waypoint.task = TaskType::TAKEOFF;
        return;
    }

    if (text == "land") {
        waypoint.task = TaskType::LAND;
        return;
    }

    if (text == "landtakeoff") {
        waypoint.task = TaskType::LANDTAKEOFF;
        return;
    }

    if (text.rfind("yaw", 0) == 0) {
        waypoint.task = TaskType::YAW;

        const std::string angle = text.substr(3);

        if (angle.empty()) {
            throw std::runtime_error(
                "Missing yaw angle on line " +
                std::to_string(line_number)
            );
        }

        waypoint.yaw_angle = parse_number(angle, line_number);
        return;
    }

    throw std::runtime_error(
        "Unknown task on line " +
        std::to_string(line_number) + ": " + text
    );
}

Mission MissionParser::load_from_csv(const std::string & file_path)
{
    std::ifstream file(file_path);

    if (!file.is_open()) {
        throw std::runtime_error(
            "Cannot open mission file: " + file_path
        );
    }

    Mission mission;

    std::string line;
    std::size_t line_number = 0;
    bool header_found = false;

    while (std::getline(file, line)) {

        ++line_number;

        line = trim(line);

        if (line.empty() || line[0] == '#') {
            continue;
        }

         const auto fields = split_csv(line);

        if (!header_found) {

            const std::vector<std::string> expected_header = {
                "x", "y", "z", "precision", "task"
            };

            if (fields != expected_header) {
                throw std::runtime_error(
                    "Invalid CSV header on line " +
                    std::to_string(line_number)
                );
            }

            header_found = true;
            continue;
        }

        if (fields.size() != 5) {
            throw std::runtime_error(
                "Expected 5 columns on line " +
                std::to_string(line_number)
            );
        }

        MissionWaypoint waypoint{};

        waypoint.x = parse_number(fields[0], line_number);
        waypoint.y = parse_number(fields[1], line_number);
        waypoint.z = parse_number(fields[2], line_number);

        waypoint.precision = parse_precision(
            fields[3], line_number
        );

        parse_task(fields[4], waypoint, line_number);

        mission.push_back(waypoint);
    }

    if (!header_found) {
        throw std::runtime_error("Mission CSV header not found");
    }

    if (mission.empty()) {
        throw std::runtime_error("Mission file contains no waypoints");
    }

    return mission;
}

}