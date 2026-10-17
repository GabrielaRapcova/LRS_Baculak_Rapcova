#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "mission_executor/position_controller.hpp"
#include "mission_executor/mission_types.hpp"

using namespace mission_executor;
using namespace std::chrono_literals;

int main()
{
    // HARD: 0.15 m
    // SOFT: 0.50 m
    // Maximalna rychlost pri HARD: 0.15 m/s
    // Cas stabilizacie HARD: 1.0 s

    PositionController controller(
        0.15,
        0.50,
        0.15,
        1.0
    );

    MissionWaypoint waypoint{};

    waypoint.x = 13.0;
    waypoint.y = 7.0;
    waypoint.z = 1.0;

    // =============================================
    // TEST 1: Transformacia Gazebo -> ENU
    // =============================================

    const auto target =
        controller.gazebo_to_enu(waypoint);

    assert(std::abs(target.x + 7.0) < 1e-9);
    assert(std::abs(target.y - 13.0) < 1e-9);
    assert(std::abs(target.z - 1.0) < 1e-9);

    std::cout
        << "Transformation test passed!"
        << std::endl;

    // =============================================
    // TEST 2: Vzdialenost
    // =============================================

    geometry_msgs::msg::Point position;

    position.x = -7.0;
    position.y = 13.0;
    position.z = 0.0;

    const double distance =
        controller.calculate_distance(
            position,
            waypoint
        );

    assert(std::abs(distance - 1.0) < 1e-9);

    std::cout
        << "Distance test passed!"
        << std::endl;

    // =============================================
    // TEST 3: SOFT - BEZ ZASTAVENIA
    // =============================================

    position.z = 0.70;

    geometry_msgs::msg::Vector3 velocity;

    velocity.x = 0.40;
    velocity.y = 0.0;
    velocity.z = 0.0;

    const auto t0 =
        std::chrono::steady_clock::now();

    waypoint.precision = Precision::SOFT;

    auto status = controller.update_arrival(
        position,
        velocity,
        waypoint,
        t0
    );

    // Vzdialenost 0.30 m < 0.50 m.
    // Rychlost 0.40 m/s nevadi pri SOFT.
    assert(status.reached);

    std::cout
        << "SOFT pass-through test passed!"
        << std::endl;

    // =============================================
    // TEST 4: HARD - PRIVYSOKA RYCHLOST
    // =============================================

    waypoint.precision = Precision::HARD;

    // Polohu nastavime do HARD tolerancie.
    position.z = 0.90;

    status = controller.update_arrival(
        position,
        velocity,
        waypoint,
        t0
    );

    // Vzdialenost je 0.10 m, ale rychlost 0.40 m/s.
    assert(!status.reached);

    std::cout
        << "HARD velocity test passed!"
        << std::endl;

    // =============================================
    // TEST 5: HARD - STABILIZACIA
    // =============================================

    velocity.x = 0.04;

    // Zaciatok stabilizacie
    status = controller.update_arrival(
        position,
        velocity,
        waypoint,
        t0
    );

    assert(!status.reached);

    // Po 0.5 sekunde este nie
    status = controller.update_arrival(
        position,
        velocity,
        waypoint,
        t0 + 500ms
    );

    assert(!status.reached);

    // Po 1 sekunde ano
    status = controller.update_arrival(
        position,
        velocity,
        waypoint,
        t0 + 1000ms
    );

    assert(status.reached);
    assert(status.stable);

    std::cout
        << "HARD stabilization test passed!"
        << std::endl;

    // =============================================
    // TEST 6: RESET PRI ZVYSENI RYCHLOSTI
    // =============================================

    velocity.x = 0.30;

    status = controller.update_arrival(
        position,
        velocity,
        waypoint,
        t0 + 1100ms
    );

    assert(!status.reached);

    // Po spomaleni sa musi cas pocitat odznova.
    velocity.x = 0.02;

    status = controller.update_arrival(
        position,
        velocity,
        waypoint,
        t0 + 1200ms
    );

    assert(!status.reached);

    status = controller.update_arrival(
        position,
        velocity,
        waypoint,
        t0 + 2300ms
    );

    assert(status.reached);

    std::cout
        << "Stabilization reset test passed!"
        << std::endl;

    // =============================================
    // TEST 7: RESET PRI ZMENE WAYPOINTU
    // =============================================

    controller.reset_arrival();

    status = controller.update_arrival(
        position,
        velocity,
        waypoint,
        t0 + 2400ms
    );

    assert(!status.reached);

    std::cout
        << "Waypoint reset test passed!"
        << std::endl;

    // =============================================
    // TEST 8: NEPLATNE UDAJE
    // =============================================

    velocity.x =
        std::numeric_limits<double>::quiet_NaN();

    status = controller.update_arrival(
        position,
        velocity,
        waypoint,
        t0 + 2500ms
    );

    assert(!status.reached);

    std::cout
        << "Invalid input test passed!"
        << std::endl;

    std::cout
        << "\nAll position controller tests passed!"
        << std::endl;

    return 0;
}