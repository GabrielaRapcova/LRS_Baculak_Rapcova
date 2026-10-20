#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "mission_executor/outdoor_geometry.hpp"

namespace mission_executor
{
constexpr double pi = 3.14159265358979323846;

TEST(OutdoorGeometry, GeographicRoundTrips)
{
    OutdoorReference reference({48.15, 17.07, 150.0}, {10.0, 20.0, 3.0}, 0.0);
    for (Point3 point : {Point3{0.0, 0.0, 0.0}, Point3{5.0, 5.0, 5.0},
            Point3{-50.0, 30.0, -2.0}}) {
        auto recovered = reference.convertGeodeticToMissionEnu(reference.convertMissionEnuToGeodetic(point));
        EXPECT_NEAR(recovered.x, point.x, 1e-7);
        EXPECT_NEAR(recovered.y, point.y, 1e-7);
        EXPECT_NEAR(recovered.z, point.z, 1e-7);
    }
}

TEST(OutdoorGeometry, TranslatedRotatedLocalFrame)
{
    OutdoorReference reference({48.15, 17.07, 150.0}, {10.0, 20.0, 3.0}, pi / 2.0);
    auto local = reference.convertMissionEnuToLocal({2.0, 3.0, 5.0});
    EXPECT_NEAR(local.x, 7.0, 1e-12);
    EXPECT_NEAR(local.y, 22.0, 1e-12);
    EXPECT_DOUBLE_EQ(local.z, 8.0);
    auto enu = reference.convertLocalToMissionEnu(local);
    EXPECT_NEAR(enu.x, 2.0, 1e-12);
    EXPECT_NEAR(enu.y, 3.0, 1e-12);
    EXPECT_DOUBLE_EQ(enu.z, 5.0);
}

TEST(OutdoorGeometry, HeadingAndWraparound)
{
    EXPECT_NEAR(convertHeadingDegreesToEnuYaw(0.0), pi / 2.0, 1e-12);
    EXPECT_NEAR(convertHeadingDegreesToEnuYaw(90.0), 0.0, 1e-12);
    EXPECT_NEAR(convertHeadingDegreesToEnuYaw(180.0), -pi / 2.0, 1e-12);
    EXPECT_NEAR(convertHeadingDegreesToEnuYaw(270.0), pi, 1e-12);
    EXPECT_NEAR(unwrapAngle(179.0 * pi / 180.0, -179.0 * pi / 180.0),
        181.0 * pi / 180.0, 1e-12);
    EXPECT_DOUBLE_EQ(calculatePoiFacingYaw({2.5, 2.5, 5.0}, {2.5, 2.5, 0.0}, 1.2, 0.2), 1.2);
    EXPECT_NEAR(calculatePoiFacingYaw({3.5, 2.5, 5.0}, {2.5, 2.5, 0.0}, 0.0, 0.2), pi, 1e-12);
}

TEST(OutdoorGeometry, AgreedSquareAndEnvelope)
{
    auto corners = calculateSquareCornerPositions({2.5, 2.5, 0.0}, 5.0, 8.0);
    EXPECT_DOUBLE_EQ(corners[0].x, 0.0);
    EXPECT_DOUBLE_EQ(corners[0].y, 5.0);
    EXPECT_DOUBLE_EQ(corners[1].x, 5.0);
    EXPECT_DOUBLE_EQ(corners[1].y, 5.0);
    EXPECT_DOUBLE_EQ(corners[3].x, 0.0);
    EXPECT_DOUBLE_EQ(corners[3].y, 0.0);
    EXPECT_DOUBLE_EQ(corners[4].x, corners[0].x);
    EXPECT_DOUBLE_EQ(corners[4].z, 8.0);
    EXPECT_NEAR(calculateMissionHorizontalDistanceBound({2.5, 2.5, 0.0}, {0.0, 0.0, 0.0}, corners, 3.0),
        std::sqrt(50.0), 1e-12);
    EXPECT_GT(calculateMissionHorizontalDistanceBound({2.5, 2.5, 0.0}, {-10.0, 0.0, 0.0}, corners, 3.0), 10.0);
}

TEST(OutdoorGeometry, GpsCornersAndActualEnvelope)
{
    OutdoorReference reference({48.15144444, 17.07375, 150.0}, {10.0, 20.0, 3.0}, 0.0);
    auto point = reference.convertGeodeticToMissionEnu({48.15148940567, 17.07381719675, 150.0});
    EXPECT_NEAR(point.x, 5.0, 0.001);
    EXPECT_NEAR(point.y, 5.0, 0.001);
    auto poi = reference.convertGeodeticToMissionEnu({48.15146692284, 17.07378359836, 150.0});
    EXPECT_NEAR(poi.x, 2.5, 0.001);
    EXPECT_NEAR(poi.y, 2.5, 0.001);
    auto corners = calculateSquareCornerPositions(poi, 5.0, 5.0);
    corners[1] = {20.0, 0.0, 5.0};
    EXPECT_DOUBLE_EQ(calculateMissionHorizontalDistanceBound(poi, {0.0, 0.0, 0.0}, corners, 3.0), 20.0);
}

TEST(OutdoorGeometry, FullFigureEightAndAnalyticVelocity)
{
    double omega = 0.2;
    auto start = calculateFigureEightSample({2.5, 2.5, 8.0}, 3.0, omega, 0.0);
    auto end = calculateFigureEightSample({2.5, 2.5, 8.0}, 3.0, omega, 2.0 * pi / omega);
    EXPECT_NEAR(start.yaw, pi / 4.0, 1e-12);
    EXPECT_NEAR(end.position.x, start.position.x, 1e-12);
    EXPECT_NEAR(end.position.y, start.position.y, 1e-12);
    EXPECT_DOUBLE_EQ(end.position.z, 8.0);
    EXPECT_NEAR(calculateFigureEightPeakSpeed(3.0, omega), 0.6 * std::sqrt(2.0), 1e-12);
    for (double time = 0.1; time < 31.0; time += 0.1) {
        auto sample = calculateFigureEightSample({2.5, 2.5, 8.0}, 3.0, omega, time);
        auto before = calculateFigureEightSample({2.5, 2.5, 8.0}, 3.0, omega, time - 1e-5);
        auto after = calculateFigureEightSample({2.5, 2.5, 8.0}, 3.0, omega, time + 1e-5);
        EXPECT_NEAR((after.position.x - before.position.x) / 2e-5, sample.velocity.x, 1e-8);
        EXPECT_NEAR((after.position.y - before.position.y) / 2e-5, sample.velocity.y, 1e-8);
        EXPECT_LE(std::hypot(sample.velocity.x, sample.velocity.y), calculateFigureEightPeakSpeed(3.0, omega));
    }
}

TEST(OutdoorGeometry, RejectsInvalidInputs)
{
    double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW((OutdoorReference({91.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0)), std::invalid_argument);
    EXPECT_THROW(wrapAngleToPi(nan), std::invalid_argument);
    EXPECT_THROW(calculateSquareCornerPositions({0.0, 0.0, 0.0}, -5.0, 5.0), std::invalid_argument);
    EXPECT_THROW(calculateFigureEightSample({0.0, 0.0, 0.0}, 3.0, 0.0, 1.0), std::invalid_argument);
    EXPECT_THROW(calculateFigureEightSample({0.0, 0.0, 0.0}, 3.0, 0.2, -1.0), std::invalid_argument);
}
}  // namespace mission_executor
