#ifndef MISSION_EXECUTOR__OUTDOOR_GEOMETRY_HPP_
#define MISSION_EXECUTOR__OUTDOOR_GEOMETRY_HPP_

#include <array>
#include <GeographicLib/LocalCartesian.hpp>

namespace mission_executor
{

struct Point3
{
    double x;
    double y;
    double z;
};

struct GeodeticPosition
{
    double latitude;
    double longitude;
    double altitude_ellipsoid;
};

struct TrajectorySample
{
    Point3 position;
    Point3 velocity;
    double yaw;
};

bool isFinitePoint(Point3 point);
bool isValidGeodeticPosition(GeodeticPosition position);
double wrapAngleToPi(double angle);
double unwrapAngle(double previous, double next);
double convertHeadingDegreesToEnuYaw(double heading_degrees);
double calculatePoiFacingYaw(Point3 position, Point3 poi, double previous_yaw, double min_distance);
std::array<Point3, 5> calculateSquareCornerPositions(Point3 poi, double side, double flight_height);
TrajectorySample calculateFigureEightSample(Point3 centre, double amplitude, double omega, double elapsed);
double calculateFigureEightPeakSpeed(double amplitude, double omega);
double calculateMissionHorizontalDistanceBound(Point3 poi, Point3 launch, double side, double amplitude);

// Mission ENU is anchored at the geodetic origin. local_anchor is that same
// point in MAVROS local coordinates; yaw_offset rotates ENU axes into local axes.
class OutdoorReference
{
public:
    OutdoorReference(GeodeticPosition origin, Point3 local_anchor, double yaw_offset);
    Point3 convertGeodeticToMissionEnu(GeodeticPosition position) const;
    GeodeticPosition convertMissionEnuToGeodetic(Point3 enu) const;
    Point3 convertMissionEnuToLocal(Point3 enu) const;
    Point3 convertLocalToMissionEnu(Point3 local) const;

private:
    GeographicLib::LocalCartesian geographic_;
    Point3 local_anchor_;
    double yaw_offset_;
};

}  // namespace mission_executor

#endif
