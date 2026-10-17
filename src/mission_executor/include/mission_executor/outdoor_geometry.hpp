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

bool is_finite(Point3 point);
bool is_valid(GeodeticPosition position);
double wrap_angle(double angle);
double unwrap_angle(double previous, double next);
double heading_to_yaw(double heading_degrees);
double poi_yaw(Point3 position, Point3 poi, double previous_yaw, double min_distance);
std::array<Point3, 5> square_corners(Point3 poi, double side, double flight_height);
TrajectorySample figure_eight(Point3 centre, double amplitude, double omega, double elapsed);
double figure_eight_peak_speed(double amplitude, double omega);
double mission_horizontal_bound(Point3 poi, Point3 launch, double side, double amplitude);

// Mission ENU is anchored at the geodetic origin. local_anchor is that same
// point in MAVROS local coordinates; yaw_offset rotates ENU axes into local axes.
class OutdoorReference
{
public:
    OutdoorReference(GeodeticPosition origin, Point3 local_anchor, double yaw_offset);
    Point3 to_enu(GeodeticPosition position) const;
    GeodeticPosition to_geodetic(Point3 enu) const;
    Point3 to_local(Point3 enu) const;
    Point3 from_local(Point3 local) const;

private:
    GeographicLib::LocalCartesian geographic_;
    Point3 local_anchor_;
    double yaw_offset_;
};

}  // namespace mission_executor

#endif
