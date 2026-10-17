#include "mission_executor/outdoor_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mission_executor
{
namespace
{
constexpr double pi = 3.14159265358979323846;

void validatePositiveValue(double value)
{
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Geometry dimensions and rates must be finite and positive");
    }
}
}  // namespace

bool isFinitePoint(Point3 point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool isValidGeodeticPosition(GeodeticPosition position)
{
    return std::isfinite(position.latitude) && std::abs(position.latitude) <= 90.0 &&
           std::isfinite(position.longitude) && std::abs(position.longitude) <= 180.0 &&
           std::isfinite(position.altitude_ellipsoid);
}

double wrapAngleToPi(double angle)
{
    if (!std::isfinite(angle)) {
        throw std::invalid_argument("Angle must be finite");
    }
    double wrapped = std::remainder(angle, 2.0 * pi);
    if (wrapped <= -pi) {
        wrapped = pi;
    }
    return wrapped;
}

double unwrapAngle(double previous, double next)
{
    return previous + wrapAngleToPi(next - previous);
}

double convertHeadingDegreesToEnuYaw(double heading_degrees)
{
    return wrapAngleToPi(pi / 2.0 - heading_degrees * pi / 180.0);
}

double calculatePoiFacingYaw(Point3 position, Point3 poi, double previous_yaw, double min_distance)
{
    validatePositiveValue(min_distance);
    if (!isFinitePoint(position) || !isFinitePoint(poi) || !std::isfinite(previous_yaw)) {
        throw std::invalid_argument("POI bearing requires finite inputs");
    }
    double east = poi.x - position.x;
    double north = poi.y - position.y;
    if (std::hypot(east, north) < min_distance) {
        return previous_yaw;
    }
    return unwrapAngle(previous_yaw, std::atan2(north, east));
}

std::array<Point3, 5> calculateSquareCornerPositions(Point3 poi, double side, double flight_height)
{
    validatePositiveValue(side);
    if (!isFinitePoint(poi) || !std::isfinite(flight_height)) {
        throw std::invalid_argument("Square position must be finite");
    }
    double half = side / 2.0;
    return {{{poi.x - half, poi.y + half, flight_height},
             {poi.x + half, poi.y + half, flight_height},
             {poi.x + half, poi.y - half, flight_height},
             {poi.x - half, poi.y - half, flight_height},
             {poi.x - half, poi.y + half, flight_height}}};
}

TrajectorySample calculateFigureEightSample(Point3 centre, double amplitude, double omega, double elapsed)
{
    validatePositiveValue(amplitude);
    validatePositiveValue(omega);
    if (!isFinitePoint(centre) || !std::isfinite(elapsed) || elapsed < 0.0) {
        throw std::invalid_argument("Trajectory requires a finite centre and nonnegative time");
    }
    double phase = omega * elapsed;
    if (!std::isfinite(phase)) {
        throw std::invalid_argument("Trajectory phase overflow");
    }
    TrajectorySample sample;
    sample.position = {
        centre.x + amplitude * std::sin(phase),
        centre.y + amplitude * std::sin(phase) * std::cos(phase),
        centre.z};
    sample.velocity = {
        amplitude * omega * std::cos(phase),
        amplitude * omega * std::cos(2.0 * phase),
        0.0};
    sample.yaw = std::atan2(sample.velocity.y, sample.velocity.x);
    return sample;
}

double calculateFigureEightPeakSpeed(double amplitude, double omega)
{
    validatePositiveValue(amplitude);
    validatePositiveValue(omega);
    return amplitude * omega * std::sqrt(2.0);
}

double calculateMissionHorizontalDistanceBound(Point3 poi, Point3 launch, double side, double amplitude)
{
    validatePositiveValue(amplitude);
    if (!isFinitePoint(launch)) {
        throw std::invalid_argument("Launch position must be finite");
    }
    auto corners = calculateSquareCornerPositions(poi, side, launch.z);
    double bound = std::hypot(
        std::abs(poi.x - launch.x) + amplitude,
        std::abs(poi.y - launch.y) + amplitude / 2.0);
    for (const auto & corner : corners) {
        bound = std::max(bound, std::hypot(corner.x - launch.x, corner.y - launch.y));
    }
    // All straight transfers lie inside the disk containing their endpoints.
    return bound;
}

OutdoorReference::OutdoorReference(
    GeodeticPosition origin, Point3 local_anchor, double yaw_offset)
    : local_anchor_(local_anchor), yaw_offset_(yaw_offset)
{
    if (!isValidGeodeticPosition(origin) || !isFinitePoint(local_anchor) || !std::isfinite(yaw_offset)) {
        throw std::invalid_argument("Reference must contain finite valid coordinates");
    }
    geographic_.Reset(origin.latitude, origin.longitude, origin.altitude_ellipsoid);
}

Point3 OutdoorReference::convertGeodeticToMissionEnu(GeodeticPosition position) const
{
    if (!isValidGeodeticPosition(position)) {
        throw std::invalid_argument("Invalid geodetic position");
    }
    Point3 result;
    geographic_.Forward(position.latitude, position.longitude, position.altitude_ellipsoid,
        result.x, result.y, result.z);
    return result;
}

GeodeticPosition OutdoorReference::convertMissionEnuToGeodetic(Point3 enu) const
{
    if (!isFinitePoint(enu)) {
        throw std::invalid_argument("ENU position must be finite");
    }
    GeodeticPosition result;
    geographic_.Reverse(enu.x, enu.y, enu.z,
        result.latitude, result.longitude, result.altitude_ellipsoid);
    return result;
}

Point3 OutdoorReference::convertMissionEnuToLocal(Point3 enu) const
{
    if (!isFinitePoint(enu)) {
        throw std::invalid_argument("ENU position must be finite");
    }
    double cosine = std::cos(yaw_offset_);
    double sine = std::sin(yaw_offset_);
    return {local_anchor_.x + cosine * enu.x - sine * enu.y,
            local_anchor_.y + sine * enu.x + cosine * enu.y,
            local_anchor_.z + enu.z};
}

Point3 OutdoorReference::convertLocalToMissionEnu(Point3 local) const
{
    if (!isFinitePoint(local)) {
        throw std::invalid_argument("Local position must be finite");
    }
    double east = local.x - local_anchor_.x;
    double north = local.y - local_anchor_.y;
    double cosine = std::cos(yaw_offset_);
    double sine = std::sin(yaw_offset_);
    return {cosine * east + sine * north, -sine * east + cosine * north,
            local.z - local_anchor_.z};
}

}  // namespace mission_executor
