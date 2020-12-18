/*!
 * \file geographic.cpp
 *
 * \author Han
 * \date 2017/05/09
 *
 * 坐标转换
 */

#include "geographic.h"

#include <GeographicLib/Geocentric.hpp>
#include <base/base.h>

namespace h2o {
using namespace GeographicLib;
std::vector<Vector3d> latlon_to_geocentric(const std::vector<Vector3d> &latlon, const std::string &planet) {
    double a = Constants::WGS84_a(), f = Constants::WGS84_f();
    if (string_to_lower(planet) == "moon") {
        a = MOON_RADIUS;
        f = 0.0;
    } else if (string_to_lower(planet) == "mars") {
        a = MARS_RADIUS;
        f = 0.0;
    } else if (string_to_lower(planet) == "earth") {
        a = Constants::WGS84_a();
        f = Constants::WGS84_f();
    }
    return latlon_to_geocentric(latlon, a, f);
}

std::vector<Vector3d> latlon_to_geocentric(const std::vector<Vector3d> &latlon, double a, double f) {
    std::shared_ptr<Geocentric> geocentric = std::make_shared<Geocentric>(a, f);
    std::vector<Vector3d> points(latlon.size());
    int i = 0;
    for (const auto &p : latlon) {
        geocentric->Forward(p[0], p[1], p[2], points[i][0], points[i][1], points[i][2]);
        ++i;
    }
    return points;
}

Vector3d latlon_to_geocentric(const Vector3d &latlon, const std::string &planet) {
    double a = Constants::WGS84_a(), f = Constants::WGS84_f();
    if (string_to_lower(planet) == "moon") {
        a = MOON_RADIUS;
        f = 0.0;
    } else if (string_to_lower(planet) == "mars") {
        a = MARS_RADIUS;
        f = 0.0;
    } else if (string_to_lower(planet) == "earth") {
        a = Constants::WGS84_a();
        f = Constants::WGS84_f();
    }
    return latlon_to_geocentric(latlon, a, f);
}

Vector3d latlon_to_geocentric(const Vector3d &latlon, double a, double f) {
    std::shared_ptr<Geocentric> geocentric = std::make_shared<Geocentric>(a, f);
    Vector3d point;
    geocentric->Forward(latlon[0], latlon[1], latlon[2], point[0], point[1], point[2]);
    return point;
}

std::vector<Vector3d> geocentric_to_latlon(const std::vector<Vector3d> &points, const std::string &planet) {
    double a = Constants::WGS84_a(), f = Constants::WGS84_f();
    if (string_to_lower(planet) == "moon") {
        a = MOON_RADIUS;
        f = 0.0;
    } else if (string_to_lower(planet) == "mars") {
        a = MARS_RADIUS;
        f = 0.0;
    } else if (string_to_lower(planet) == "earth") {
        a = Constants::WGS84_a();
        f = Constants::WGS84_f();
    }
    return geocentric_to_latlon(points, a, f);
}

std::vector<Vector3d> geocentric_to_latlon(const std::vector<Vector3d> &points, double a, double f) {
    std::shared_ptr<Geocentric> geocentric = std::make_shared<Geocentric>(a, f);
    std::vector<Vector3d> latlons(points.size());
    int i = 0;
    for (const auto &p : points) {
        geocentric->Reverse(p[0], p[1], p[2], latlons[i][0], latlons[i][1], latlons[i][2]);
        ++i;
    }
    return latlons;
}

Vector3d geocentric_to_latlon(const Vector3d &point, const std::string &planet) {
    double a = Constants::WGS84_a(), f = Constants::WGS84_f();
    if (string_to_lower(planet) == "moon") {
        a = MOON_RADIUS;
        f = 0.0;
    } else if (string_to_lower(planet) == "mars") {
        a = MARS_RADIUS;
        f = 0.0;
    } else if (string_to_lower(planet) == "earth") {
        a = Constants::WGS84_a();
        f = Constants::WGS84_f();
    }
    return geocentric_to_latlon(point, a, f);
}

Vector3d geocentric_to_latlon(const Vector3d &point, double a, double f) {
    std::shared_ptr<Geocentric> geocentric = std::make_shared<Geocentric>(a, f);
    Vector3d latlon;
    geocentric->Reverse(point[0], point[1], point[2], latlon[0], latlon[1], latlon[2]);

    return latlon;
}
} // namespace h2o
