/*
 * Filename: d:\repo\linsar\src\polygon\polygon_simplification.cpp
 * Path: d:\repo\linsar\src\polygon
 * Created Date: Monday, December 16th 2019, 8:00:18 pm
 * Author: Han Hu
 * 多边形简化
 * Copyright (c) 2019 Vrlab
 */
#include "polygon_simplification.h"

#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Simple_cartesian.h>

#include <gdal.h>
#include <gdal_alg.h>
#include <gdal_priv.h>
#include <ogrsf_frmts.h>

#include <base/filesystem.h>

#include <QTemporaryDir>

namespace h2o {

PolygonRing polygon_simplification(const PolygonRing &ring, double threshold, int npoints) {
    namespace PS = CGAL::Polyline_simplification_2;
    typedef CGAL::Simple_cartesian<double> Kernel;
    typedef CGAL::Polygon_2<Kernel> Polygon_2;
    typedef PS::Squared_distance_cost Cost;
    typedef Kernel::Point_2 Point_2;

    std::vector<Point_2> points;
    for (const Vector2f &p : ring) {
        points.emplace_back(p.x(), p.y());
    }

    Polygon_2 cgal_polygon(begin(points), end(points));
    auto cgal_simplified = PS::simplify(
        cgal_polygon, Cost(), [&](const auto &cdt, auto vertex, auto cost, size_t initial_cound, size_t count) -> bool {
            return (count < npoints) || (cost > threshold * threshold);
        });

    PolygonRing output;
    for (auto i = cgal_simplified.vertices_begin(); i != cgal_simplified.vertices_end(); ++i) {
        output.emplace_back(i->x(), i->y());
    }
    return output;
}

void polygon_fix(PolygonPtr &polygon, double t_length, double t_hole) {}

PolygonPtr polygon_fix(const PolygonPtr &polygon, double t_length, double t_hole) {
    // 只编辑多边形
    if (polygon->get_polygon_type() == PolygonTypePolyline) {
        return nullptr;
    }

    Paths paths = polygon->get_paths();
    PolygonHeights heights = polygon->get_heights();
    double t_area = t_hole * t_hole;

    Paths path_fixed;
    PolygonHeights heights_fixed;

    for (int i = 0; i < paths.size(); ++i) {
        Path path = paths.at(i);
        double area = path_area(path, Polygon::SCALE);
        if (i != 0 && area < t_area) {
            continue;
        }

        PolygonRing ring = path_to_ring(path);
        if (t_length > 0) {
            ring = polygon_simplification(ring, t_length);
        }

        path = ring_to_path(ring);
        PolygonHeight height(path.size(), heights.at(i).front());

        path_fixed.push_back(path);
        heights_fixed.push_back(height);
    }
    PolygonPtr polygon_fixed = polygon->clone();
    polygon_fixed->set_paths(path_fixed);
    polygon_fixed->set_heights(heights_fixed);
    polygon_fixed->init_bounds();
    return polygon_fixed;
}

} // namespace h2o
