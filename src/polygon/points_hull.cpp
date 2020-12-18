/*
 * Created Date: Tuesday, September 8th 2020, 10:46:29 am
 * Author: Han
 * 点云计算 2d 凸包
 * Copyright (c) 2020 Vrlab
 */

#include "polygon.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/ch_graham_andrew.h>
#include <CGAL/convex_hull_2.h>

using Kernel = CGAL::Simple_cartesian<float>;
using Point_2 = Kernel::Point_2;

namespace h2o {
Path find_hull(const MatrixXf &points, int64_t scale) {
    std::vector<Point_2> points_2(points.cols());
    for (int i = 0; i < points_2.size(); ++i) {
        points_2[i] = Point_2(points(0, i), points(1, i));
    }
    std::vector<Point_2> points_hull;

    CGAL::ch_graham_andrew(begin(points_2), end(points_2), std::back_inserter(points_hull));

    PolygonRing ring(points_hull.size());
    for (int i = 0; i < ring.size(); ++i) {
        ring[i] = Vector2f(points_hull[i].x(), points_hull[i].y());
    }

    Path path = ring_to_path(ring, scale);
    if (!ClipperLib::Orientation(path)) {
        ClipperLib::ReversePath(path);
    }
    return path;
}
} // namespace h2o
