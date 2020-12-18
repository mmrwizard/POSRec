/*
 * Filename: /home/han/Documents/LinSAR/src/polygon/polygon_bool.h
 * Path: /home/han/Documents/LinSAR/src/polygon
 * Created Date: Saturday, December 14th 2019, 9:38:16 pm
 * Author: han
 * 多边形布尔运算
 * Copyright (c) 2019 Vrlab
 */
#include "polygon_bool.h"

namespace h2o {

using namespace ClipperLib;

bool point_in_paths(const ClipperPoint &point, const Paths &paths) {
    // 只对根节点处理
    // 需要判断所有的child和自身
    bool in_polygon = true;
    for (int i = 0; i < paths.size(); ++i) {
        if (i == 0) {
            in_polygon = in_polygon && PointInPolygon(point, paths.at(i));
        } else {
            in_polygon = in_polygon && !PointInPolygon(point, paths.at(i));
        }
        if (!in_polygon)
            return false;
    }
    return in_polygon;
}

ClipperLib::Paths polygon_boolop_union(const ClipperLib::Paths &paths) {
    if (paths.empty()) {
        return Paths();
    }
    Clipper clipper;
    clipper.AddPaths(paths, ptSubject, true);
    Paths solutions;
    if (clipper.Execute(ctUnion, solutions, pftNonZero, pftNonZero)) {
        return solutions;
    } else {
        return Paths();
    }
}

ClipperLib::Paths polygon_boolop_subtract(const ClipperLib::Paths &subject, const ClipperLib::Paths &clip) {
    if (subject.empty() || clip.empty()) {
        return Paths();
    }
    Clipper clipper;
    clipper.AddPaths(subject, ptSubject, true);
    clipper.AddPaths(clip, ptClip, true);
    Paths solutions;
    if (clipper.Execute(ctDifference, solutions, pftNonZero, pftNonZero)) {
        return solutions;
    } else {
        return Paths();
    }
}

ClipperLib::Paths polygon_boolop_intersect(const ClipperLib::Paths &subject, const ClipperLib::Paths &clip) {
    if (subject.empty() || clip.empty()) {
        return Paths();
    }
    Clipper clipper;
    clipper.AddPaths(subject, ptSubject, true);
    clipper.AddPaths(clip, ptClip, true);
    Paths solutions;
    if (clipper.Execute(ctIntersection, solutions, pftNonZero, pftNonZero)) {
        return solutions;
    } else {
        return Paths();
    }
}

bool polygon_boolop_intersect_line(const ClipperLib::Paths &subject, const ClipperLib::Paths &clip) {
    if (subject.empty() || clip.empty()) {
        return false;
    }
    Clipper clipper;
    clipper.AddPaths(subject, ptSubject, false);
    clipper.AddPaths(clip, ptClip, true);
    PolyTree solutions;
    if (clipper.Execute(ctIntersection, solutions, pftNonZero, pftNonZero)) {
        return solutions.Total();
    } else {
        return false;
    }
}

ClipperLib::Paths polygon_offset(const ClipperLib::Paths &paths, double delta) {
    ClipperOffset offseter;
    offseter.AddPaths(paths, jtMiter, etClosedPolygon);
    Paths solutions;
    offseter.Execute(solutions, delta);
    return solutions;
}

ClipperLib::Paths polyline_offset(const ClipperLib::Path &path, double delta, bool rounded) {
    ClipperOffset offseter;
    EndType type = rounded ? etOpenRound : etOpenButt;
    if (Orientation(path)) {
        offseter.AddPath(path, jtMiter, type);
    } else {
        Path rev = path;
        ReversePath(rev);
        offseter.AddPath(rev, jtMiter, type);
    }

    Paths solutions;
    offseter.Execute(solutions, delta);
    return solutions;
}

} // namespace h2o
