/*
 * Filename: /home/han/Documents/LinSAR/src/polygon/polygon.cpp
 * Path: /home/han/Documents/LinSAR/src/polygon
 * Created Date: Sunday, December 8th 2019, 7:03:19 pm
 * Author: han
 * 多边形类别，底层采用Clipper
 * Copyright (c) 2019 Vrlab
 */
#include "polygon.h"

#include <ogl/camera.h>

#include <QPainterPath>

namespace h2o {

void Polygon::init_bounds() {
    if (paths_.empty())
        return;
    bounds2d_ = BoundingBox2f();

    Path outer = paths_.at(0);
    for (const auto &cpoint : outer) {
        Vector2f point = cpoint_to_point(cpoint);
        bounds2d_.extend(point);
    }

    if (heights_.empty())
        return;
    bounds3d_ = BoundingBox3f();
    PolygonHeight height = heights_.at(0);
    for (int i = 0; i < height.size(); ++i) {
        Vector3f point;
        Vector2f point2d = cpoint_to_point(outer.at(i));
        point.x() = point2d.x();
        point.y() = point2d.y();
        point.z() = height.at(i);

        bounds3d_.extend(point);
    }
}

std::tuple<PolygonDecoration, PolygonVertexIndex> Polygon::select(const Vector2f &point2d, int threshold) const {
    PolygonDecoration decoration = PolygonNone;
    PolygonVertexIndex index;
    index.first = INVALID_INDEX;
    index.second = INVALID_INDEX;

    if (paths_.empty())
        return std::make_tuple(decoration, index);

    PolygonVertexIndex vindex = select_vertex(point2d, threshold);
    if (vindex.first != INVALID_INDEX)
        return std::make_tuple(PolygonVertex, vindex);

    PolygonVertexIndex eindex = select_edge(point2d, threshold);
    if (eindex.first != INVALID_INDEX)
        return std::make_tuple(PolygonEdge, eindex);

    PolygonVertexIndex cindex = select_center(point2d, threshold);
    if (cindex.first != INVALID_INDEX)
        return std::make_tuple(PolygonCenter, cindex);

    // 没有选择任何内容
    return std::make_tuple(PolygonNone, index);
}

float Polygon::get_scale() const {
    if (bounds2d_.isEmpty())
        return 0.0f;

    Vector3f point3d;
    point3d.x() = bounds2d_.center().x();
    point3d.y() = bounds2d_.center().y();
    point3d.z() = 0.0f;

    if (!bounds3d_.isEmpty())
        point3d.z() = bounds3d_.center().z();

    Vector2f screen2d1 = camera_->convert_3d_to_2d(point3d);
    point3d.x() += 1.0f;
    Vector2f screen2d2 = camera_->convert_3d_to_2d(point3d);

    // how much pixels for 1 meter in object
    float scale = (screen2d2 - screen2d1).norm();
    return 1.0f / scale;
}

Vector2f cpoint_to_point(const ClipperPoint &cpoint, int64_t scale) {
    Vector2f point;
    int64_t x_integer = cpoint.X / scale;
    int64_t y_integer = cpoint.Y / scale;

    point.x() = x_integer + (double)(cpoint.X - x_integer * scale) / scale;
    point.y() = y_integer + (double)(cpoint.Y - y_integer * scale) / scale;
    return point;
}

ClipperPoint point_to_cpoint(const Vector2f &point, int64_t scale) {
    ClipperPoint cpoint;
    cpoint.X = point.x() * scale + 0.5;
    cpoint.Y = point.y() * scale + 0.5;
    return cpoint;
}

PolygonRing path_to_ring(const Path &path, int64_t scale) {
    PolygonRing ring(path.size());
    for (int i = 0; i < path.size(); ++i) {
        ring[i] = cpoint_to_point(path.at(i));
    }
    return ring;
}

PolygonRings paths_to_rings(const Paths &paths, int64_t scale) {
    PolygonRings rings(paths.size());
    for (int i = 0; i < paths.size(); ++i) {
        PolygonRing ring = path_to_ring(paths.at(i));
        rings[i] = ring;
    }
    return rings;
}

Path ring_to_path(const PolygonRing &ring, int64_t scale) {
    Path path(ring.size());
    for (int i = 0; i < path.size(); ++i) {
        path[i] = point_to_cpoint(ring.at(i));
    }
    return path;
}

Paths rings_to_paths(const PolygonRings &rings, int64_t scale) {
    Paths paths(rings.size());
    for (int i = 0; i < paths.size(); ++i) {
        paths[i] = ring_to_path(rings.at(i));
    }
    return paths;
}

QPainterPath ring_to_painter(const PolygonRing &ring, bool close_sub_path) {
    QPolygonF polygon;
    for (const auto &point : ring) {
        polygon.append(QPointF(point.x(), point.y()));
    }
    QPainterPath painter;
    painter.addPolygon(polygon);
    if (close_sub_path)
        painter.closeSubpath();
    return painter;
}

QPainterPath rings_to_painter(const PolygonRings &rings, bool close_sub_path) {
    QPainterPath painter;
    for (const auto &ring : rings) {
        QPolygonF polygon;
        for (const auto &point : ring) {
            polygon.append(QPointF(point.x(), point.y()));
        }
        painter.addPolygon(polygon);
        if (close_sub_path)
            painter.closeSubpath();
    }
    return painter;
}

QPainterPath paths_to_painter(const Paths &paths, bool close_sub_path, int64_t scale) {
    PolygonRings rings = paths_to_rings(paths, scale);
    return rings_to_painter(rings, close_sub_path);
}

QPainterPath path_to_painter(const Path &path, bool close_sub_path, int64_t scale) {
    PolygonRing ring = path_to_ring(path, scale);
    return ring_to_painter(ring, close_sub_path);
}

float path_area(const Path &path, int64_t scale) {
    if (path.size() < 3) {
        return -1.0f;
    }

    int64_t area = std::abs(ClipperLib::Area(path));
    int64_t area_modular = area / scale / scale;
    double aread = area_modular + (double)(area - area_modular * scale * scale) / scale / scale;

    return aread;
}

float ring_aera(const PolygonRing &ring) {
    Path path = ring_to_path(ring, 1000);
    return path_area(path, 1000);
}

std::vector<PolygonRings> paths_to_multirings(const ClipperLib::Paths &paths, int64_t scale) {
    // 先填充所有 outer
    std::vector<PolygonRings> results;
    for (int i = 0; i < paths.size(); ++i) {
        if (Orientation(paths[i])) {
            const Path &outer_path = paths[i];
            PolygonRing outer = path_to_ring(outer_path, scale);
            PolygonRings rings;
            rings.push_back(outer);
            results.push_back(rings);
        }
    }

    for (int i = 0; i < paths.size(); ++i) {
        const Path &hole = paths[i];
        if (Orientation(hole)) {
            // 只填充内部的洞
            continue;
        }

        bool all_inside = true;
        // 判断当前hole 是否在某个多边形内部
        for (PolygonRings &rings : results) {
            Path outer_path = ring_to_path(rings.at(0), scale);
            for (const ClipperPoint &pt : hole) {
                if (!PointInPolygon(pt, outer_path)) {
                    all_inside = false;
                    break;
                }
            }
            if (!all_inside) {
                continue;
            }
            PolygonRing ring = path_to_ring(hole, scale);
            rings.push_back(ring);
        }
    }

    return results;
}

std::vector<Paths> paths_to_multipaths(const Paths &paths) {
    // 先填充所有 outer
    std::vector<Paths> results;
    for (int i = 0; i < paths.size(); ++i) {
        if (Orientation(paths[i])) {
            const Path &outer_path = paths[i];
            Paths rings;
            rings.push_back(outer_path);
            results.push_back(rings);
        }
    }

    for (int i = 0; i < paths.size(); ++i) {
        const Path &hole = paths[i];
        if (Orientation(hole)) {
            // 只填充内部的洞
            continue;
        }

        bool all_inside = true;
        // 判断当前hole 是否在某个多边形内部
        for (Paths &rings : results) {
            Path outer_path = rings.at(0);
            for (const ClipperPoint &pt : hole) {
                if (!PointInPolygon(pt, outer_path)) {
                    all_inside = false;
                    break;
                }
            }
            if (!all_inside) {
                continue;
            }
            rings.push_back(hole);
        }
    }

    return results;
}

std::vector<PolygonRings> find_contours(const cv::Mat &mask, int decimode, const Vector2i &offset) {
    std::vector<PolygonRings> results;

    std::vector<cv::Vec4i> hierachy;
    std::vector<std::vector<cv::Point>> contours;

    // 由于边缘扩充了一像素，这里顾及这个偏移
    cv::findContours(mask, contours, hierachy, cv::RETR_CCOMP, decimode, {offset(0), offset(1)});

    PolygonRings *rings;
    bool has_outer = false;
    for (int i = 0; i < contours.size(); ++i) {
        const std::vector<cv::Point> &contour = contours[i];
        cv::Vec4i list = hierachy[i];

        if (contour.size() <= 3) {
            // 不知道为什么会有一个点的洞
            continue;
        }

        if (has_outer && list[3] < 0) {
            // 新的 outer，创建一个新的带洞多边形
            rings = nullptr;
            has_outer = false;
        }

        PolygonRing ring(contour.size());
        for (int j = 0; j < ring.size(); ++j) {
            // 转换到 物方坐标系
            Vector2f pixel(contour[j].x, contour[j].y);
            ring[j] = pixel;
        }

        if (list[3] >= 0) {
            // hole
            rings->push_back(ring);
        } else {
            results.push_back(PolygonRings());
            rings = &results.back();
            // outer
            rings->push_back(ring);
            has_outer = true;
        }
    }

    return results;
}

} // namespace h2o
