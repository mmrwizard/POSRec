/*
 * Filename: /home/han/Documents/LinSAR/src/polygon/polygon_polygonwithhole.cpp
 * Path: /home/han/Documents/LinSAR/src/polygon
 * Created Date: Saturday, December 14th 2019, 10:38:42 pm
 * Author: han
 * 带洞的多边形
 * Copyright (c) 2019 Vrlab
 */
#include "polygon.h"
#include "polygon_bool.h"
#include "polygon_triangulate.h"

#include <ogl/camera.h>

#include <QPainterPath>

namespace h2o {
class PolygonWithHoles : public Polygon {
public:
    void set_outer_ring(const PolygonRing &ring) override;
    void add_hole(const PolygonRing &hole) override;

    void move_point(const PolygonVertexIndex &index, const Vector2f &point) override;
    void add_point(const PolygonVertexIndex &index, const Vector2f &point) override;
    void sep_point(const PolygonVertexIndex &index) override;
    void del_point(const PolygonVertexIndex &index) override;

    PolygonVertexIndex select_vertex(const Vector2f &point2d, int threshold = 5) const override;
    PolygonVertexIndex select_edge(const Vector2f &point2d, int threshold = 5) const override;
    PolygonVertexIndex select_center(const Vector2f &point2d, int threshold = 5) const override;

    bool is_selected(const Vector2f &point2d, int threshold = 5) const override;
    bool is_center_selected(const Vector2f &point2d, int threshold = 5) const override;
    bool is_contained(const Vector2f &point2d) const override;

    PolygonType get_polygon_type() const override { return PolygonTypePolygon; }
    PolygonRings convert_to_rings() const override;

    PolygonMesh triangulate() const override;
    QPainterPath get_painterpath() const override;

    float area() const override;
    float length() const override;

    std::shared_ptr<Polygon> clone() const override;
};

PolygonPtr create_polygon() { return std::make_shared<PolygonWithHoles>(); }

void PolygonWithHoles::set_outer_ring(const PolygonRing &ring) {
    paths_.clear();

    Path path(ring.size());

    for (int i = 0; i < ring.size(); ++i) {
        path[i] = point_to_cpoint(ring.at(i));
    }

    if (!ClipperLib::Orientation(path)) {
        ClipperLib::ReversePath(path);
    }

    paths_.push_back(path);
}

void PolygonWithHoles::add_hole(const PolygonRing &hole) {
    if (paths_.empty())
        return;

    Path path(hole.size());
    for (int i = 0; i < hole.size(); ++i) {
        path[i] = point_to_cpoint(hole.at(i));
    }
    paths_.push_back(path);
}

void PolygonWithHoles::move_point(const PolygonVertexIndex &index, const Vector2f &point) {
    if (paths_.empty() || index.first >= paths_.size() || index.second == INVALID_INDEX ||
        index.second >= paths_.at(index.first).size())
        return;

    paths_.at(index.first).at(index.second) = point_to_cpoint(point);
}

void PolygonWithHoles::add_point(const PolygonVertexIndex &index, const Vector2f &point) {
    if (paths_.empty() || index.first >= paths_.size())
        return;

    paths_.at(index.first).push_back(point_to_cpoint(point));
}

void PolygonWithHoles::sep_point(const PolygonVertexIndex &index) {
    if (paths_.empty() || index.first >= paths_.size() || index.second == INVALID_INDEX ||
        index.second >= paths_.at(index.first).size())
        return;

    auto &path = paths_.at(index.first);
    const auto &pt0 = path.at(index.second);
    const auto &pt1 = path.at((index.second + 1) % path.size());

    ClipperPoint center;
    center.X = (pt0.X + pt1.X) / 2;
    center.Y = (pt0.Y + pt1.Y) / 2;

    path.insert(path.begin() + index.second + 1, center);
}

void PolygonWithHoles::del_point(const PolygonVertexIndex &index) {
    if (paths_.empty() || index.first >= paths_.size() || index.second == INVALID_INDEX ||
        index.second >= paths_.at(index.first).size())
        return;

    auto &path = paths_.at(index.first);
    path.erase(path.begin() + index.second);
}

PolygonVertexIndex PolygonWithHoles::select_vertex(const Vector2f &point2d, int threshold) const {
    PolygonVertexIndex index;
    index.first = INVALID_INDEX;
    index.second = INVALID_INDEX;

    if (paths_.empty())
        return index;
    float scale = get_scale();
    float object_thresh = (float)threshold * scale;
    int64_t object_thresh_integer = object_thresh * SCALE;
    ClipperPoint cpoint2d = point_to_cpoint(point2d);

    // 对每个 ring
    for (int i = 0; i < paths_.size(); ++i) {
        const auto &path = paths_.at(i);

        for (int j = 0; j < path.size(); ++j) {
            ClipperPoint pt = path.at(j);

            pt.X = cpoint2d.X - pt.X;
            pt.Y = cpoint2d.Y - pt.Y;

            if (pt.X * pt.X + pt.Y * pt.Y < object_thresh_integer * object_thresh_integer) {
                index.first = i;
                index.second = j;
                return index;
            }
        }
    }

    return index;
}

PolygonVertexIndex PolygonWithHoles::select_edge(const Vector2f &point2d, int threshold) const {
    PolygonVertexIndex index;
    index.first = INVALID_INDEX;
    index.second = INVALID_INDEX;

    if (paths_.empty())
        return index;
    float scale = get_scale();
    float object_thresh = (float)threshold * scale;
    int64_t object_thresh_integer = object_thresh * SCALE;
    ClipperPoint cpoint2d = point_to_cpoint(point2d);

    // 对每个 ring
    for (int i = 0; i < paths_.size(); ++i) {
        const auto &path = paths_.at(i);

        for (int j = 0; j < path.size(); ++j) {
            ClipperPoint pt0 = path.at(j);
            ClipperPoint pt1 = path.at((j + 1) % path.size());

            ClipperPoint center;
            center.X = (pt0.X + pt1.X) / 2;
            center.Y = (pt0.Y + pt1.Y) / 2;

            center.X = cpoint2d.X - center.X;
            center.Y = cpoint2d.Y - center.Y;

            if (center.X * center.X + center.Y * center.Y < object_thresh_integer * object_thresh_integer) {
                index.first = i;
                index.second = j;
                return index;
            }
        }
    }

    return index;
}

PolygonVertexIndex PolygonWithHoles::select_center(const Vector2f &point2d, int threshold) const {
    // 判断每个点是否被选中
    PolygonVertexIndex index;
    index.first = INVALID_INDEX;
    index.second = INVALID_INDEX;

    if (paths_.empty())
        return index;

    if (bounds2d_.isEmpty())
        return index;

    float scale = get_scale();
    float object_thresh = (float)threshold * scale;
    Vector2f center = bounds2d_.center();
    if ((point2d - center).norm() < object_thresh) {
        index.first = 0;
        index.second = 0;
        return index;
    }
    return index;
}

bool PolygonWithHoles::is_selected(const Vector2f &point2d, int threshold) const {
    // ignore the threshold
    ClipperPoint cpoint = point_to_cpoint(point2d);
    return point_in_paths(cpoint, paths_);
}

bool PolygonWithHoles::is_center_selected(const Vector2f &point2d, int threshold) const {
    if (paths_.empty())
        return false;

    if (bounds2d_.isEmpty())
        return false;

    float scale = get_scale();
    float object_thresh = (float)threshold * scale;
    Vector2f center = bounds2d_.center();
    if ((point2d - center).norm() < object_thresh) {
        return true;
    }
    return false;
}

bool PolygonWithHoles::is_contained(const Vector2f &point2d) const {
    ClipperPoint cpoint = point_to_cpoint(point2d);
    return point_in_paths(cpoint, paths_);
}

PolygonRings PolygonWithHoles::convert_to_rings() const { return paths_to_rings(paths_, SCALE); }

PolygonMesh PolygonWithHoles::triangulate() const {
    //    PolygonRings rings = paths_to_rings(paths_, SCALE);
    if (heights_.size() == paths_.size()) {
        return polygon_triangulate(paths_, heights_);
    }
    return polygon_triangulate(paths_);
}

QPainterPath PolygonWithHoles::get_painterpath() const { return paths_to_painter(paths_, true, SCALE); }

float PolygonWithHoles::area() const {
    float value = 0.0f;
    if (paths_.empty())
        return -1.0f;

    for (int i = 0; i < paths_.size(); ++i) {
        if (i == 0)
            value += path_area(paths_.at(i), SCALE);
        else
            value -= path_area(paths_.at(i), SCALE);
    }
    return value;
}

float PolygonWithHoles::length() const {
    float value = 0.0f;
    if (paths_.empty())
        return -1.0f;

    const auto &path = paths_.at(0);
    for (int i = 0; i < path.size(); ++i) {
        const auto &pt0 = path.at(i);
        const auto &pt1 = path.at((i + 1) % path.size());

        float x = pt0.X - pt1.X;
        float y = pt0.Y - pt1.Y;

        value += std::sqrt(x * x + y * y);
    }
    return value;
}

std::shared_ptr<Polygon> PolygonWithHoles::clone() const {
    std::shared_ptr<Polygon> cloned = std::make_shared<PolygonWithHoles>();
    cloned->set_paths(paths_);
    cloned->set_heights(heights_);
    cloned->init_bounds();
    return cloned;
}

} // namespace h2o
