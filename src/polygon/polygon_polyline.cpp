/*
 * Filename: /home/han/Documents/LinSAR/src/polygon/polygon_polyline.cpp
 * Path: /home/han/Documents/LinSAR/src/polygon
 * Created Date: Saturday, December 14th 2019, 4:42:02 pm
 * Author: han
 * 折线编辑
 * Copyright (c) 2019 Vrlab
 */
#include "polygon.h"

#include <QPainterPath>
#include <base/clipper.hpp>

namespace h2o {

class Polyline : public Polygon {
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

    PolygonType get_polygon_type() const override;
    PolygonRings convert_to_rings() const override;

    PolygonMesh triangulate() const override;
    QPainterPath get_painterpath() const override;

    float area() const override;
    float length() const override;

    std::shared_ptr<Polygon> clone() const override;
};

PolygonPtr create_polyline() { return std::make_shared<Polyline>(); }

void Polyline::set_outer_ring(const PolygonRing &ring) {
    // this clears all the data
    paths_.clear();
    paths_ = Paths(1);

    Path path(ring.size());
    for (int i = 0; i < ring.size(); ++i) {
        ClipperPoint cpoint = point_to_cpoint(ring.at(i));
        path[i] = cpoint;
    }
    paths_[0] = path;
}

void Polyline::add_hole(const PolygonRing &hole) { return; }

void Polyline::move_point(const PolygonVertexIndex &index, const Vector2f &point) {
    if (index.first != 0 || index.second == INVALID_INDEX || paths_.empty() || index.second >= paths_.at(0).size())
        return;
    paths_[0][index.second] = point_to_cpoint(point);
}

void Polyline::add_point(const PolygonVertexIndex &index, const Vector2f &point) {
    if (index.first != 0 || paths_.empty()) {
        return;
    }

    paths_[0].push_back(point_to_cpoint(point));
}

void Polyline::sep_point(const PolygonVertexIndex &index) {
    if (index.first != 0 || index.second == INVALID_INDEX || paths_.empty() || index.second > paths_.size()) {
        return;
    }

    const auto &pt0 = paths_.at(0).at(index.second);
    const auto &pt1 = paths_.at(0).at((index.second + 1) % paths_.at(0).size());

    ClipperPoint center;
    center.X = (pt0.X + pt1.X) / 2;
    center.Y = (pt0.Y + pt1.Y) / 2;

    paths_.at(0).insert(begin(paths_.at(0)) + index.second + 1, center);
}

void Polyline::del_point(const PolygonVertexIndex &index) {
    if (paths_.empty() || index.second < paths_.at(0).size())
        return;
    paths_.at(0).erase(paths_.at(0).begin() + index.second);
}

PolygonVertexIndex Polyline::select_vertex(const Vector2f &point2d, int threshold) const {
    // 判断每个点是否被选中
    PolygonVertexIndex index;
    index.first = INVALID_INDEX;
    index.second = INVALID_INDEX;

    if (paths_.empty())
        return index;

    const auto &path = paths_.at(0);
    float scale = get_scale();

    float object_thresh = (float)threshold * scale;
    int64_t object_thresh_integer = object_thresh * SCALE;

    ClipperPoint cpoint2d = point_to_cpoint(point2d);

    for (int i = 0; i < path.size(); ++i) {
        ClipperPoint pt = path.at(i);

        pt.X = cpoint2d.X - pt.X;
        pt.Y = cpoint2d.Y - pt.Y;

        if (pt.X * pt.X + pt.Y * pt.Y < object_thresh_integer * object_thresh_integer) {
            index.first = 0;
            index.second = i;
            return index;
        }
    }

    return index;
}

PolygonVertexIndex Polyline::select_edge(const Vector2f &point2d, int threshold) const {
    // 判断每个点是否被选中
    PolygonVertexIndex index;
    index.first = INVALID_INDEX;
    index.second = INVALID_INDEX;

    if (paths_.empty())
        return index;

    const auto &path = paths_.at(0);
    float scale = get_scale();

    float object_thresh = (float)threshold * scale;
    int64_t object_thresh_integer = object_thresh * SCALE;

    ClipperPoint cpoint2d = point_to_cpoint(point2d);

    for (int i = 0; i < path.size() - 1; ++i) {
        ClipperPoint pt0 = path.at(i);
        ClipperPoint pt1 = path.at((i + 1) % path.size());

        ClipperPoint center;
        center.X = (pt0.X + pt1.X) / 2;
        center.Y = (pt0.Y + pt1.Y) / 2;

        center.X = cpoint2d.X - center.X;
        center.Y = cpoint2d.Y - center.Y;

        if (center.X * center.X + center.Y * center.Y < object_thresh_integer * object_thresh_integer) {
            index.first = 0;
            index.second = i;
            return index;
        }
    }

    return index;
}

PolygonVertexIndex Polyline::select_center(const Vector2f &point2d, int threshold) const {
    // 判断每个点是否被选中
    PolygonVertexIndex index;
    index.first = INVALID_INDEX;
    index.second = INVALID_INDEX;

    if (paths_.empty())
        return index;

    if (bounds2d_.isEmpty())
        return index;

    const auto &path = paths_.at(0);
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

bool Polyline::is_selected(const Vector2f &point2d, int threshold) const {
    // test if the point intersect with the polyline
    if (paths_.empty())
        return false;

    float scale = get_scale();
    float object_thresh = (float)threshold * scale;
    int64_t object_thresh_integer = object_thresh * SCALE;

    ClipperPoint cpoint2d = point_to_cpoint(point2d);

    Path point_path = {ClipperPoint(cpoint2d.X - object_thresh_integer, cpoint2d.Y - object_thresh_integer),
                       ClipperPoint(cpoint2d.X + object_thresh_integer, cpoint2d.Y - object_thresh_integer),
                       ClipperPoint(cpoint2d.X + object_thresh_integer, cpoint2d.Y + object_thresh_integer),
                       ClipperPoint(cpoint2d.X - object_thresh_integer, cpoint2d.Y + object_thresh_integer)};

    ClipperLib::Clipper clipper;
    clipper.AddPath(paths_.at(0), ClipperLib::ptSubject, false);
    clipper.AddPath(point_path, ClipperLib::ptClip, true);
    ClipperLib::PolyTree solutions;
    bool res = clipper.Execute(ClipperLib::ctIntersection, solutions);

    if (solutions.Total() == 0) {
        return false;
    } else {
        return true;
    }
}

bool Polyline::is_center_selected(const Vector2f &point2d, int threshold) const {
    PolygonVertexIndex cindex = select_center(point2d, threshold);
    if (cindex.first != INVALID_INDEX)
        return true;
    return false;
}

bool Polyline::is_contained(const Vector2f &point2d) const { return false; }

PolygonType Polyline::get_polygon_type() const { return PolygonTypePolyline; }

PolygonRings Polyline::convert_to_rings() const {
    PolygonRings rings;
    if (paths_.empty())
        return rings;

    rings.resize(paths_.size());
    for (int i = 0; i < paths_.size(); ++i) {

        const auto &path = paths_.at(i);
        PolygonRing ring(path.size());
        for (int j = 0; j < path.size(); ++j) {
            ring[j] = cpoint_to_point(path.at(j));
        }

        rings[i] = ring;
    }
    return rings;
}

PolygonMesh Polyline::triangulate() const {
    PolygonMesh mesh;
    if (paths_.empty())
        return mesh;

    if (bounds3d_.isEmpty())
        return mesh;

    const auto &path = paths_.at(0);
    mesh.V = MatrixXf::Zero(3, path.size());
    mesh.L = MatrixXu::Zero(2, path.size() - 1);

    PolygonHeight height;
    if (has_3d())
        height = heights_.at(0);

    for (int i = 0; i < path.size(); ++i) {
        Vector2f point2d = cpoint_to_point(path[i]);
        float z = bounds3d_.center().z();

        if (has_3d())
            z = height.at(i);

        mesh.V(0, i) = point2d.x();
        mesh.V(1, i) = point2d.y();
        mesh.V(2, i) = z;

        if (i < path.size() - 1) {
            mesh.L(0, i) = i;
            mesh.L(1, i) = i + 1;
        }
    }
    return mesh;
}

QPainterPath Polyline::get_painterpath() const {
    QPainterPath qpath;
    if (paths_.empty())
        return qpath;

    const Path &path = paths_.at(0);
    QPolygonF qpolygon;
    for (const auto &cpoint : path) {
        Vector2f point = cpoint_to_point(cpoint);
        qpolygon.append(QPointF(point.x(), point.y()));
    }
    qpath.addPolygon(qpolygon);
    return qpath;
}

float Polyline::area() const { return -1.0f; }

float Polyline::length() const {
    float value = 0.0f;
    if (paths_.empty())
        return -1.0f;

    const auto &path = paths_.at(0);
    for (int i = 0; i < path.size() - 1; ++i) {
        const auto &pt0 = path.at(i);
        const auto &pt1 = path.at(i + 1);

        float x = pt0.X - pt1.X;
        float y = pt0.Y - pt1.Y;

        value += std::sqrt(x * x + y * y);
    }
    return value;
}

std::shared_ptr<Polygon> Polyline::clone() const {
    std::shared_ptr<Polygon> cloned = std::make_shared<Polyline>();
    cloned->set_paths(paths_);
    cloned->set_heights(heights_);
    cloned->init_bounds();
    return cloned;
}
} // namespace h2o
