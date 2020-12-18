/*
 * Filename: /home/han/Documents/LinSAR/src/polygon/polygon_triangulate.h
 * Path: /home/han/Documents/LinSAR/src/polygon
 * Created Date: Saturday, December 14th 2019, 9:38:16 pm
 * Author: han
 * 三角网构网，可顾及多边形的空洞
 * Copyright (c) 2019 Vrlab
 */
#include "polygon_triangulate.h"
#include <base/clipper.hpp>

#include <mapbox/earcut.hpp>

namespace mapbox {
namespace util {

template <> struct nth<0, Eigen::Vector2f> {
    inline static auto get(const Eigen::Vector2f &t) { return t(0); };
};
template <> struct nth<1, Eigen::Vector2f> {
    inline static auto get(const Eigen::Vector2f &t) { return t(1); };
};

template <> struct nth<0, ClipperLib::IntPoint> {
    inline static int64_t get(const ClipperLib::IntPoint &t) { return t.X; };
};
template <> struct nth<1, ClipperLib::IntPoint> {
    inline static int64_t get(const ClipperLib::IntPoint &t) { return t.Y; };
};

} // namespace util
} // namespace mapbox

namespace h2o {

PolygonMesh polygon_triangulate(const PolygonRings &rings) {
    PolygonMesh mesh;

    uint32_t total = 0;
    std::vector<uint32_t> num_points(rings.size());

    for (int i = 0; i < rings.size(); ++i) {
        total += rings.at(i).size();
        num_points[i] = total;
    }

    std::vector<uint32_t> indices;
    // 构网
    indices = mapbox::earcut<uint32_t>(rings);

    if (indices.empty())
        return mesh;

    mesh.V.resize(3, total);
    mesh.F.resize(3, indices.size() / 3);
    mesh.L.resize(2, total);

    for (int r = 0; r < rings.size(); ++r) {
        uint32_t start = r == 0 ? 0 : num_points[r - 1];
        uint32_t end = num_points[r];
        // 对每个 ring
        const PolygonRing &ring = rings[r];

        for (int p = 0; p < ring.size(); ++p) {
            // 对每个点
            const Vector2f &pt = ring[p];
            mesh.V(0, start + p) = pt(0);
            mesh.V(1, start + p) = pt(1);
            mesh.V(2, start + p) = 0.0f;

            mesh.L(0, start + p) = start + p;
            mesh.L(1, start + p) = start + (p + 1) % ring.size();
        }
    }

    // 对每个面
    for (int f = 0; f < mesh.F.cols(); ++f) {
        mesh.F(0, f) = indices[f * 3 + 0];
        mesh.F(1, f) = indices[f * 3 + 1];
        mesh.F(2, f) = indices[f * 3 + 2];
    }

    return mesh;
}

PolygonMesh polygon_triangulate(const PolygonRings &rings, const PolygonHeights &heights) {
    PolygonMesh mesh;

    uint32_t total = 0;
    std::vector<uint32_t> num_points(rings.size());

    for (int i = 0; i < rings.size(); ++i) {
        total += rings.at(i).size();
        num_points[i] = total;
    }

    std::vector<uint32_t> indices;
    // 构网
    indices = mapbox::earcut<uint32_t>(rings);

    if (indices.empty())
        return mesh;

    mesh.V.resize(3, total);
    mesh.F.resize(3, indices.size() / 3);
    mesh.L.resize(2, total);

    for (int r = 0; r < rings.size(); ++r) {
        uint32_t start = r == 0 ? 0 : num_points[r - 1];
        uint32_t end = num_points[r];
        // 对每个 ring
        const PolygonRing &ring = rings[r];
        const auto &height = heights.at(r);

        for (int p = 0; p < ring.size(); ++p) {
            // 对每个点
            const Vector2f &pt = ring[p];
            mesh.V(0, start + p) = pt(0);
            mesh.V(1, start + p) = pt(1);
            mesh.V(2, start + p) = height.at(p);

            mesh.L(0, start + p) = start + p;
            mesh.L(1, start + p) = start + (p + 1) % ring.size();
        }
    }

    // 对每个面
    for (int f = 0; f < mesh.F.cols(); ++f) {
        mesh.F(0, f) = indices[f * 3 + 0];
        mesh.F(1, f) = indices[f * 3 + 1];
        mesh.F(2, f) = indices[f * 3 + 2];
    }

    return mesh;
}

PolygonMesh polygon_triangulate(const Paths &paths, const PolygonHeights &heights, int64_t SCALE) {
    PolygonMesh mesh;

    uint32_t total = 0;
    std::vector<uint32_t> num_points(paths.size());

    for (int i = 0; i < paths.size(); ++i) {
        total += paths.at(i).size();
        num_points[i] = total;
    }

    std::vector<uint32_t> indices;
    // 构网
    indices = mapbox::earcut<uint32_t>(paths);

    if (indices.empty())
        return mesh;

    mesh.V.resize(3, total);
    mesh.F.resize(3, indices.size() / 3);
    mesh.L.resize(2, total);

    for (int r = 0; r < paths.size(); ++r) {
        uint32_t start = r == 0 ? 0 : num_points[r - 1];
        uint32_t end = num_points[r];
        // 对每个 ring
        const Path &path = paths[r];

        for (int p = 0; p < path.size(); ++p) {
            // 对每个点
            const ClipperPoint &cpt = path[p];
            Vector2f pt = cpoint_to_point(cpt, SCALE);
            mesh.V(0, start + p) = pt(0);
            mesh.V(1, start + p) = pt(1);
            if (heights.empty()) {
                mesh.V(2, start + p) = 0.0f;
            } else {
                mesh.V(2, start + p) = heights.at(r).at(p);
            }

            mesh.L(0, start + p) = start + p;
            mesh.L(1, start + p) = start + (p + 1) % path.size();
        }
    }

    // 对每个面
    for (int f = 0; f < mesh.F.cols(); ++f) {
        mesh.F(0, f) = indices[f * 3 + 0];
        mesh.F(1, f) = indices[f * 3 + 1];
        mesh.F(2, f) = indices[f * 3 + 2];
    }

    return mesh;
}

} // namespace h2o
