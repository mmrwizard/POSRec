/*
 * Filename: d:\repo\linsar\src\polygon\polygon_serialization.hpp
 * Path: d:\repo\linsar\src\polygon
 * Created Date: Thursday, January 9th 2020, 3:37:28 pm
 * Author: Han Hu
 * 序列化
 * Copyright (c) 2020 Vrlab
 */
#pragma once

#include "polygon.h"

#include <cereal/archives/binary.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/vector.hpp>

using namespace h2o;

namespace cereal {
template <class Archive> void serialize(Archive &ar, ClipperPoint &o) {
    ar(o.X);
    ar(o.Y);
}
// template <class Archive> void serialize(Archive &ar, Path &o) { ar(o); }
// template <class Archive> void serialize(Archive &ar, Paths &o) { ar(o); }

template <class Archive, typename T>
inline void save(Archive &ar, const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &m) {
    ar(m.rows());
    ar(m.cols());
    ar(cereal::binary_data(m.data(), m.rows() * m.cols() * sizeof(T)));
}
template <class Archive, typename T>
inline void load(Archive &ar, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &m) {
    Eigen::Index rows, cols;
    ar(rows);
    ar(cols);

    m.resize(rows, cols);
    ar(cereal::binary_data(m.data(), rows * cols * sizeof(T)));
}
template <class Archive, typename T> void serialize(Archive &ar, Eigen::Matrix<T, 2, 1> &m) {
    ar(m(0));
    ar(m(1));
}
template <class Archive, typename T> void serialize(Archive &ar, Eigen::Matrix<T, 3, 1> &m) {
    ar(m(0));
    ar(m(1));
    ar(m(2));
}
template <class Archive> void serialize(Archive &ar, PolygonMesh &mesh) {
    ar(mesh.F);
    ar(mesh.L);
    ar(mesh.V);
}

template <class Archive> void save(Archive &ar, const PolygonPtr &polygon) {
    PolygonType type = polygon->get_polygon_type();
    h2o::Paths paths = polygon->get_paths();
    PolygonHeights heights = polygon->get_heights();

    ar(type);
    ar(paths);
    ar(heights);
}

template <class Archive> void load(Archive &ar, PolygonPtr &polygon) {
    PolygonType type;
    h2o::Paths paths;
    PolygonHeights heights;

    ar(type);
    ar(paths);
    ar(heights);

    if (type == PolygonTypePolygon) {
        polygon = create_polygon();
    } else if (type == PolygonTypePolyline) {
        polygon = create_polyline();
    } else {
        polygon = nullptr;
        return;
    }

    polygon->set_paths(paths);
    polygon->set_heights(heights);
    polygon->init_bounds();
}

template <class Archive> void serialize(Archive &ar, FilterPolygons &filter) {
    ar(filter.blend_type);
    ar(filter.filter_type);
    ar(filter.polygons);
}
} // namespace cereal

namespace h2o {} // namespace h2o
