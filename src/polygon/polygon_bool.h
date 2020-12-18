/*
 * Filename: /home/han/Documents/LinSAR/src/polygon/polygon_bool.h
 * Path: /home/han/Documents/LinSAR/src/polygon
 * Created Date: Saturday, December 14th 2019, 9:38:16 pm
 * Author: han
 * 多边形布尔运算
 * Copyright (c) 2019 Vrlab
 */

#pragma once

#include "polygon.h"
#include <base/clipper.hpp>

namespace h2o {
// first is outer shell and all the others are holes
bool point_in_paths(const ClipperPoint &point, const Paths &paths);

// 合并所有
ClipperLib::Paths polygon_boolop_union(const ClipperLib::Paths &paths);
// subject - clip
ClipperLib::Paths polygon_boolop_subtract(const ClipperLib::Paths &subject, const ClipperLib::Paths &clip);
// subject 交 clip
ClipperLib::Paths polygon_boolop_intersect(const ClipperLib::Paths &subject, const ClipperLib::Paths &clip);
// subject 交 clip, 必须是 subject 是 line
bool polygon_boolop_intersect_line(const ClipperLib::Paths &subject, const ClipperLib::Paths &clip);

// 外部边界的 orientation 是 正的，hole的orientation 是负的
// 返回的path，的orientation也是一样的， 执行的是 多边形的 offset 操作
ClipperLib::Paths polygon_offset(const ClipperLib::Paths &paths, double delta);

// 折线扩充成一个多边形, offset 需要大于0，线的 orientation 是正的(否则会在内部转换)
ClipperLib::Paths polyline_offset(const ClipperLib::Path &path, double delta, bool rounded);
} // namespace h2o
