/*
 * Filename: d:\repo\linsar\src\polygon\polygon_simplification.h
 * Path: d:\repo\linsar\src\polygon
 * Created Date: Monday, December 16th 2019, 8:00:18 pm
 * Author: Han Hu
 * 多边形简化
 * Copyright (c) 2019 Vrlab
 */
#pragma once

#include "polygon.h"

namespace h2o {
PolygonRing polygon_simplification(const PolygonRing &ring, double threshold, int npoints = 1000);
PolygonPtr polygon_fix(const PolygonPtr &polygon, double t_length, double t_hole);
} // namespace h2o
