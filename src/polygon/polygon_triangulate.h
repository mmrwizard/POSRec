/*
 * Filename: /home/han/Documents/LinSAR/src/polygon/polygon_triangulate.h
 * Path: /home/han/Documents/LinSAR/src/polygon
 * Created Date: Saturday, December 14th 2019, 9:38:16 pm
 * Author: han
 * 三角网构网，可顾及多边形的空洞
 * Copyright (c) 2019 Vrlab
 */
#pragma once

#include "polygon.h"

namespace h2o {
PolygonMesh polygon_triangulate(const PolygonRings &rings);
PolygonMesh polygon_triangulate(const PolygonRings &rings, const PolygonHeights &heights);
PolygonMesh polygon_triangulate(const Paths &paths, const PolygonHeights &heights = PolygonHeights(),
                                int64_t SCALE = Polygon::SCALE);
} // namespace h2o
