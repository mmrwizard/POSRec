/*
 * Filename: d:\repo\linsar\src\polygon\polygonize.h
 * Path: d:\repo\linsar\src\polygon
 * Created Date: Friday, December 27th 2019, 2:57:38 pm
 * Author: Han Hu
 * 栅格转矢量
 * Copyright (c) 2019 Vrlab
 */
#pragma once

#include "polygon.h"

namespace h2o {
PolygonRings polygonize(const cv::Mat &mask);
} // namespace h2o
