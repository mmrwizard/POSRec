/*
 * Filename: d:\repo\linsar\src\polygon\polygon_draw.h
 * Path: d:\repo\linsar\src\polygon
 * Created Date: Monday, December 16th 2019, 8:00:18 pm
 * Author: Han Hu
 * 辅助绘制多边形的函数
 * Copyright (c) 2019 Vrlab
 */
#pragma once

#include <QBrush>
#include <QColor>
#include <QPen>

#include "polygon.h"

namespace h2o {
QPen create_dashed_pen(int dash);
QPen create_white_pen();
QPen create_black_pen();
QPen create_color_pen(const QColor &color);

void draw_polygon(cv::Mat &mat, const PolygonRings &paths, const Vector4f &color, const Matrix23d &xform);
void draw_polygon(cv::Mat &mat, const Paths &paths, const Vector4f &color, const Matrix23d &xform,
                  int64_t scale = 1000);
void draw_polygon(cv::Mat &mat, const PolygonRings &paths, const Vector4f &color);
void draw_polygon(cv::Mat &mat, const Paths &paths, const Vector4f &color, int64_t scale = 1000);

void draw_polylines(cv::Mat &mat, const PolygonRings &polyline, const Vector4f &value);
void draw_polylines(cv::Mat &mat, const PolygonRing &polyline, const Vector4f &value);
void draw_polylines(cv::Mat &mat, const PolygonRings &polyline, const Vector4f &value, const Matrix23d &xform);
void draw_polylines(cv::Mat &mat, const PolygonRing &polyline, const Vector4f &value, const Matrix23d &xform);

// TODO draw polyline 的 paths 的接口

// 线性内插一条直线
void line2(cv::Mat &img, const cv::Point &start, const cv::Point &end, const cv::Scalar &c1, const cv::Scalar &c2);
// TODO 加入带变换参数的接口

void fill_boundary(cv::Mat &mask, uint8_t value);

} // namespace h2o
