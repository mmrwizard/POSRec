/*
 * Filename: d:\repo\linsar\src\polygon\polygon_draw.cpp
 * Path: d:\repo\linsar\src\polygon
 * Created Date: Monday, December 16th 2019, 8:00:18 pm
 * Author: Han Hu
 * 辅助绘制多边形的函数
 * Copyright (c) 2019 Vrlab
 */
#include "polygon_draw.h"

namespace h2o {

QPen create_dashed_pen(int dash) {
    QPen pen;
    // QT 的 pattern 不能以 0 结尾，所以加一些 0.1 的 pattern
    QVector<qreal> dashes[8] = {{4.1, 4.1, 0.1, 0.1}, {0.1, 1.1, 4.1, 3.1}, {0.1, 2.1, 4.1, 2.1}, {0.1, 3.1, 4.1, 1.1},
                                {0.1, 4.1, 4.1, 0.1}, {1.1, 4.1, 3.1, 0.1}, {2.1, 4.1, 2.1, 0.1}, {3.1, 4.1, 1.1, 0.1}};
    pen.setStyle(Qt::CustomDashLine);
    pen.setDashPattern(dashes[dash % 8]);
    pen.setWidth(2);
    pen.setColor(QColor(0, 0, 0));
    pen.setCosmetic(true);

    return pen;
}

QPen create_white_pen() {
    QPen white;
    white.setWidth(2);
    white.setColor(QColor(255, 255, 255));
    white.setCosmetic(true);

    return white;
}

QPen create_black_pen() {
    QPen pen;
    pen.setWidth(2);
    pen.setColor(QColor(0, 0, 0));
    pen.setCosmetic(true);
    return pen;
}

QPen create_color_pen(const QColor &color) {
    QPen pen;
    pen.setWidth(2);
    pen.setColor(color);
    pen.setCosmetic(true);
    return pen;
}

void draw_polygon(cv::Mat &mat, const PolygonRings &rings, const Vector4f &color, const Matrix23d &xform) {
    const int SHIFT = 4; // 小数点的值，有 4 bit
    std::vector<std::vector<cv::Point>> points;
    for (const auto &ring : rings) {
        std::vector<cv::Point> cv_ring(ring.size());
        for (int i = 0; i < ring.size(); ++i) {
            Vector2d point(ring[i].x(), ring[i].y());
            point = xform * point.homogeneous();
            cv_ring[i] = cv::Point(point(0) * std::pow(2, SHIFT), point(1) * std::pow(2, SHIFT));
        }
        points.push_back(cv_ring);
    }
    cv::fillPoly(mat, points, cv::Scalar(color(0), color(1), color(2), color(3)), cv::LINE_8, SHIFT);
}

void draw_polygon(cv::Mat &mat, const Paths &paths, const Vector4f &color, const Matrix23d &xform, int64_t scale) {
    PolygonRings rings = paths_to_rings(paths, scale);
    draw_polygon(mat, rings, color, xform);
}

void draw_polygon(cv::Mat &mat, const PolygonRings &rings, const Vector4f &color) {
    const int SHIFT = 4; // 小数点的值，有 4 bit
    std::vector<std::vector<cv::Point>> points;
    for (const auto &ring : rings) {
        std::vector<cv::Point> cv_ring(ring.size());
        for (int i = 0; i < ring.size(); ++i) {
            Vector2d point(ring[i].x(), ring[i].y());
            cv_ring[i] = cv::Point(point(0) * std::pow(2, SHIFT), point(1) * std::pow(2, SHIFT));
        }
        points.push_back(cv_ring);
    }
    cv::fillPoly(mat, points, cv::Scalar(color(0), color(1), color(2), color(3)), cv::LINE_8, SHIFT);
}

void draw_polygon(cv::Mat &mat, const Paths &paths, const Vector4f &color, int64_t scale) {
    PolygonRings rings = paths_to_rings(paths, scale);
    draw_polygon(mat, rings, color);
}

void fill_boundary(cv::Mat &mask, uint8_t value) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    for (const auto &ring : contours) {
        for (const auto &pt : ring) {
            mask.at<uint8_t>(pt.y, pt.x) = value;
        }
    }
}

void draw_polylines(cv::Mat &mat, const PolygonRings &polyline, const Vector4f &value) {
    const int SHIFT = 4; // 小数点的值，有 4 bit
    std::vector<std::vector<cv::Point>> points;
    for (const auto &ring : polyline) {
        std::vector<cv::Point> cv_ring(ring.size());
        for (int i = 0; i < ring.size(); ++i) {
            cv_ring[i] = cv::Point(ring[i](0) * std::pow(2, SHIFT), ring[i](1) * std::pow(2, SHIFT));
        }
        points.push_back(cv_ring);
    }
    cv::polylines(mat, points, false, cv::Scalar(value(0), value(1), value(2), value(3)), 1, cv::LINE_8, SHIFT);
}

void draw_polylines(cv::Mat &mat, const PolygonRing &polyline, const Vector4f &value) {
    const int SHIFT = 4; // 小数点的值，有 4 bit
    std::vector<std::vector<cv::Point>> points;

    std::vector<cv::Point> cv_ring(polyline.size());
    for (int i = 0; i < polyline.size(); ++i) {
        cv_ring[i] = cv::Point(polyline[i](0) * std::pow(2, SHIFT), polyline[i](1) * std::pow(2, SHIFT));
    }
    points.push_back(cv_ring);
    cv::polylines(mat, points, false, cv::Scalar(value(0), value(1), value(2), value(3)), 1, cv::LINE_8, SHIFT);
}

void draw_polylines(cv::Mat &mat, const PolygonRings &polyline, const Vector4f &value, const Matrix23d &xform) {
    const int SHIFT = 4; // 小数点的值，有 4 bit
    std::vector<std::vector<cv::Point>> points;

    for (const auto &ring : polyline) {
        std::vector<cv::Point> cv_ring(ring.size());
        for (int i = 0; i < ring.size(); ++i) {
            Vector2d point(ring[i].x(), ring[i].y());
            point = xform * point.homogeneous();
            cv_ring[i] = cv::Point(point(0) * std::pow(2, SHIFT), point(1) * std::pow(2, SHIFT));
        }
        points.push_back(cv_ring);
    }
    cv::polylines(mat, points, false, cv::Scalar(value(0), value(1), value(2), value(3)), 1, cv::LINE_8, SHIFT);
}

void draw_polylines(cv::Mat &mat, const PolygonRing &polyline, const Vector4f &value, const Matrix23d &xform) {
    const int SHIFT = 4; // 小数点的值，有 4 bit
    std::vector<std::vector<cv::Point>> points;

    std::vector<cv::Point> cv_ring(polyline.size());
    for (int i = 0; i < polyline.size(); ++i) {
        Vector2d point(polyline[i].x(), polyline[i].y());
        point = xform * point.homogeneous();
        cv_ring[i] = cv::Point(point(0) * std::pow(2, SHIFT), point(1) * std::pow(2, SHIFT));
    }
    points.push_back(cv_ring);
    cv::polylines(mat, points, false, cv::Scalar(value(0), value(1), value(2), value(3)), 1, cv::LINE_8, SHIFT);
}

void line2(cv::Mat &img, const cv::Point &start, const cv::Point &end, const cv::Scalar &c1, const cv::Scalar &c2) {
    cv::LineIterator iter(img, start, end, cv::LINE_8);
    for (int i = 0; i < iter.count; i++, iter++) {
        cv::Point pos = iter.pos();
        if (pos.x < 0 || pos.y < 0 || pos.x >= img.cols || pos.y >= img.rows) {
            continue;
        }
        double alpha = double(i) / iter.count;
        img(cv::Rect(iter.pos(), cv::Size(1, 1))) = c1 * (1.0 - alpha) + c2 * alpha;
    }
}

} // namespace h2o
