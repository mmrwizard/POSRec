/*
 * Filename: /home/han/Documents/LinSAR/src/base/spatial.cpp
 * Path: /home/han/Documents/LinSAR/src/base
 * Created Date: Friday, January 10th 2020, 5:26:10 pm
 * Author: han
 * 空间坐标转换
 * Copyright (c) 2020 Vrlab
 */
#include "spatial.h"
#include "imgproc.h"

#include <ogr_spatialref.h>
#include <ogr_srs_api.h>

#include <opencv2/core/eigen.hpp>

namespace h2o {
void set_proj_directory(const std::string &path) {
    std::vector<const char *> paths = {path.c_str()};
    proj_context_set_search_paths(PJ_DEFAULT_CTX, 1, paths.data());
}

Spatial::Spatial(const std::string &src_wkt, const std::string &dst_wkt) {
    identity_ = false;

    std::string src_proj, dst_proj;
    src_proj = wkt1_gdal_to_proj(src_wkt);
    dst_proj = wkt1_gdal_to_proj(dst_wkt);

    pj_ = proj_create_crs_to_crs(PJ_DEFAULT_CTX, src_proj.c_str(), dst_proj.c_str(), NULL);
    pj_ = proj_normalize_for_visualization(PJ_DEFAULT_CTX, pj_);

    if (int errornum = proj_errno(PJ_DEFAULT_CTX)) {
        CHECK(false) << proj_errno_string(errornum);
    }

    {
        PJ *src_pj = proj_create(PJ_DEFAULT_CTX, src_proj.c_str());
        PJ *dst_pj = proj_create(PJ_DEFAULT_CTX, dst_proj.c_str());

        int res = proj_is_equivalent_to(src_pj, dst_pj, PJ_COMP_EQUIVALENT);

        proj_destroy(src_pj);
        proj_destroy(dst_pj);

        if (res == TRUE) {
            identity_ = true;
            return;
        }
    }

    CHECK(pj_ != nullptr);
}

Spatial::Spatial(const std::string &op) {
    identity_ = false;
    pj_ = proj_create(PJ_DEFAULT_CTX, op.c_str());
    pj_ = proj_normalize_for_visualization(PJ_DEFAULT_CTX, pj_);
    if (int errornum = proj_errno(PJ_DEFAULT_CTX)) {
        CHECK(false) << proj_errno_string(errornum);
    }
    CHECK(pj_ != nullptr);
}

Spatial::~Spatial() {
    if (pj_ != NULL) {
        proj_destroy(pj_);
    }
}

Vector2d Spatial::forward(const Vector2d &point) const {
    if (identity_) {
        return point;
    }
    PJ_COORD pj_src = proj_coord(point.x(), point.y(), 0.0, 0.0);
    PJ_COORD pj_dst = proj_trans(pj_, PJ_FWD, pj_src);

    if (int errornum = proj_errno(PJ_DEFAULT_CTX)) {
        CHECK(false) << proj_errno_string(errornum);
    }

    return Vector2d(pj_dst.xy.x, pj_dst.xy.y);
}

Vector2d Spatial::backward(const Vector2d &point) const {
    if (identity_) {
        return point;
    }

    PJ_COORD pj_dst = proj_coord(point.x(), point.y(), 0.0, 0.0);
    PJ_COORD pj_src = proj_trans(pj_, PJ_INV, pj_dst);

    if (int errornum = proj_errno(PJ_DEFAULT_CTX)) {
        CHECK(false) << proj_errno_string(errornum);
    }

    return Vector2d(pj_src.xy.x, pj_src.xy.y);
}

std::vector<Vector2d> Spatial::forward(const std::vector<Vector2d> &points) const {
    if (identity_) {
        return points;
    }
    int num_points = points.size();
    std::vector<PJ_COORD> pj_coords(num_points);
    for (int i = 0; i < num_points; ++i) {
        pj_coords[i] = proj_coord(points[i].x(), points[i].y(), 0.0, 0.0);
    }
    int num_error = proj_trans_array(pj_, PJ_FWD, num_points, pj_coords.data());
    CHECK(num_error == 0);

    std::vector<Vector2d> results(num_points);
    for (int i = 0; i < num_points; ++i) {
        results[i].x() = pj_coords[i].xy.x;
        results[i].y() = pj_coords[i].xy.y;
    }
    return results;
}

std::vector<Vector2d> Spatial::backward(const std::vector<Vector2d> &points) const {
    if (identity_) {
        return points;
    }
    int num_points = points.size();
    std::vector<PJ_COORD> pj_coords(num_points);
    for (int i = 0; i < num_points; ++i) {
        pj_coords[i] = proj_coord(points[i].x(), points[i].y(), 0.0, 0.0);
    }
    int num_error = proj_trans_array(pj_, PJ_INV, num_points, pj_coords.data());
    CHECK(num_error == 0);

    std::vector<Vector2d> results(num_points);
    for (int i = 0; i < num_points; ++i) {
        results[i].x() = pj_coords[i].xy.x;
        results[i].y() = pj_coords[i].xy.y;
    }
    return results;
}

std::string Spatial::get_src_wkt() const {
    PJ *pj_src = proj_get_source_crs(PJ_DEFAULT_CTX, pj_);
    const char *wkt = proj_as_wkt(PJ_DEFAULT_CTX, pj_src, PJ_WKT1_GDAL, nullptr);
    CHECK(wkt != nullptr);

    std::string wkt_string(wkt);
    proj_destroy(pj_src);

    return wkt_string;
}

std::string Spatial::get_dst_wkt() const {
    PJ *pj_dst = proj_get_target_crs(PJ_DEFAULT_CTX, pj_);
    const char *wkt = proj_as_wkt(PJ_DEFAULT_CTX, pj_dst, PJ_WKT1_GDAL, nullptr);

    CHECK(wkt != nullptr);

    std::string wkt_string(wkt);
    proj_destroy(pj_dst);

    return wkt_string;
}

std::string wkt1_gdal_to_proj(const std::string &wkt) {
    PJ *pj = proj_create(PJ_DEFAULT_CTX, wkt.c_str());
    std::string proj = proj_as_proj_string(PJ_DEFAULT_CTX, pj, PJ_PROJ_5, nullptr);
    proj_destroy(pj);
    return proj;
}

std::string proj_to_wkt(const std::string &proj) {
    PJ *pj = proj_create(PJ_DEFAULT_CTX, proj.c_str());
    std::string wkt = proj_as_wkt(PJ_DEFAULT_CTX, pj, PJ_WKT1_GDAL, nullptr);
    proj_destroy(pj);
    return wkt;
}

SpatialEgm96::SpatialEgm96(const std::string &wkt) {
    std::string proj = wkt1_gdal_to_proj(wkt);
    proj += " +vunits=m";

    std::string proj_egm96 = proj + " +geoidgrids=egm96_15.gtx";

    pj_ = proj_create_crs_to_crs(PJ_DEFAULT_CTX, proj_egm96.c_str(), proj.c_str(), nullptr);
    pj_ = proj_normalize_for_visualization(PJ_DEFAULT_CTX, pj_);
}

SpatialEgm96::~SpatialEgm96() { proj_destroy(pj_); }

Vector3d SpatialEgm96::forward(const Vector3d &point) const {
    PJ_COORD coord = proj_coord(point.x(), point.y(), point.z(), 0);
    coord = proj_trans(pj_, PJ_FWD, coord);
    return Vector3d(coord.xyz.y, coord.xyz.x, coord.xyz.z);
}

Vector3d SpatialEgm96::backward(const Vector3d &point) const {
    // 不知道为什么 forward 和 backward 就算 normalize 了，顺序也不一样
    PJ_COORD coord = proj_coord(point.y(), point.x(), point.z(), 0);
    coord = proj_trans(pj_, PJ_INV, coord);
    return Vector3d(coord.xyz.x, coord.xyz.y, coord.xyz.z);
}

std::vector<Vector3d> SpatialEgm96::forward(const std::vector<Vector3d> &points) const {
    int npoints = points.size();
    int step = sizeof(Vector3d);

    if (points.empty()) {
        return std::vector<Vector3d>();
    }

    std::vector<Vector3d> trans = points;
    int nerror = proj_trans_generic(pj_, PJ_FWD, &trans[0].x(), step, npoints, &trans[0].y(), step, npoints,
                                    &trans[0].z(), step, npoints, nullptr, 0, 0);

    for (auto &point : trans) {
        std::swap(point.x(), point.y());
    }
    return trans;
}

std::vector<Vector3d> SpatialEgm96::backward(const std::vector<Vector3d> &points) const {
    int npoints = points.size();
    int step = sizeof(Vector3d);

    if (points.empty()) {
        return std::vector<Vector3d>();
    }

    std::vector<Vector3d> trans = points;
    int nerror = proj_trans_generic(pj_, PJ_INV, &trans[0].y(), step, npoints, &trans[0].x(), step, npoints,
                                    &trans[0].z(), step, npoints, nullptr, 0, 0);
    for (auto &point : trans) {
        std::swap(point.x(), point.y());
    }
    return trans;
}

GeoImagePtr project_image(const GeoImagePtr &image, const Spatial &srs, int interpolation) {
    BoundingBox2d srs_bounds = image->get_bounding_box();
    BoundingBox2d dst_bounds;

    for (int i = 0; i < 4; ++i) {
        Vector2d corner = srs_bounds.corner((BoundingBox2d::CornerType)i);
        corner = srs.forward(corner);
        dst_bounds.extend(corner);
    }

    if (dst_bounds.isEmpty()) {
        return GeoImagePtr();
    }

    int rows = image->get_size().y();
    int cols = image->get_size().x();

    double resolution = (dst_bounds.sizes().y() / rows + dst_bounds.sizes().x() / cols) / 2.0;
    rows = lround(dst_bounds.sizes().y() / resolution);
    cols = lround(dst_bounds.sizes().x() / resolution);

    Matrix23d xform;
    xform(0, 2) = dst_bounds.min().x();
    xform(1, 2) = dst_bounds.max().y();
    xform(0, 0) = resolution;
    xform(1, 1) = -resolution;
    xform(0, 1) = xform(1, 0) = 0.0;

    GeoImagePtr image_dst = std::make_shared<GeoImage>();
    image_dst->init_image(rows, cols, image->get_image().type(), xform);
    cv::Mat dst(rows, cols, image->get_image().type());

    cv::Mat src_mask = image->get_mask().empty()
                           ? cv::Mat(image->get_image().rows, image->get_image().cols, CV_8U, ValidDataDem)
                           : image->get_mask();
    cv::Mat dst_mask = cv::Mat::zeros(rows, cols, CV_8U);

#pragma omp parallel for
    for (int r = 0; r < rows; r += 256) {
        for (int c = 0; c < cols; c += 256) {
            int width = 256;
            int height = 256;
            width = std::min(width, cols - c);
            height = std::min(height, rows - r);
            cv::Rect roi(c, r, width, height);

            std::vector<Vector2d> points_dst = {Vector2d(0, 0), Vector2d(width, 0), Vector2d(width, height),
                                                Vector2d(0, height)};
            std::vector<Vector2d> points_src(4);
            for (int i = 0; i < 4; ++i) {
                Vector2d point = points_dst[i];
                point.x() += c;
                point.y() += r;
                point = image_dst->pixel_to_projection(point);
                point = srs.backward(point);
                point = image->projection_to_pixel(point);
                points_src[i] = point;
            }
            Matrix3d H = compute_homography(points_src, points_dst);
            cv::Mat cH;
            cv::eigen2cv(H, cH);

            cv::warpPerspective(image->get_image(), dst(roi), cH, cv::Size(width, height), interpolation,
                                cv::BORDER_CONSTANT, cv::Scalar::all(0));
            cv::warpPerspective(src_mask, dst_mask(roi), cH, cv::Size(width, height),
                                interpolation | cv::WARP_FILL_OUTLIERS, cv::BORDER_CONSTANT, cv::Scalar::all(0));
        }
    }

    dst_mask = dst_mask == 255;

    image_dst->set_image(dst);
    image_dst->set_mask(dst_mask);
    image_dst->set_wkt(srs.get_dst_wkt());

    return image_dst;
}

void project_image(const GeoImagePtr &src_image, GeoImagePtr &dst_image, const Spatial &srs, int interpolation) {
    int rows = dst_image->get_image().rows;
    int cols = dst_image->get_image().cols;
    cv::Mat dst = cv::Mat::zeros(rows, cols, src_image->get_image().type());

    cv::Mat src_mask = src_image->get_mask().empty()
                           ? cv::Mat(src_image->get_image().rows, src_image->get_image().cols, CV_8U, ValidDataDem)
                           : src_image->get_mask();
    cv::Mat dst_mask = cv::Mat::zeros(rows, cols, CV_8U);

#pragma omp parallel for
    for (int r = 0; r < rows; r += 256) {
        for (int c = 0; c < cols; c += 256) {
            int width = 256;
            int height = 256;
            width = std::min(width, cols - c);
            height = std::min(height, rows - r);
            cv::Rect roi(c, r, width, height);

            std::vector<Vector2d> points_dst = {Vector2d(0, 0), Vector2d(width, 0), Vector2d(width, height),
                                                Vector2d(0, height)};
            std::vector<Vector2d> points_src(4);
            for (int i = 0; i < 4; ++i) {
                Vector2d point = points_dst[i];
                point.x() += c;
                point.y() += r;
                point = dst_image->pixel_to_projection(point);
                point = srs.backward(point);
                point = src_image->projection_to_pixel(point);
                points_src[i] = point;
            }
            Matrix3d H = compute_homography(points_src, points_dst);
            cv::Mat cH;
            cv::eigen2cv(H, cH);

            cv::warpPerspective(src_image->get_image(), dst(roi), cH, cv::Size(width, height), interpolation,
                                cv::BORDER_CONSTANT, cv::Scalar::all(0));
            cv::warpPerspective(src_mask, dst_mask(roi), cH, cv::Size(width, height),
                                interpolation | cv::WARP_FILL_OUTLIERS, cv::BORDER_CONSTANT, cv::Scalar::all(0));
        }
    }
    dst_mask = dst_mask == 255;
    dst_image->set_image(dst);
    dst_image->set_mask(dst_mask);
    dst_image->set_wkt(srs.get_dst_wkt());
}

void warp_image(const GeoImagePtr &src_image, GeoImagePtr &dst_image, int interpolation /*= cv::INTER_LINEAR*/) {
    int rows = dst_image->get_image().rows;
    int cols = dst_image->get_image().cols;
    cv::Mat dst = cv::Mat::zeros(rows, cols, src_image->get_image().type());

    cv::Mat src_mask = src_image->get_mask().empty()
                           ? cv::Mat(src_image->get_image().rows, src_image->get_image().cols, CV_8U, ValidDataDem)
                           : src_image->get_mask();
    cv::Mat dst_mask = cv::Mat::zeros(rows, cols, CV_8U);

#pragma omp parallel for
    for (int r = 0; r < rows; r += 256) {
        for (int c = 0; c < cols; c += 256) {
            int width = 256;
            int height = 256;
            width = std::min(width, cols - c);
            height = std::min(height, rows - r);
            cv::Rect roi(c, r, width, height);

            std::vector<Vector2d> points_dst = {Vector2d(0, 0), Vector2d(width, 0), Vector2d(width, height),
                                                Vector2d(0, height)};
            std::vector<Vector2d> points_src(4);
            for (int i = 0; i < 4; ++i) {
                Vector2d point = points_dst[i];
                point.x() += c;
                point.y() += r;
                point = dst_image->pixel_to_projection(point);
                point = src_image->projection_to_pixel(point);
                points_src[i] = point;
            }
            Matrix3d H = compute_homography(points_src, points_dst);
            cv::Mat cH;
            cv::eigen2cv(H, cH);

            cv::warpPerspective(src_image->get_image(), dst(roi), cH, cv::Size(width, height), interpolation,
                                cv::BORDER_CONSTANT, cv::Scalar::all(0));
            cv::warpPerspective(src_mask, dst_mask(roi), cH, cv::Size(width, height),
                                interpolation | cv::WARP_FILL_OUTLIERS, cv::BORDER_CONSTANT, cv::Scalar::all(0));
        }
    }
    dst_mask = dst_mask == 255;
    dst_image->set_image(dst);
    dst_image->set_mask(dst_mask);
}

bool is_srs_same(const std::string &wkt1, const std::string &wkt2) {

    std::string src_proj, dst_proj;
    src_proj = wkt1_gdal_to_proj(wkt1);
    dst_proj = wkt1_gdal_to_proj(wkt2);

    PJ *src_pj = proj_create(PJ_DEFAULT_CTX, src_proj.c_str());
    PJ *dst_pj = proj_create(PJ_DEFAULT_CTX, dst_proj.c_str());

    int res = proj_is_equivalent_to(src_pj, dst_pj, PJ_COMP_EQUIVALENT);

    proj_destroy(src_pj);
    proj_destroy(dst_pj);

    if (res == TRUE) {
        return true;
    } else {
        return false;
    }
}

bool is_srs_geographic(const std::string &wkt) {
    PJ *pj = proj_create(PJ_DEFAULT_CTX, wkt.c_str());
    PJ_TYPE type = proj_get_type(pj);
    bool is_geographic = type == PJ_TYPE_GEOGRAPHIC_2D_CRS || type == PJ_TYPE_GEOGRAPHIC_3D_CRS;
    proj_destroy(pj);
    return is_geographic;
}

} // namespace h2o
