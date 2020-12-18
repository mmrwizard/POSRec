/*
 * Filename: /home/han/Documents/LinSAR/src/base/spatial.h
 * Path: /home/han/Documents/LinSAR/src/base
 * Created Date: Friday, January 10th 2020, 5:26:05 pm
 * Author: han
 * 空间坐标转换
 * Copyright (c) 2020 Vrlab
 */
#pragma once

#include "common.h"
#include "geo_image.h"

#include <proj.h>

class OGRCoordinateTransformation;

namespace h2o {

// 把 wkt1 格式的投影转换成 proj 支持的格式
std::string wkt1_gdal_to_proj(const std::string &wkt);
std::string proj_to_wkt(const std::string &proj);
bool is_srs_same(const std::string &wkt1, const std::string &wkt2);
bool is_srs_geographic(const std::string &wkt);

// 平面坐标系统转换
class Spatial {
public:
    Spatial(const std::string &src_wkt, const std::string &dst_wkt);
    Spatial(const std::string &op);
    ~Spatial();

    Vector2d forward(const Vector2d &point) const;
    Vector2d backward(const Vector2d &point) const;
    std::vector<Vector2d> forward(const std::vector<Vector2d> &points) const;
    std::vector<Vector2d> backward(const std::vector<Vector2d> &points) const;

    std::string get_src_wkt() const;
    std::string get_dst_wkt() const;

private:
    PJ *pj_;
    bool identity_;
};

// 高程租表系统转换
class SpatialEgm96 {
public:
    SpatialEgm96(const std::string &wkt);
    ~SpatialEgm96();

    // 输入是输出都是 lon lat，单位为度
    Vector3d forward(const Vector3d &point) const;                             // egm 转 wgs
    Vector3d backward(const Vector3d &point) const;                            // wgs 转 egm
    std::vector<Vector3d> forward(const std::vector<Vector3d> &points) const;  // egm 转 wgs
    std::vector<Vector3d> backward(const std::vector<Vector3d> &points) const; // wgs 转 egm

private:
    PJ *pj_;
};

using SpatialPtr = std::shared_ptr<Spatial>;

void set_proj_directory(const std::string &path);
// 输出影像范围和分辨率根据输入影像自动计算,会记录边缘 mask
GeoImagePtr project_image(const GeoImagePtr &image, const Spatial &srs, int interpolation = cv::INTER_NEAREST);
// 输出影像范围和分辨率已经给定,会记录边缘 mask
void project_image(const GeoImagePtr &src_image, GeoImagePtr &dst_image, const Spatial &srs,
                   int interpolation = cv::INTER_NEAREST);

void warp_image(const GeoImagePtr &src_image, GeoImagePtr &dst_image, int interpolation = cv::INTER_LINEAR);
} // namespace h2o
