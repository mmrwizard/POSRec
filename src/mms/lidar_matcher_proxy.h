/*
 * Created Date: Thursday, January 7th 2021, 3:59:26 pm
 * Author: Han
 * 进行两站点云的匹配，这个头文件不做外部接口
 * Copyright (c) 2021 Vrlab
 */
#pragma once

#include "lidar_matcher.h"
#include "pos.h"

#include <open3d/Open3D.h>

namespace h2o {
using PointCloud = open3d::geometry::PointCloud;
using PointCloudPtr = std::shared_ptr<open3d::geometry::PointCloud>;
using PointIndices = std::vector<int>;
using PointFeature = open3d::pipelines::registration::Feature;
using PointFeaturePtr = std::shared_ptr<PointFeature>;

struct PointCloudWithData {
    PointCloudPtr pcd;
    std::vector<double> gpstime;
    Vector3d offset;

    PointCloudPtr keys;
    PointIndices ikeys;
    PointFeaturePtr feats;
};

class LiDARMatcherProxy {
public:
    LiDARMatcherProxy(const std::string &path_las_i, const std::string &path_las_j, const std::string &path_pos_i,
                      const std::string &path_pos_j);
    LiDARMatcherProxy(const std::string &path_las);

public:
    void set_param(const LiDARMatchParam &param) { param_ = param; }

    LiDARMatches match();

protected:
    PointCloudWithData read_las(const std::string &path, const Vector3d &offset);
    PointCloudWithData downsample_voxel(const PointCloudWithData &pcdwt);
    void compute_normal(const PointCloudWithData &pcd_data, const Pos &pos);
    PointFeaturePtr extract_feature(const PointCloudPtr &pcd);
    std::tuple<PointCloudPtr, PointIndices> detect_keypoints(const PointCloudPtr &pcd);
    LiDARMatches do_match(const PointCloudPtr &pcd_i, const PointCloudPtr &key_i, const std::vector<double>& gps_i,
                          const PointFeaturePtr &fpfh_i, const PointCloudPtr &pcd_j, const PointCloudPtr &key_j,
                          const std::vector<double> &gps_j, const PointFeaturePtr &fpfh_j);

protected:
    PointCloudWithData pcd_data_i_;
    PointCloudWithData pcd_data_j_;

    Pos pos_i_;
    Pos pos_j_;

    LiDARMatchParam param_;
};

} // namespace h2o
