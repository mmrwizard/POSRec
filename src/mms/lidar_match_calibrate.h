/*
 * Created Date: Thursday, January 14th 2021, 7:49:17 pm
 * Author: Han
 * 匹配和检校一起运行，可以重用特征点 ICP 等信息
 * Copyright (c) 2021 Vrlab
 */
#pragma once

#include "lidar_calibrate.h"
#include "lidar_matcher.h"
#include "open3d_fwd.h"
#include "pos.h"

namespace h2o {
class LiDARMatchCalibrate {
public:
    LiDARMatchCalibrate(const std::string &las_i, const std::string &las_j, const std::string &pos_i,
                        const std::string &pos_j, const LiDARMatchParam &param_mat = LiDARMatchParam(),
                        const LiDARCalibrate::Config &param_cal = LiDARCalibrate::Config());

    LiDARMatches match();

protected:
    std::tuple<PointCloudPtr, std::vector<double>> read_las(const std::string &path);
    void downsample_voxel(PointCloudPtr &pcd, std::vector<double> &gps);
    void compute_normal(PointCloudPtr &pcd, const std::vector<double> &gps, const Pos &pos);
    PointFeaturePtr extract_feature(const PointCloudPtr &pcd);
    std::tuple<PointCloudPtr, PointIndices> detect_keypoints(const PointCloudPtr &pcd);
    LiDARMatches do_match(const PointCloudPtr &pcd_i, const PointCloudPtr &key_i, const std::vector<double> &gps_i,
                          const PointFeaturePtr &fpfh_i, const PointCloudPtr &pcd_j, const PointCloudPtr &key_j,
                          const std::vector<double> &gps_j, const PointFeaturePtr &fpfh_j);

protected:
    PointCloudPtr pcd_i_;
    PointCloudPtr pcd_j_;

    std::vector<double> gps_i_;
    std::vector<double> gps_j_;

    PointCloudPtr keys_i_;
    PointCloudPtr keys_j_;

    PointIndices key_indices_i_;
    PointIndices key_indices_j_;

    PointFeaturePtr fpfh_i_;
    PointFeaturePtr fpfh_j_;

    Pos pos_i_;
    Pos pos_j_;

    Vector3d offset_;

    LiDARMatchParam param_mat_;
    LiDARCalibrate::Config param_cal_;
};

} // namespace h2o
