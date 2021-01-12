/*
 * Created Date: Thursday, January 7th 2021, 3:59:26 pm
 * Author: Han
 * 进行两站点云的匹配
 * Copyright (c) 2021 Vrlab
 */
#pragma once

#include <base/common.h>

#include "lidar_calibrate.h"

namespace h2o {

struct LiDARMatchParam {
    double voxel_size = 0.05;

    double icp_radius = 1.0;                // 用此范围内的点进行 ICP 配准
    double icp_max_keypoint_distance = 0.5; // 超过此距离的特征点就不进行 ICP 了
    double icp_max_distance = 0.1;          // 超过此距离的点认为是粗差在 ICP 中不考虑
    int icp_max_points = 1000;              // 一个点云中最多只用这多点
    int icp_t_need_points = 50;             // 至少要有这么多重叠点
    double icp_t_overlap = 0.8;             // 重叠率至少要大于此阈值
    bool icp_use_initial = true;            // 是否额外进行粗配准

    double ne_radius = 0.3; // 计算法线的半径
    double ne_max_knn = 30; // 最多用于法线估算的点

    double key_radius = 0.5; // 计算特征点的半径，最好和 icp 特征点的最大半径相等
    double key_gamma = 0.5;  // 计算特征点的阈值，越小该范围越显著

    bool show_match = false;
};

using LiDARMatches = std::vector<LiDARCalibrate::TPS>;
LiDARMatchParam lidar_load_config(const std::string &path);
LiDARMatches lidar_match(const std::string &las_i, const std::string &las_j, const std::string &pos_i,
                         const std::string &pos_j, const LiDARMatchParam &param = LiDARMatchParam());
} // namespace h2o
