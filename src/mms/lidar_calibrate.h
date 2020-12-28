/*
 * Created Date: Wednesday, December 23rd 2020, 3:55:13 pm
 * Author: Han
 * 检校 LiDAR 和 IMU 之间的误差
 * Copyright (c) 2020 Vrlab
 */

#pragma once

#include <base/common.h>

#include "pos.h"

namespace ceres {
class Problem;
namespace internal {
class ResidualBlock;
} // namespace internal
typedef internal::ResidualBlock *ResidualBlockId;
} // namespace ceres

namespace h2o {
class LiDARCalibrate {
public:
    struct GCP {
        // LiDAR 点获取时间
        double gpstime;

        // LiDAR 点在世界坐标系
        Vector3d p_lidar;

        // LiDAR 点对饮的控制点坐标
        Vector3d p_gcp;

        // LiDAR 点所在的局部坐标系
        Vector3d p_lidar_local;

        // 从 Pos 文件中内插的信息
        Quaterniond q_pos;
        Vector3d t_pos;

        static std::vector<GCP> load(const std::string &path);
    };

    struct TPS {
        double t1, t2;
        Vector3d p_lidar1, p_lidar2;
        Vector3d p_local1, p_local2;
        Quaterniond q_pos1, q_pos2;
        Vector3d t_pos1, t_pos2;

        static std::vector<TPS> load(const std::string &path);
    };

    struct Config {
        Config() {}
        bool with_translate = false;

        double std_body_angle = 3.0 * DEG2RAD;
        double std_lidar_point = 0.2;
    };

public:
    LiDARCalibrate(const Pos &pos, const Config &config = Config());
    LiDARCalibrate(const Pos &pos1, const Pos &pos2, const Config &config = Config());
    ~LiDARCalibrate() {}

public:
    void set_body(const Quaterniond &q, const Vector3d &t);
    void set_gcp(const std::vector<GCP> &gcps);
    void set_tps(const std::vector<TPS> &tps);

    std::tuple<Quaterniond, Vector3d> calibrate();

protected:
    void evaluate();

protected:
    // 初始参数
    Quaterniond q_body_;
    Vector3d t_body_;

    // 需要检校的参数
    Quaterniond dq_body_;
    Vector3d dt_body_;

    std::vector<GCP> gcps_;
    std::vector<TPS> tps_;

    std::vector<ceres::ResidualBlockId> blocks_gcps_;
    std::vector<ceres::ResidualBlockId> blocks_tps_;
    std::shared_ptr<ceres::Problem> problem_;

    const Pos &pos_;
    const Pos &pos2_;
    bool has_pos2_;

    Config param_;
};

} // namespace h2o
