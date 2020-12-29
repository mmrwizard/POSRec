/*
 * Created Date: Tuesday, December 29th 2020, 2:55:29 pm
 * Author: Han
 * Ladybug 影像
 * Copyright (c) 2020 Vrlab
 */
#pragma once
#include "pos.h"

#include <base/common.h>
#include <base/kdtree.h>
#include <base/lrucache.h>
#include <mutex>

namespace h2o {
struct LadybugImage {
    int width = 0;
    int height = 0;

    // 0 - sphere, 1 - cylinder
    // 当为 cylinder 模式，phi 角记录的是 tan 值
    int type = 1;

    double time = 0.0;
    double radius = 20.0;

    Quaterniond q_world;
    Vector3d t_world;
    Quaterniond q_body;
    Vector3d t_body;

    std::string path;

    // angle: (theta, phi)
    // theta -> [-pi, pi]
    // phi -> [-pi/2, pi/2] (type = 0)
    // phi -> tan[-pi/4, pi/4] (type = 1)
    // width = 2 * height
    Vector2d to_pixel(const Vector2d &angle) const;
    Vector2d to_angle(const Vector2d &pixel) const;

    Vector3d to_local(const Vector3d &world) const;
    Vector3d to_world(const Vector3d &local) const;
    
    Vector2d to_angle(const Vector3d &local) const;
    
    bool is_valid(const Vector2d& angle) const;
};

class LadybugStream {
public:
    LadybugStream(const std::string &dir, const std::string &name, const std::string &timestamp, const Pos &pos);

    // 0 - sphere, 1 - cylinder
    void set_image_type(int type);

    // 设置到 imu 坐标系的旋转平移
    void set_body(const std::string &path);
    void set_body(const Vector3d &r, const Vector3d &t);

    // 找到最近的测站，并投影
    Vector3b colorize(const Vector3d &X);

protected:
    const Pos &pos_;

    std::vector<LadybugImage> images_;

    Quaterniond q_body_;
    Vector3d t_body_;

    using KDTreeTrajectory = nanoflann::KDTreeAdaptor<MatrixXd, 3>;
    MatrixXd trajectory_;
    std::shared_ptr<KDTreeTrajectory> kdtree_trajectory_;

    Cache<uint32_t, cv::Mat, std::mutex> mat_caches_;
};

} // namespace h2o
