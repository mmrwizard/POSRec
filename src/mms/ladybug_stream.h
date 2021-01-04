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

    Matrix3d R_world = Matrix3d::Identity();
    Quaterniond q_world = Quaterniond::Identity();
    Vector3d t_world = Vector3d::Zero();
    Matrix3d R_body = Matrix3d::Identity();
    Quaterniond q_body = Quaterniond::Identity();
    Vector3d t_body = Vector3d::Zero();

    std::string path;

    // angle: (theta, phi)
    // theta -> [0, 2pi]，影像从左到右为 2pi->pi->0
    // phi -> [-pi/2, pi/2] (type = 0)
    // phi -> tan[-pi/4, pi/4] (type = 1)
    // width = 2 * height
    Vector2d to_pixel(const Vector2d &angle) const;
    Vector2d to_angle(const Vector2d &pixel) const;

    Vector3d to_local(const Vector3d &world) const;
    Vector3d to_world(const Vector3d &local) const;

    Vector2d to_angle(const Vector3d &local) const;

    bool is_valid(const Vector2d &angle) const;
};

class LadybugStream {
public:
    LadybugStream(const std::string &dir, const std::string &name, const std::string &timestamp, const Pos &pos);

    // 0 - sphere, 1 - cylinder
    void set_image_type(int type);
    void set_image_type(const std::string &path);

    // 设置到 imu 坐标系的旋转平移
    void set_body(const std::string &path);
    void set_body(const Vector3d &r, const Vector3d &t);

    // 在指定的影像上着色，不能读取新数据
    Vector3b colorize_on_images(const Vector3d &X, const std::vector<uint32_t> &images) const;

    // 存储这些影像
    void cache_images(const std::vector<uint32_t> &images);

    auto get_image_infos() const { return images_; }

protected:
    void debug_trajectory();

protected:
    const Pos &pos_;

    std::vector<LadybugImage> images_;

    Quaterniond q_body_;
    Vector3d t_body_;

    using KDTreeTrajectory = nanoflann::KDTreeAdaptor<MatrixXd, 3>;
    MatrixXd trajectory_;
    std::shared_ptr<KDTreeTrajectory> kdtree_trajectory_;

    using CacheImages = Cache<uint32_t, cv::Mat, std::mutex>;
    std::shared_ptr<CacheImages> mat_caches_;
};

using LadybugStreamPtr = std::shared_ptr<LadybugStream>;
} // namespace h2o
