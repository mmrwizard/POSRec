/*
 * Created Date: Friday, December 18th 2020, 9:28:14 pm
 * Author: Han
 * 管理 pos 文件
 * Copyright (c) 2020 Vrlab
 */
#pragma once

#include <base/common.h>

#include "spline/squad.hpp"
#include <unsupported/Eigen/Splines>

namespace h2o {
class Pos {
public:
    void load_pos(const std::string &path);
    std::tuple<Vector3f, Matrix3f> get(double gpstime) const;

    Vector3d get_position_offset() const { return position_offset_; }
    BoundingBox3d get_bounds() const;

public:
    static Matrix3f yawpitchroll(double yaw, double pitch, double roll);

protected:
    Vector2d gpstime_offset_;
    Vector3d position_offset_;

    VectorXf gpstimes_;

    std::vector<Vector3f> positions_;
    std::vector<Quaternionf> quaternions_;
};

} // namespace h2o
