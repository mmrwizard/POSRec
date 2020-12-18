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

protected:
    inline Matrix3d yawpitchroll(double yaw, double pitch, double roll);

protected:
    Vector2d gpstime_offset_;
    Vector3d position_offset_;

    VectorXd gpstimes_;
    MatrixXd positions_;
    std::vector<Quaterniond> rotations_;

    interpol::squad::QSpline rotation_spline_;
    Eigen::Spline<double, 3> position_spline_;
};

} // namespace h2o
