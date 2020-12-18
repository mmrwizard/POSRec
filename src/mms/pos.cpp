/*
 * Created Date: Friday, December 18th 2020, 9:28:23 pm
 * Author: Han
 * 管理 pos 文件
 * Copyright (c) 2020 Vrlab
 */
#include "pos.h"
#include "spline/rapidcsv.h"
#include "spline/squad.hpp"

#include <base/base.h>

namespace h2o {
void Pos::load_pos(const std::string &path) {
    rapidcsv::Document doc(string_utf8_to_fstream(path), rapidcsv::LabelParams(-1));

    std::vector<double> gpstime = doc.GetColumn<double>(0);
    std::vector<double> x = doc.GetColumn<double>(1);
    std::vector<double> y = doc.GetColumn<double>(1);
    std::vector<double> z = doc.GetColumn<double>(1);
    std::vector<double> head = doc.GetColumn<double>(1);
    std::vector<double> pitch = doc.GetColumn<double>(1);
    std::vector<double> roll = doc.GetColumn<double>(1);

    int nframes = gpstime.size();
    gpstime_offset_ = Vector2d(DBL_MAX, -DBL_MAX);
    position_offset_ = Vector3d::Constant(DBL_MAX);

    for (int i = 0; i < nframes; ++i) {
        gpstime_offset_.x() = std::min(gpstime[i], gpstime_offset_.x());
        gpstime_offset_.y() = std::max(gpstime[i], gpstime_offset_.y());
        position_offset_.x() = std::min(x[i], position_offset_.x());
        position_offset_.y() = std::min(y[i], position_offset_.y());
        position_offset_.z() = std::min(z[i], position_offset_.z());
    }

    gpstimes_.resize(nframes);
    positions_.resize(3, nframes);
    rotations_.resize(nframes);

    for (int i = 0; i < nframes; ++i) {
        gpstime[i] -= gpstime_offset_.x();
        x[i] -= position_offset_.x();
        y[i] -= position_offset_.y();
        z[i] -= position_offset_.z();

        head[i] = head[i] * DEG2RAD;
        pitch[i] = pitch[i] * DEG2RAD;
        roll[i] = pitch[i] * DEG2RAD;

        gpstimes_[i] = gpstime[i];
        positions_.col(i) = Vector3d(x[i], y[i], z[i]);

        Matrix3d R = yawpitchroll(head[i], pitch[i], roll[i]);
        rotations_[i] = R;
    }

    position_spline_ = Eigen::SplineFitting<Eigen::Spline<double, 3>>::Interpolate(positions_, 3, gpstimes_);
}

inline Matrix3d Pos::yawpitchroll(double yaw, double pitch, double roll) {

    double cyaw, syaw, cpitch, spitch, croll, sroll;
    cyaw = cos(yaw);
    syaw = sin(yaw);
    cpitch = cos(pitch);
    spitch = sin(pitch);
    croll = cos(roll);
    sroll = sin(roll);

    Matrix3d Ryaw = Matrix3d::Identity(), Rpitch = Matrix3d::Identity(), Rroll = Matrix3d::Identity();

    Ryaw(0, 0) = cyaw;
    Ryaw(0, 1) = syaw;
    Ryaw(1, 0) = -syaw;
    Ryaw(1, 1) = cyaw;

    Rpitch(1, 1) = cpitch;
    Rpitch(1, 2) = -spitch;
    Rpitch(2, 1) = spitch;
    Rpitch(2, 2) = cpitch;

    Rroll(0, 0) = croll;
    Rroll(0, 2) = sroll;
    Rroll(2, 0) = -sroll;
    Rroll(2, 2) = croll;

    Matrix3d R = Ryaw * Rpitch * Rroll;

    return R;
}
} // namespace h2o