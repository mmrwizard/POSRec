/*
 * Created Date: Friday, December 18th 2020, 9:28:23 pm
 * Author: Han
 * 管理 pos 文件
 * Copyright (c) 2020 Vrlab
 */
#include "pos.h"
#include "rapidcsv.h"

#include <base/base.h>

namespace h2o {
void Pos::load_pos(const std::string &path) {
    rapidcsv::Document doc(string_utf8_to_fstream(path), rapidcsv::LabelParams(-1));

    std::vector<double> gpstime = doc.GetColumn<double>(0);
    std::vector<double> x = doc.GetColumn<double>(1);
    std::vector<double> y = doc.GetColumn<double>(2);
    std::vector<double> z = doc.GetColumn<double>(3);
    std::vector<double> yaw = doc.GetColumn<double>(4);
    std::vector<double> pitch = doc.GetColumn<double>(5);
    std::vector<double> roll = doc.GetColumn<double>(6);

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
    positions_.resize(nframes);
    quaternions_.resize(nframes);

    for (int i = 0; i < nframes; ++i) {
        gpstime[i] -= gpstime_offset_.x();
        x[i] -= position_offset_.x();
        y[i] -= position_offset_.y();
        z[i] -= position_offset_.z();

        yaw[i] = yaw[i] * DEG2RAD;
        pitch[i] = pitch[i] * DEG2RAD;
        roll[i] = pitch[i] * DEG2RAD;

        gpstimes_[i] = gpstime[i];
        positions_[i] = Vector3f(x[i], y[i], z[i]);

        Matrix3f R = yawpitchroll(yaw[i], pitch[i], roll[i]);
        quaternions_[i] = R;
    }
}

std::tuple<Vector3f, Matrix3f> Pos::get(double gpstime) const {
    int i0, i1;
    double u = (gpstime - gpstime_offset_.x()) / (gpstime_offset_.y() - gpstime_offset_.x()) * (gpstimes_.size() - 1);

    i0 = floor(u);
    i1 = i0 + 1;
    u = u - i0;

    Vector3f position;
    Matrix3f rotation;
    if (i0 < 0) {
        rotation = quaternions_.front().toRotationMatrix();
        position = positions_.front();
    } else if (i1 >= gpstimes_.size()) {
        rotation = quaternions_.back().toRotationMatrix();
        position = positions_.back();
    } else {
        rotation = quaternions_.at(i0).slerp(u, quaternions_.at(i1)).toRotationMatrix();
        position = positions_[i0] * (1 - u) + positions_[i1] * u;
    }

    return std::make_tuple(position, rotation);
}

BoundingBox3d Pos::get_bounds() const {
    BoundingBox3d bounds;
    for (const Vector3f &point : positions_) {
        bounds.extend(point.cast<double>() + position_offset_);
    }
    return bounds;
}

Matrix3f Pos::yawpitchroll(double yaw, double pitch, double roll) {
    // 采用的是海大数云自己定义的 yaw，和 IE 的 yaw 相差负数
    double cyaw, syaw, cpitch, spitch, croll, sroll;
    cyaw = cos(yaw);
    syaw = sin(yaw);
    cpitch = cos(pitch);
    spitch = sin(pitch);
    croll = cos(roll);
    sroll = sin(roll);

    Matrix3f Ryaw = Matrix3f::Identity(), Rpitch = Matrix3f::Identity(), Rroll = Matrix3f::Identity();

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

    Matrix3f R = Ryaw * Rpitch * Rroll;

    return R;
}
} // namespace h2o