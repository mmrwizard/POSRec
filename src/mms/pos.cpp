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
    std::vector<double> gpstime;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<double> yaw;
    std::vector<double> pitch;
    std::vector<double> roll;
    std::vector<double> vx;
    std::vector<double> vy;
    std::vector<double> vz;

    if (get_extension_name(path) == ".csv") {
        rapidcsv::Document doc;

        doc = rapidcsv::Document(string_utf8_to_fstream(path), rapidcsv::LabelParams(-1));
        gpstime = doc.GetColumn<double>(0);
        x = doc.GetColumn<double>(1);
        y = doc.GetColumn<double>(2);
        z = doc.GetColumn<double>(3);
        yaw = doc.GetColumn<double>(4);
        pitch = doc.GetColumn<double>(5);
        roll = doc.GetColumn<double>(6);
        vx = doc.GetColumn<double>(7);
        vy = doc.GetColumn<double>(8);
        vz = doc.GetColumn<double>(9);
    } else if (get_extension_name(path) == ".pos") {
        std::ifstream posfile(string_utf8_to_fstream(path));
        std::string line;
        while (getline(posfile, line) && posfile.good()) {
            auto new_end =
                std::unique(begin(line), end(line), [](char lhs, char rhs) { return (lhs == rhs) && (lhs == ' '); });
            line.erase(new_end, end(line));
            std::vector<std::string> chunks = string_split(line, " ");
            if (chunks.empty()) {
                continue;
            }

            // 判断第一个点是不是数字
            try {
                double first_value = std::stod(chunks.front());
            } catch (const std::exception &) {
                continue;
            }

            gpstime.push_back(std::stod(chunks[0]));
            x.push_back(std::stod(chunks[1]));
            y.push_back(std::stod(chunks[2]));
            z.push_back(std::stod(chunks[3]));
            yaw.push_back(std::stod(chunks[4]));
            pitch.push_back(std::stod(chunks[5]));
            roll.push_back(std::stod(chunks[6]));
            vx.push_back(std::stod(chunks[7]));
            vy.push_back(std::stod(chunks[8]));
            vz.push_back(std::stod(chunks[9]));
        }
    }

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
    velocities_.resize(nframes);

    for (int i = 0; i < nframes; ++i) {
        gpstime[i] -= gpstime_offset_.x();
        x[i] -= position_offset_.x();
        y[i] -= position_offset_.y();
        z[i] -= position_offset_.z();

        yaw[i] = yaw[i] * DEG2RAD;
        pitch[i] = pitch[i] * DEG2RAD;
        roll[i] = pitch[i] * DEG2RAD;

        gpstimes_[i] = gpstime[i];
        positions_[i] = Vector3d(x[i], y[i], z[i]);

        Matrix3f R = ypr_to_rotation(yaw[i], pitch[i], roll[i]);
        quaternions_[i] = R.cast<double>();

        velocities_[i] = Vector3d(vx[i], vy[i], vz[i]);
    }
}

std::tuple<Vector3f, Matrix3f> Pos::get(double gpstime) const {
    auto [p, R] = getd(gpstime);
    return std::make_tuple(p.cast<float>(), R.cast<float>());
}

std::tuple<Vector3d, Matrix3d> Pos::getd(double gpstime) const {
    int i0, i1;
    int n = gpstimes_.size();
    double u = (gpstime - gpstime_offset_.x()) / (gpstime_offset_.y() - gpstime_offset_.x()) * (n - 1);

    i0 = floor(u);
    i1 = i0 + 1;
    u = u - i0;

    Vector3d position;
    Matrix3d rotation;
    if (i0 < 0) {
        rotation = quaternions_.front().toRotationMatrix();
        position = positions_.front();
    } else if (i1 >= n) {
        rotation = quaternions_.back().toRotationMatrix();
        position = positions_.back();
    } else {
        rotation = Quaterniond(quaternions_[i0].coeffs() * (1.0 - u) + quaternions_[i1].coeffs() * u);
        position = positions_[i0] * (1 - u) + positions_[i1] * u;
    }

    return std::make_tuple(position, rotation);
}

Vector3d Pos::get_velocity(double gpstime) const {
    int i0, i1;
    int n = gpstimes_.size();
    double u = (gpstime - gpstime_offset_.x()) / (gpstime_offset_.y() - gpstime_offset_.x()) * (n - 1);

    i0 = floor(u);
    i1 = i0 + 1;
    u = u - i0;

    if (i0 < 0) {
        return velocities_.front();
    } else if (i1 >= n) {
        return velocities_.back();
    } else {
        return velocities_[i0] * (1 - u) + velocities_[i1] * u;
    }
}

double Pos::get_speed(double gpstime) const { return get_velocity(gpstime).norm(); }

BoundingBox3d Pos::get_bounds() const {
    BoundingBox3d bounds;
    for (const Vector3d &point : positions_) {
        bounds.extend(point + position_offset_);
    }
    return bounds;
}

Matrix3f Pos::ypr_to_rotation(double yaw, double pitch, double roll) {
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

Vector3f Pos::rotation_to_ypr(const Matrix3f &R) {
    Vector3f euler = R.eulerAngles(2, 0, 1);

    // 海达定义的第一个角是相反的
    euler(0) = -euler(0);

    if (euler(0) < 0.0) {
        euler(0) = euler(0) + 2.0 * M_PI;
    }

    return euler;
}
} // namespace h2o