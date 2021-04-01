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
Pos::Pos(const std::string &path) { load_pos(path); }

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
    std::vector<int> q;

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
            string_left_trim(&line);
            string_right_trim(&line);

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
            q.push_back(std::stod(chunks[16]));
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
        roll[i] = roll[i] * DEG2RAD;

        gpstimes_[i] = gpstime[i];
        positions_[i] = Vector3d(x[i], y[i], z[i]);

        Matrix3d R = ypr_to_rotation(yaw[i], pitch[i], roll[i]);
        quaternions_[i] = R;

        velocities_[i] = Vector3d(vx[i], vy[i], vz[i]);
    }
}

std::tuple<Matrix3d, Vector3d> Pos::calculate_rotation(const std::string &path_control_pts) {
    std::vector<double> gps_time;
    std::vector<double> x_pointcloud;
    std::vector<double> y_pointcloud;
    std::vector<double> z_pointcloud;
    std::vector<double> x_measure;
    std::vector<double> y_measure;
    std::vector<double> z_measure;

    std::ifstream cpt_file(string_utf8_to_fstream(path_control_pts));
    std::string line;
    while (std::getline(cpt_file, line) && cpt_file.good()) {
        string_left_trim(&line);
        string_right_trim(&line);

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
        gps_time.push_back(std::stod(chunks[0]));
        x_pointcloud.push_back(std::stod(chunks[1]));
        y_pointcloud.push_back(std::stod(chunks[2]));
        z_pointcloud.push_back(std::stod(chunks[3]));
        x_measure.push_back(std::stod(chunks[4]));
        y_measure.push_back(std::stod(chunks[5]));
        z_measure.push_back(std::stod(chunks[6]));
    }

    std::vector<Vector3d> control_pointcloud;
    std::vector<Vector3d> control_measure;
    int ncontrol = gps_time.size();
    control_pointcloud.resize(ncontrol);
    control_measure.resize(ncontrol);

    for (int i = 0; i < ncontrol; i++) {
        control_pointcloud[i] = Vector3d{x_pointcloud[i], y_pointcloud[i], z_pointcloud[i]};
        control_measure[i] = Vector3d{x_measure[i], y_measure[i], z_measure[i]};
    }

    Vector3d p1, p2;
    p1 = p2 = Vector3d::Zero();

    int N = control_pointcloud.size();
    for (int i = 0; i < N; i++) {
        p1 += control_pointcloud[i];
        p2 += control_measure[i];
    }

    p1 = Eigen::Vector3d(Eigen::Vector3d(p1) / N);
    p2 = Eigen::Vector3d(Eigen::Vector3d(p2) / N);
    // std::cout << p2 << std::endl;
    // 得到去中心坐标
    std::vector<Eigen::Vector3d> q1(N), q2(N);
    for (int i = 0; i < N; i++) {
        q1[i] = control_pointcloud[i] - p1;
        q2[i] = control_measure[i] - p2;
    }

    //计算需要进行奇异值分解的 W = sum(qi * qi’转置) compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++)
        W += Eigen::Vector3d(q1[i].x(), q1[i].y(), q1[i].z()) *
             Eigen::Vector3d(q2[i].x(), q2[i].y(), q2[i].z()).transpose();

    //对  W 进行SVD 奇异值分解
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // 计算旋转 和平移矩阵 R  和 t,  R= V *M* UT
    double det = (U * V.transpose()).determinant();
    Eigen::Matrix3d M;
    M << 1, 0, 0, 0, 1, 0, 0, 0, det;

    Matrix3d R_ = V * M * (U.transpose());

    // t =  p' - R * p
    Vector3d T_ = Eigen::Vector3d(p2.x(), p2.y(), p2.z()) - R_ * Eigen::Vector3d(p1.x(), p1.y(), p1.z());

    //计算POS
    Eigen::Vector3d pos_o(358078.98600000, 3437848.65000000, 948.28400000);
    Eigen::Vector3d pos_r(358079.31700000, 3437848.26600000, 945.92100000);
    auto chk = pos_r - T_ - R_ * pos_o;
    std::cout << chk.x() << "  " << chk.y() << "  " << chk.z() << std::endl;

    double x_sum = 0, y_sum = 0, z_sum = 0;
    for (int i = 0; i < control_measure.size(); i++) {
        x_sum += control_measure[i](0);
        y_sum += control_measure[i](1);
        z_sum += control_measure[i](2);
    }
    Eigen::Vector3d t(x_sum / control_measure.size(), y_sum / control_measure.size(), z_sum / control_measure.size());

    return std::make_tuple(R_, T_);
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
        rotation = quaternions_[i0].slerp(u, quaternions_[i1]);
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

VectorXd Pos::get_gpstime() { 
    VectorXd gpstimes;
    for (int i=0;i<gpstimes_.size();i++) {
        gpstimes[i] = gpstimes_[i] + gpstime_offset_.x();
    }
    return gpstimes; }

std::vector<Vector3d> Pos::get_positions() { return positions_; }
std::vector<int> Pos::get_quality() { return qualitys; }

BoundingBox3d Pos::get_bounds() const {
    BoundingBox3d bounds;
    for (const Vector3d &point : positions_) {
        bounds.extend(point + position_offset_);
    }
    return bounds;
}

Matrix3d Pos::ypr_to_rotation(double yaw, double pitch, double roll) {
    // 采用的是海大数云自己定义的 yaw，和 IE 的 yaw 相差负数
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

Vector3d Pos::rotation_to_ypr(const Matrix3d &R) {
    Vector3d euler = R.eulerAngles(2, 0, 1);

    // 海达定义的第一个角是相反的
    euler(0) = -euler(0);

    if (euler(0) < 0.0) {
        euler(0) = euler(0) + 2.0 * M_PI;
    }

    return euler;
}
} // namespace h2o
