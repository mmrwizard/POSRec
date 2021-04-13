/*
 * Created Date: Friday, December 18th 2020, 9:28:14 pm
 * Author: Han
 * 管理 pos 文件
 * Copyright (c) 2020 Vrlab
 */
#pragma once

#include <base/common.h>

namespace GeographicLib {
class Geocentric;
} // namespace GeographicLib

namespace h2o {
class Pos {
public:
    Pos(const std::string &path);
    Pos() = default;

    void load_pos(const std::string &path);
    std::tuple<Vector3d, Matrix3d> getd(double gpstime) const;
    Vector3d get_velocity(double gpstime) const;
    double get_speed(double gpstime) const;

    Vector3d get_position_offset() const { return position_offset_; }
    BoundingBox3d get_bounds() const;

    VectorXd get_gpstime();
    std::vector<Vector3d> get_positions();
    std::vector<int> get_quality();
    void calculate_rotation(const std::string &path_control_pts);
    std::vector<Vector2d> get_tunnel_period();
    void export_pos(const std::string &path);
    void read_controls(const std::string &path_control_pts);

public:
    static Matrix3d ypr_to_rotation(double yaw, double pitch, double roll);
    static Vector3d rotation_to_ypr(const Matrix3d &R);
    
protected:
    Vector2d gpstime_offset_;
    Vector3d position_offset_;

    Matrix3d rotation;
    Vector3d translation;

    VectorXd gpstimes_;

    std::vector<Vector3d> positions_;
    std::vector<Vector3d> velocities_;
    std::vector<Quaterniond> quaternions_;

    std::string proj_;

    std::vector<double> sd_n_;
    std::vector<double> sd_e_;
    std::vector<double> sd_h_;
    std::vector<double> sd_hd_;
    std::vector<double> sd_p_;
    std::vector<double> sd_r_;
    std::vector<int> q_;

    std::vector<std::vector<VectorXd>> control_points_sets;
    std::vector<Matrix3d> rotations;
    std::vector<Vector3d> translations;
    std::vector<Quaterniond> rotation_quaternions;

    std::vector<double> x_pointcloud_;
    std::vector<double> y_pointcloud_;
    std::vector<double> z_pointcloud_;
    std::vector<double> x_measure_;
    std::vector<double> y_measure_;
    std::vector<double> z_measure_;

    std::vector<double> utctime_;
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> z_;
    std::vector<double> yaw_;
    std::vector<double> pitch_;
    std::vector<double> roll_;
    std::vector<double> vx_;
    std::vector<double> vy_;
    std::vector<double> vz_;
};

} // namespace h2o
