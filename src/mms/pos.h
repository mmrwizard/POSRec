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
    std::tuple<Matrix3d, Vector3d> calculate_rotation(const std::string &path_control_pts);

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
    std::vector<int> qualitys;

    std::string proj_;
};

} // namespace h2o
