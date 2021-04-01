#pragma once

#include <base/common.h>

namespace h2o {
    class Pos_rec {
public:  
    static std::tuple<Matrix3d, Vector3d> calculate_rotation(const std::string &path_control_pts);
   // void test_ceres();
    Vector2d get_tunnel_time();

protected:
    Vector3d T_;
    Matrix3d R_;
    };

}//namespace h2o