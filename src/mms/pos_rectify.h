#pragma once

#include <base/common.h>

namespace h2o {
	class Pos_rec {
public:        
    std::tuple<Matrix3d, Vector3d> calculate_rotation(const std::string path_control_pts);
    void test_ceres();
};

}//namespace h2o