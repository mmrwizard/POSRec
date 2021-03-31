#pragma once

#include <base/common.h>

namespace h2o {
	class Pos_rec {

    public:        
    void calculate_rotation(const std::string path_control_pts);
    void test_ceres();
 
    private:
    std::vector<Eigen::Vector3d> _T;
    std::vector<Eigen::Matrix3d> _R;

    };

}//namespace h2o