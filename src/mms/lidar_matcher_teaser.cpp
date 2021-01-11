/*
 * Created Date: Sunday, January 10th 2021, 9:27:20 pm
 * Author: han
 * 利用 teaserpp 进行粗匹配
 * Copyright (c) 2021 Vrlab
 */

#include "lidar_matcher_proxy.h"

#include <open3d/geometry/KDTreeFlann.h>
#include <teaser/registration.h>

namespace h2o {
Matrix4d LiDARMatcherProxy::match_teaser(const PointCloudPtr &pcd_i, const PointCloudPtr &pcd_j,
                                         const PointFeaturePtr &fpfh_i, const PointFeaturePtr &fpfh_j,
                                         double voxel_size) const {
    using namespace open3d;
    // STEP 1) Initial matching
    int nPti = int(pcd_i->points_.size());
    int nPtj = int(pcd_j->points_.size());
    geometry::KDTreeFlann feature_tree_i(*fpfh_i);
    geometry::KDTreeFlann feature_tree_j(*fpfh_j);
    std::vector<int> corresK;
    std::vector<double> dis;
    std::vector<std::pair<int, int>> corres;
    std::vector<std::pair<int, int>> corres_ij;
    std::vector<std::pair<int, int>> corres_ji;
    std::vector<int> i_to_j(nPti, -1);
    for (int j = 0; j < nPtj; j++) {
        feature_tree_i.SearchKNN(Eigen::VectorXd(fpfh_j->data_.col(j)), 1, corresK, dis);
        int i = corresK[0];
        if (i_to_j[i] == -1) {
            feature_tree_j.SearchKNN(Eigen::VectorXd(fpfh_i->data_.col(i)), 1, corresK, dis);
            int ij = corresK[0];
            i_to_j[i] = ij;
        }
        corres_ji.push_back(std::pair<int, int>(i, j));
    }
    for (int i = 0; i < nPti; i++) {
        if (i_to_j[i] != -1)
            corres_ij.push_back(std::pair<int, int>(i, i_to_j[i]));
    }
    int ncorres_ij = int(corres_ij.size());
    int ncorres_ji = int(corres_ji.size());
    for (int i = 0; i < ncorres_ij; ++i)
        corres.push_back(std::pair<int, int>(corres_ij[i].first, corres_ij[i].second));
    for (int j = 0; j < ncorres_ji; ++j)
        corres.push_back(std::pair<int, int>(corres_ji[j].first, corres_ji[j].second));

    // STEP 2) CROSS CHECK
    std::vector<std::pair<int, int>> corres_cross;
    std::vector<std::vector<int>> Mi(nPti), Mj(nPtj);
    int ci, cj;
    for (int i = 0; i < ncorres_ij; ++i) {
        ci = corres_ij[i].first;
        cj = corres_ij[i].second;
        Mi[ci].push_back(cj);
    }
    for (int j = 0; j < ncorres_ji; ++j) {
        ci = corres_ji[j].first;
        cj = corres_ji[j].second;
        Mj[cj].push_back(ci);
    }
    for (int i = 0; i < nPti; ++i) {
        for (size_t ii = 0; ii < Mi[i].size(); ++ii) {
            int j = Mi[i][ii];
            for (size_t jj = 0; jj < Mj[j].size(); ++jj) {
                if (Mj[j][jj] == i)
                    corres_cross.push_back(std::pair<int, int>(i, j));
            }
        }
    }

    Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, corres_cross.size());
    Eigen::Matrix<double, 3, Eigen::Dynamic> dst(3, corres_cross.size());
    for (int m = 0; m < corres_cross.size(); ++m) {
        src.col(m) = pcd_i->points_.at(corres_cross[m].first);
        dst.col(m) = pcd_j->points_.at(corres_cross[m].second);
    }

    teaser::RobustRegistrationSolver::Params param;
    param.noise_bound = voxel_size;
    param.estimate_scaling = false;
    teaser::RobustRegistrationSolver solver(param);

    auto results = solver.solve(src, dst);
    Matrix4d xform = Matrix4d::Identity();
    if (!results.valid) {
        return xform;
    }
    xform.block(0, 0, 3, 3) = results.rotation;
    xform.col(3).head(3) = results.translation;
    return xform;
}
} // namespace h2o
