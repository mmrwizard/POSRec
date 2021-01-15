/*
 * Created Date: Thursday, January 14th 2021, 7:49:17 pm
 * Author: Han
 * 匹配和检校一起运行，可以重用特征点 ICP 等信息
 * Copyright (c) 2021 Vrlab
 */
#include "lidar_match_calibrate.h"

#include <base/base.h>
#include <laszip/laszip_api.h>
#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/FastGlobalRegistration.h>

namespace h2o {
LiDARMatchCalibrate::LiDARMatchCalibrate(const std::string &las_i, const std::string &las_j, const std::string &pos_i,
                                         const std::string &pos_j, const LiDARMatchParam &param_mat,
                                         const LiDARCalibrate::Config &param_cal)
    : param_cal_(param_cal), param_mat_(param_mat_) {
    {
        LasHeader header = pc_read_header(las_i);
        offset_ = header.offset;
    }
    LOG(INFO) << "Loading...";
    std::tie(pcd_i_, gps_i_) = read_las(las_i);
    std::tie(pcd_j_, gps_j_) = read_las(las_i);

    pos_i_ = Pos(pos_i);
    pos_j_ = Pos(pos_j);

    LOG(INFO) << "Downsampling...";
    downsample_voxel(pcd_i_, gps_i_);
    downsample_voxel(pcd_j_, gps_j_);

    LOG(INFO) << "Detecting keypoints...";
    std::tie(keys_i_, key_indices_i_) = detect_keypoints(pcd_i_);
    std::tie(keys_j_, key_indices_j_) = detect_keypoints(pcd_j_);

    if (param_mat_.icp_use_initial) {
        LOG(INFO) << "Computing normal...";
        compute_normal(pcd_i_, gps_i_, pos_i_);
        compute_normal(pcd_j_, gps_j_, pos_j_);

        LOG(INFO) << "Computing features...";
        fpfh_i_ = extract_feature(pcd_i_);
        fpfh_j_ = extract_feature(pcd_j_);
    }
}

LiDARMatches LiDARMatchCalibrate::match() {
    LiDARMatches matches = do_match(pcd_i_, keys_i_, gps_i_, fpfh_i_, pcd_j_, keys_j_, gps_j_, fpfh_j_);
    return LiDARMatches();
}

std::tuple<PointCloudPtr, std::vector<double>> LiDARMatchCalibrate::read_las(const std::string &path) {
    laszip_POINTER lasreader;
    laszip_create(&lasreader);

    std::string local = string_utf8_to_fstream(path);
    laszip_BOOL compressed;
    laszip_open_reader(lasreader, local.c_str(), &compressed);

    laszip_header *lasheader;
    laszip_get_header_pointer(lasreader, &lasheader);
    laszip_I64 num_points = (lasheader->number_of_point_records ? lasheader->number_of_point_records
                                                                : lasheader->extended_number_of_point_records);
    laszip_point *point;
    laszip_get_point_pointer(lasreader, &point);

    PointCloudPtr pcd = std::make_shared<open3d::geometry::PointCloud>();
    pcd->points_.resize(num_points);
    std::vector<double> gpstime(num_points);

    for (int i = 0; i < num_points; ++i) {
        if (laszip_read_point(lasreader)) {
            CHECK(false) << "Read point error!";
        }
        double x = (double)point->X * lasheader->x_scale_factor + lasheader->x_offset;
        double y = (double)point->Y * lasheader->y_scale_factor + lasheader->y_offset;
        double z = (double)point->Z * lasheader->z_scale_factor + lasheader->z_offset;

        x -= offset_.x();
        y -= offset_.y();
        z -= offset_.z();

        double intensity = point->intensity / 65535.0;

        pcd->points_[i] = Vector3d(x, y, z);
        gpstime[i] = point->gps_time;
    }

    laszip_close_reader(lasreader);
    laszip_destroy(lasreader);

    return std::make_tuple(pcd, gpstime);
}

void LiDARMatchCalibrate::downsample_voxel(PointCloudPtr &pcd, std::vector<double> &gps) {
    auto bounds = pcd->GetAxisAlignedBoundingBox();

    double inverse_voxel = 1.0 / param_mat_.voxel_size;

    // 整个 volumn 的行列号大小
    Vector3i volumn = (bounds.GetExtent() * inverse_voxel).array().cast<int>();
    volumn += Vector3i::Ones();
    Vector3i div_mul = Eigen::Vector3i(1, volumn[0], volumn[0] * volumn[1]);

    std::unordered_set<uint64_t> indices_voxels;
    std::vector<size_t> indices;

    for (int i = 0; i < pcd->points_.size(); ++i) {
        Vector3d point = pcd->points_.at(i);
        point -= bounds.GetMinBound();
        point *= inverse_voxel;

        uint64_t ijk0 = floor(point.x());
        uint64_t ijk1 = floor(point.y());
        uint64_t ijk2 = floor(point.z());

        uint64_t idx = ijk0 * (uint64_t)div_mul[0] + ijk1 * (uint64_t)div_mul[1] + ijk2 * (uint64_t)div_mul[2];

        auto pair = indices_voxels.insert(idx);
        if (pair.second) {
            indices.push_back(size_t(i));
        }
    }

    std::vector<double> voxel_gpstime(indices.size());
    for (int i = 0; i < indices.size(); ++i) {
        voxel_gpstime[i] = gps[indices[i]];
    }
    auto voxel_pcd = pcd->SelectByIndex(indices);

    pcd = voxel_pcd;
    gps = voxel_gpstime;
}

void LiDARMatchCalibrate::compute_normal(PointCloudPtr &pcd, const std::vector<double> &gps, const Pos &pos) {
    // 计算法线
    if (!pcd->HasNormals()) {
        auto search_param = open3d::geometry::KDTreeSearchParamHybrid(param_mat_.ne_radius, param_mat_.ne_max_knn);
        pcd->EstimateNormals(search_param);
    }

    // 法线朝向轨迹位置
    Vector3d pos_offset = pos.get_position_offset();
    for (int i = 0; i < gps.size(); ++i) {
        double t = gps[i];
        Vector3d t_pos = std::get<0>(pos.getd(t)) + pos_offset - offset_;
        Vector3d point = pcd->points_[i];
        point = t_pos - point; // 点云到pos位置得朝向
        if (pcd->normals_[i].dot(point) < 0) {
            pcd->normals_[i] = -pcd->normals_[i];
        }
    }
}

PointFeaturePtr LiDARMatchCalibrate::extract_feature(const PointCloudPtr &pcd) {
    auto search_param =
        open3d::geometry::KDTreeSearchParamHybrid(param_mat_.ne_radius * 2.0, param_mat_.ne_max_knn * 4);
    auto feature = open3d::pipelines::registration::ComputeFPFHFeature(*pcd, search_param);
    return feature;
}

std::tuple<PointCloudPtr, PointIndices> LiDARMatchCalibrate::detect_keypoints(const PointCloudPtr &pcd) {
    PointCloudPtr keypoints = open3d::geometry::keypoint::ComputeISSKeypoints(
        *pcd, param_mat_.key_radius, param_mat_.key_radius, param_mat_.key_gamma, param_mat_.key_gamma);

    PointIndices indices(keypoints->points_.size());
    open3d::geometry::KDTreeFlann kdtree(*pcd);
    for (int i = 0; i < keypoints->points_.size(); ++i) {
        PointIndices knn;
        std::vector<double> sq_distances;
        CHECK(kdtree.SearchKNN(keypoints->points_[i], 1, knn, sq_distances) == 1 &&
              std::abs(sq_distances.front()) < 1e-3);
        indices[i] = knn[0];
    }

    return std::make_tuple(keypoints, indices);
}

LiDARMatches LiDARMatchCalibrate::do_match(const PointCloudPtr &pcd_i, const PointCloudPtr &key_i,
                                           const std::vector<double> &gps_i, const PointFeaturePtr &fpfh_i,
                                           const PointCloudPtr &pcd_j, const PointCloudPtr &key_j,
                                           const std::vector<double> &gps_j, const PointFeaturePtr &fpfh_j) {
    open3d::geometry::KDTreeFlann tree_i(*pcd_i);
    open3d::geometry::KDTreeFlann tree_j(*pcd_j);
    open3d::geometry::KDTreeFlann tree_key_i(*key_i);
    open3d::geometry::KDTreeFlann tree_key_j(*key_j);

    LiDARMatches matches;

#pragma omp parallel for
    for (int ki = 0; ki < key_i->points_.size(); ++ki) {
        Vector3d keypoint_i = key_i->points_[ki];

        // 最近的 key_j 中的点
        Vector3d keypoint_j;
        {
            PointIndices indices;
            std::vector<double> square_distances;
            CHECK(tree_key_j.SearchKNN(keypoint_i, 1, indices, square_distances));
            keypoint_j = key_j->points_.at(indices[0]);
        }

        double distance_ij_old = (keypoint_i - keypoint_j).norm();
        if (distance_ij_old > param_mat_.icp_max_keypoint_distance) {
            continue;
        }

        // 找到对应区域的点云
        PointCloudPtr cluster_i, cluster_j;
        PointFeature cluster_fpfh_i, cluster_fpfh_j;
        {
            {
                PointIndices indices;
                std::vector<double> square_distances;
                tree_i.SearchHybrid(keypoint_i, param_mat_.icp_radius, param_mat_.icp_max_points, indices,
                                    square_distances);
                if (indices.empty()) {
                    continue;
                }
                cluster_i = pcd_i->SelectByIndex(std::vector<size_t>(begin(indices), end(indices)));
                if (param_mat_.icp_use_initial) {
                    cluster_fpfh_i.Resize(fpfh_i->Dimension(), indices.size());
                    for (int idx = 0; idx < indices.size(); ++idx) {
                        cluster_fpfh_i.data_.col(idx) = fpfh_i->data_.col(indices[idx]);
                    }
                }
            }
            {
                PointIndices indices;
                std::vector<double> square_distances;
                tree_j.SearchHybrid(keypoint_j, param_mat_.icp_radius, param_mat_.icp_max_points, indices,
                                    square_distances);
                if (indices.empty()) {
                    continue;
                }
                cluster_j = pcd_j->SelectByIndex(std::vector<size_t>(begin(indices), end(indices)));
                if (param_mat_.icp_use_initial) {
                    cluster_fpfh_j.Resize(fpfh_j->Dimension(), indices.size());
                    for (int idx = 0; idx < indices.size(); ++idx) {
                        cluster_fpfh_j.data_.col(idx) = fpfh_j->data_.col(indices[idx]);
                    }
                }
            }
        }

        open3d::pipelines::registration::RegistrationResult results;
        if (param_mat_.icp_use_initial) {
            open3d::pipelines::registration::FastGlobalRegistrationOption fgr_option;
            fgr_option.maximum_correspondence_distance_ = param_mat_.icp_max_distance * 3.0;
            results = open3d::pipelines::registration::FastGlobalRegistration(*cluster_i, *cluster_j, cluster_fpfh_i,
                                                                              cluster_fpfh_j, fgr_option);
        }

        results = open3d::pipelines::registration::RegistrationICP(*cluster_i, *cluster_j, param_mat_.icp_max_distance,
                                                                   results.transformation_);

        if (results.inlier_rmse_ > 2.0 * param_mat_.voxel_size || results.fitness_ < param_mat_.icp_t_overlap ||
            results.correspondence_set_.size() < param_mat_.icp_t_need_points) {
            continue;
        }

        Vector3d xform_keypoint_i = (results.transformation_ * keypoint_i.homogeneous()).eval().hnormalized();
        double distance_ij_new = (xform_keypoint_i - keypoint_j).norm();
        if (distance_ij_new > distance_ij_old) {
            continue;
        }

        // 找到最近点
        int pi, pj;
        {
            PointIndices indices;
            std::vector<double> sq_distances;
            CHECK(tree_j.SearchKNN(xform_keypoint_i, 1, indices, sq_distances) == 1);
            if (sqrt(sq_distances[0]) > param_mat_.voxel_size) {
                continue;
            }
            keypoint_j = pcd_j->points_.at(indices[0]);
            pj = indices[0];

            CHECK(tree_i.SearchKNN(keypoint_i, 1, indices, sq_distances) == 1);
            if (sqrt(sq_distances[0]) > 1e-3) {
                continue;
            }
            pi = indices[0];
        }

        LiDARCalibrate::TPS match;
        match.p_lidar1 = keypoint_i;
        match.p_lidar2 = keypoint_j;
        match.t1 = gps_i[pi];
        match.t2 = gps_j[pj];

        {
#pragma omp critical
            matches.push_back(match);
        }
    }
    return matches;
}

} // namespace h2o
