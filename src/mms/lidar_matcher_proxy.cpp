/*
 * Created Date: Friday, January 8th 2021, 2:51:53 pm
 * Author: Han
 * 实现 MMS 匹配的类
 * Copyright (c) 2021
 */

#include "lidar_matcher_proxy.h"

#include <base/base.h>
#include <laszip/laszip_api.h>
#include <open3d/pipelines/registration/FastGlobalRegistration.h>

namespace h2o {
LiDARMatcherProxy::LiDARMatcherProxy(const std::string &path_las_i, const std::string &path_las_j,
                                     const std::string &path_pos_i, const std::string &path_pos_j) {
    Vector3d offset;
    {
        LasHeader header = pc_read_header(path_las_i);
        offset = header.offset;
    }
    LOG(INFO) << "Loading...";
    pcd_data_i_ = read_las(path_las_i, offset);
    pcd_data_j_ = read_las(path_las_j, offset);

    pos_i_ = Pos(path_pos_i);
    pos_j_ = Pos(path_pos_j);
}
LiDARMatches LiDARMatcherProxy::match() {
    LOG(INFO) << "Downsampling...";
    pcd_data_i_ = downsample_voxel(pcd_data_i_);
    pcd_data_j_ = downsample_voxel(pcd_data_j_);

    if (param_.icp_use_initial) {
        LOG(INFO) << "Computing normal...";
        compute_normal(pcd_data_i_, pos_i_);
        compute_normal(pcd_data_j_, pos_j_);

        LOG(INFO) << "Computing features...";
        pcd_data_i_.feats = extract_feature(pcd_data_i_.pcd);
        pcd_data_j_.feats = extract_feature(pcd_data_j_.pcd);
    }

    LOG(INFO) << "Detecting keypoints...";
    std::tie(pcd_data_i_.keys, pcd_data_i_.ikeys) = detect_keypoints(pcd_data_i_.pcd);
    std::tie(pcd_data_j_.keys, pcd_data_j_.ikeys) = detect_keypoints(pcd_data_j_.pcd);

    LOG(INFO) << "Matching...";
    LiDARMatches matches = do_match(pcd_data_i_.pcd, pcd_data_i_.keys, pcd_data_i_.gpstime, pcd_data_i_.feats,
                                    pcd_data_j_.pcd, pcd_data_j_.keys, pcd_data_j_.gpstime, pcd_data_j_.feats);

    LOG(INFO) << "Final matches is " << matches.size();
    // 绘制同名点
    if (param_.show_match) {
        std::vector<Vector3d> key_match_i(matches.size()), key_match_j(matches.size());
        for (int i = 0; i < matches.size(); ++i) {
            key_match_i[i] = matches[i].p_lidar1;
            key_match_j[i] = matches[i].p_lidar2;
        }

        auto keypoint_to_sphere = [](const Vector3d &point, const Vector3d &color) {
            auto sphere = open3d::geometry::TriangleMesh::CreateSphere(0.05, 8);
            sphere->Translate(point);
            sphere->PaintUniformColor(color);
            return sphere;
        };

        auto keypoints_to_sphere = [](const std::vector<Vector3d> &points, const Vector3d &color) {
            auto spheres = std::make_shared<open3d::geometry::TriangleMesh>();
            for (const auto &point : points) {
                auto sphere = open3d::geometry::TriangleMesh::CreateSphere(0.05, 8);
                sphere->Translate(point);
                sphere->PaintUniformColor(color);

                *spheres += *sphere;
            }

            return spheres;
        };

        auto matches_to_lines = [](const std::vector<Vector3d> &points_i, const std::vector<Vector3d> &points_j,
                                   const Vector3d &color) {
            std::vector<Vector3d> all_points;
            all_points.insert(end(all_points), begin(points_i), end(points_i));
            all_points.insert(end(all_points), begin(points_j), end(points_j));
            std::vector<Vector2i> indices(points_i.size());
            for (int i = 0; i < points_i.size(); ++i) {
                indices[i].x() = i;
                indices[i].y() = i + points_i.size();
            }

            auto lineset = std::make_shared<open3d::geometry::LineSet>(all_points, indices);
            return lineset;
        };

        pcd_data_i_.pcd->PaintUniformColor(Vector3d(1, 0.706, 0));
        pcd_data_j_.pcd->PaintUniformColor(Vector3d(0, 0.651, 0.929));
        open3d::visualization::DrawGeometries({pcd_data_i_.pcd, pcd_data_j_.pcd,
                                               keypoints_to_sphere(key_match_i, Vector3d(1, 0, 0)),
                                               keypoints_to_sphere(key_match_j, Vector3d(0, 1, 1)),
                                               matches_to_lines(key_match_i, key_match_j, Vector3d(0, 0, 1))},
                                              "Match", 1920, 1080);
    }

    // 获取 pos 信息
    for (auto &m : matches) {
        m.p_lidar1 += pcd_data_i_.offset;
        m.p_lidar2 += pcd_data_j_.offset;

        auto [t_pos1, q_pos1] = pos_i_.getd(m.t1);
        auto [t_pos2, q_pos2] = pos_j_.getd(m.t2);

        m.q_pos1 = q_pos1;
        m.q_pos2 = q_pos2;
        m.t_pos1 = t_pos1 + pos_i_.get_position_offset();
        m.t_pos2 = t_pos2 + pos_j_.get_position_offset();
    }
    return matches;
}

PointCloudWithData LiDARMatcherProxy::read_las(const std::string &path, const Vector3d &offset) {
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
    pcd->colors_.resize(num_points);
    std::vector<double> gpstime(num_points);

    for (int i = 0; i < num_points; ++i) {
        if (laszip_read_point(lasreader)) {
            CHECK(false) << "Read point error!";
        }
        double x = (double)point->X * lasheader->x_scale_factor + lasheader->x_offset;
        double y = (double)point->Y * lasheader->y_scale_factor + lasheader->y_offset;
        double z = (double)point->Z * lasheader->z_scale_factor + lasheader->z_offset;

        x -= offset.x();
        y -= offset.y();
        z -= offset.z();

        double intensity = point->intensity / 65535.0;

        pcd->points_[i] = Vector3d(x, y, z);
        pcd->colors_[i] = Vector3d(intensity, intensity, intensity);
        gpstime[i] = point->gps_time;
    }

    laszip_close_reader(lasreader);
    laszip_destroy(lasreader);

    PointCloudWithData pcdwt;
    pcdwt.pcd = pcd;
    pcdwt.gpstime = gpstime;
    pcdwt.offset = offset;
    return pcdwt;
}

PointCloudWithData LiDARMatcherProxy::downsample_voxel(const PointCloudWithData &pcd_data) {
    auto bounds = pcd_data.pcd->GetAxisAlignedBoundingBox();

    double inverse_voxel = 1.0 / param_.voxel_size;

    // 整个 volumn 的行列号大小
    Vector3i volumn = (bounds.GetExtent() * inverse_voxel).array().cast<int>();
    volumn += Vector3i::Ones();
    Vector3i div_mul = Eigen::Vector3i(1, volumn[0], volumn[0] * volumn[1]);

    std::unordered_set<uint64_t> indices_voxels;
    std::vector<size_t> indices;

    for (int i = 0; i < pcd_data.pcd->points_.size(); ++i) {
        Vector3d point = pcd_data.pcd->points_.at(i);
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
        voxel_gpstime[i] = pcd_data.gpstime[indices[i]];
    }
    auto voxel_pcd = pcd_data.pcd->SelectByIndex(indices);

    return {voxel_pcd, voxel_gpstime, pcd_data.offset};
}

void LiDARMatcherProxy::compute_normal(const PointCloudWithData &pcd_data, const Pos &pos) {
    // 计算法线
    if (!pcd_data.pcd->HasNormals()) {
        auto search_param = open3d::geometry::KDTreeSearchParamHybrid(param_.ne_radius, param_.ne_max_knn);
        pcd_data.pcd->EstimateNormals(search_param);
    }

    // 法线朝向轨迹位置
    Vector3d pos_offset = pos.get_position_offset();
    for (int i = 0; i < pcd_data.gpstime.size(); ++i) {
        double t = pcd_data.gpstime[i];
        Vector3d t_pos = std::get<0>(pos.getd(t)) + pos_offset - pcd_data.offset;
        Vector3d point = pcd_data.pcd->points_[i];
        point = t_pos - point; // 点云到pos位置得朝向
        if (pcd_data.pcd->normals_[i].dot(point) < 0) {
            pcd_data.pcd->normals_[i] = -pcd_data.pcd->normals_[i];
        }
    }
}

PointFeaturePtr LiDARMatcherProxy::extract_feature(const PointCloudPtr &pcd) {
    auto search_param = open3d::geometry::KDTreeSearchParamHybrid(param_.ne_radius * 2.0, param_.ne_max_knn * 4);
    auto feature = open3d::pipelines::registration::ComputeFPFHFeature(*pcd, search_param);
    return feature;
}

std::tuple<PointCloudPtr, PointIndices> LiDARMatcherProxy::detect_keypoints(const PointCloudPtr &pcd) {
    PointCloudPtr keypoints = open3d::geometry::keypoint::ComputeISSKeypoints(
        *pcd, param_.key_radius, param_.key_radius, param_.key_gamma, param_.key_gamma);

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

LiDARMatches LiDARMatcherProxy::do_match(const PointCloudPtr &pcd_i, const PointCloudPtr &key_i,
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
        if (distance_ij_old > param_.icp_max_keypoint_distance) {
            continue;
        }

        // 找到对应区域的点云
        PointCloudPtr cluster_i, cluster_j;
        PointFeature cluster_fpfh_i, cluster_fpfh_j;
        {
            {
                PointIndices indices;
                std::vector<double> square_distances;
                tree_i.SearchHybrid(keypoint_i, param_.icp_radius, param_.icp_max_points, indices, square_distances);
                if (indices.empty()) {
                    continue;
                }
                cluster_i = pcd_i->SelectByIndex(std::vector<size_t>(begin(indices), end(indices)));
                if (param_.icp_use_initial) {
                    cluster_fpfh_i.Resize(fpfh_i->Dimension(), indices.size());
                    for (int idx = 0; idx < indices.size(); ++idx) {
                        cluster_fpfh_i.data_.col(idx) = fpfh_i->data_.col(indices[idx]);
                    }
                }
            }
            {
                PointIndices indices;
                std::vector<double> square_distances;
                tree_j.SearchHybrid(keypoint_j, param_.icp_radius, param_.icp_max_points, indices, square_distances);
                if (indices.empty()) {
                    continue;
                }
                cluster_j = pcd_j->SelectByIndex(std::vector<size_t>(begin(indices), end(indices)));
                if (param_.icp_use_initial) {
                    cluster_fpfh_j.Resize(fpfh_j->Dimension(), indices.size());
                    for (int idx = 0; idx < indices.size(); ++idx) {
                        cluster_fpfh_j.data_.col(idx) = fpfh_j->data_.col(indices[idx]);
                    }
                }
            }
        }

        open3d::pipelines::registration::RegistrationResult results;
        if (param_.icp_use_initial) {
            open3d::pipelines::registration::FastGlobalRegistrationOption fgr_option;
            fgr_option.maximum_correspondence_distance_ = param_.icp_max_distance * 3.0;
            results = open3d::pipelines::registration::FastGlobalRegistration(*cluster_i, *cluster_j, cluster_fpfh_i,
                                                                              cluster_fpfh_j, fgr_option);
        }

        results = open3d::pipelines::registration::RegistrationICP(*cluster_i, *cluster_j, param_.icp_max_distance,
                                                                   results.transformation_);

        if (results.inlier_rmse_ > 2.0 * param_.voxel_size || results.fitness_ < param_.icp_t_overlap ||
            results.correspondence_set_.size() < param_.icp_t_need_points) {
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
            if (sqrt(sq_distances[0]) > param_.voxel_size) {
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
