/*
 * Created Date: Tuesday, January 12th 2021, 10:50:01 am
 * Author: Han
 * 采用类似 ICP 的策略进行 LiDAR 的精配准
 * Copyright (c) 2021 Vrlab
 */
#include "lidar_calibrate_icp.h"
#include "lidar_calibrate_cost.h"
#include "pos.h"

#include <ceres/ceres.h>
#include <laszip/laszip_api.h>

namespace h2o {
class LiDARMatchCostFunctor {
public:
    LiDARMatchCostFunctor(LiDARCalibrateIcp *_calibrate, int _idx_i, int _idx_j)
        : calibrate(_calibrate), idx_i(_idx_i), idx_j(_idx_j) {}

    template <class T> bool operator()(const T *const ptr_dq, const T *const ptr_dt, T *residuals) const {

        using Vector3JetT = Eigen::Matrix<T, 3, 1>;
        using QuaternionJetT = Eigen::Quaternion<T>;

        // 世界坐标系
        const Vector3d &X1 = calibrate->points_i_[idx_i];
        const Vector3d &X2 = calibrate->points_j_[idx_j];
        double t1 = calibrate->gps_i_[idx_i];
        double t2 = calibrate->gps_j_[idx_j];

        const Pos &pos1 = calibrate->pos1_;
        const Pos &pos2 = calibrate->pos2_;

        const Quaterniond &q_body = calibrate->q_body_;
        const Vector3d &t_body = calibrate->t_body_;

        QuaternionJetT dq(ptr_dq);
        Vector3JetT dt(ptr_dt);

        auto [t_pos1, R_pos1] = pos1.getd(t1);
        auto [t_pos2, R_pos2] = pos2.getd(t2);
        t_pos1 += pos1.get_position_offset();
        t_pos2 += pos2.get_position_offset();

        Vector3d L1 = q_body.conjugate() * (R_pos1.transpose() * (X1 - t_pos1) - t_body);
        Vector3d L2 = q_body.conjugate() * (R_pos2.transpose() * (X2 - t_pos2) - t_body);

        Vector3JetT X1_hat =
            R_pos1.cast<T>() * (dq * q_body.cast<T>() * L1.cast<T>() + t_body.cast<T>() + dt) + t_pos1.cast<T>();
        Vector3JetT X2_hat =
            R_pos2.cast<T>() * (dq * q_body.cast<T>() * L2.cast<T>() + t_body.cast<T>() + dt) + t_pos2.cast<T>();

        residuals[0] = X1_hat.x() - X2_hat.x();
        residuals[1] = X1_hat.y() - X2_hat.y();
        residuals[2] = X1_hat.z() - X2_hat.z();
        return true;
    }

private:
    LiDARCalibrateIcp *calibrate;
    int idx_i;
    int idx_j;
};

LiDARCalibrateIcp::LiDARCalibrateIcp(const std::string &path_las1, const std::string &path_las2,
                                     const std::string &path_pos1, const std::string &path_pos2,
                                     const LiDARCalibrateIcpParam &param) {
    pos1_ = Pos(path_pos1);
    pos2_ = Pos(path_pos2);

    param_ = param;

    std::tie(points_i_, gps_i_) = load_and_downsample_las(path_las1);
    std::tie(points_j_, gps_j_) = load_and_downsample_las(path_las2);

    dq_ = Quaterniond::Identity();
    dt_ = Vector3d::Zero();

    q_body_ = Quaterniond::Identity();
    t_body_ = Vector3d::Zero();
}

std::tuple<Quaterniond, Vector3d> LiDARCalibrateIcp::calibrate() {
    dq_.setIdentity();
    dt_.setZero();

    for (int i = 0; i < param_.icp_max_iteration; ++i) {
        solve();
        evaluate();
    }

    return std::make_tuple(dq_, dt_);
}

void LiDARCalibrateIcp::solve() {
    blocks_.clear();

    problem_ = std::make_shared<ceres::Problem>();

    auto matches = knn_match();

    double huber_scale = (2.0 * param_.std_point) * (2.0 * param_.std_point);
    int num_valid = 0;
    for (const auto &match : matches) {
        if (match.first != -1 && match.second != -1) {
            num_valid++;
        }
    }

    for (const auto &match : matches) {
        if (match.first == -1 || match.second == -1)
            continue;

        int idx_i = match.first;
        int idx_j = match.second;
        ceres::CostFunction *cost = new ceres::AutoDiffCostFunction<LiDARMatchCostFunctor, 3, 4, 3>(
            new LiDARMatchCostFunctor(this, idx_i, idx_j));
        ceres::LossFunction *loss = new ceres::ScaledLoss(
            // new ceres::HuberLoss(huber_scale),
            nullptr, 1.0 / (param_.std_point * param_.std_point * num_valid * 3.0) * param_.lambda_point,
            ceres::TAKE_OWNERSHIP);
        auto block = problem_->AddResidualBlock(cost, loss, dq_.coeffs().data(), dt_.data());
        blocks_.push_back(block);
    }

    {
        // 加入角度信息的带权观测值
        ceres::CostFunction *cost = new ceres::AutoDiffCostFunction<BodyRotationRegularizationCostFunctor, 1, 4>(
            new BodyRotationRegularizationCostFunctor());
        problem_->AddResidualBlock(
            cost,
            new ceres::ScaledLoss(nullptr, 1.0 / (param_.std_body_angle * param_.std_body_angle) * param_.lambda_body,
                                  ceres::TAKE_OWNERSHIP),
            dq_.coeffs().data());
    }

    {
        // 加入偏移信息的带权观测值
        ceres::CostFunction *cost = new ceres::AutoDiffCostFunction<BodyTranslationRegularizationCostFunctor, 3, 3>(
            new BodyTranslationRegularizationCostFunctor());
        problem_->AddResidualBlock(
            cost,
            new ceres::ScaledLoss(nullptr,
                                  1.0 / (param_.std_body_trans * param_.std_body_trans * 3) * param_.lambda_body,
                                  ceres::TAKE_OWNERSHIP),
            dt_.data());
    }

    {
        ceres::LocalParameterization *quaternion_parametrization = new ceres::EigenQuaternionParameterization();
        problem_->SetParameterization(dq_.coeffs().data(), quaternion_parametrization);
    }

    ceres::Solver::Options op;
    op.linear_solver_type = ceres::DENSE_QR;
    op.num_threads = Eigen::nbThreads();

    ceres::Solver::Summary summary;
    ceres::Solve(op, problem_.get(), &summary);
    LOG(INFO) << summary.BriefReport();

    if (!summary.IsSolutionUsable()) {
        dq_ = Quaterniond::Identity();
        dt_ = Vector3d::Zero();
    }
}

void LiDARCalibrateIcp::evaluate() {
    ceres::Problem::EvaluateOptions evops;
    evops.apply_loss_function = false;
    evops.residual_blocks = blocks_;

    std::vector<double> residuals;
    problem_->Evaluate(evops, nullptr, &residuals, nullptr, nullptr);

    std::vector<double> errors(residuals.size() / 3);
    for (int i = 0; i < residuals.size() / 3; ++i) {
        errors[i] = residuals[3 * i + 0] * residuals[3 * i + 0] + residuals[3 * i + 1] * residuals[3 * i + 1] +
                    residuals[3 * i + 2] * residuals[3 * i + 2];
        errors[i] = sqrt(errors[i]);
    }

    LOG(INFO) << "RMSE for correspondence is: " << rmse(errors);
}

std::vector<std::pair<int, int>> LiDARCalibrateIcp::knn_match() {
    // 根据当前的 dq dt，更新点云坐标，构建 kdtree 查找最近邻
    auto update_points = [this](const std::vector<Vector3d> &points, const std::vector<double> &gps, const Pos &pos) {
        std::vector<Vector3d> points_hat(points.size());

#pragma omp parallel for
        for (int i = 0; i < points.size(); ++i) {
            Vector3d X = points[i];
            double t = gps[i];
            auto [t_pos, R_pos] = pos.getd(t);
            t_pos += pos.get_position_offset();

            Vector3d L = q_body_.conjugate() * (R_pos.transpose() * (X - t_pos) - t_body_);
            Vector3d X_hat = R_pos * (dq_ * q_body_ * L + t_body_ + dt_) + t_pos;
            points_hat[i] = X_hat;
        }
        return points_hat;
    };

    auto points_i_hat = update_points(points_i_, gps_i_, pos1_);
    auto points_j_hat = update_points(points_j_, gps_j_, pos2_);

    auto kdtree = std::make_shared<KDTreeVectorAdaptor>(3, points_j_hat);

    constexpr int KNN = 1;
    std::vector<std::pair<int, int>> matches_ij(points_i_.size(), std::make_pair(-1, -1));

#pragma omp parallel for
    for (int i = 0; i < points_i_.size(); ++i) {
        std::vector<Eigen::Index> indices(KNN);
        std::vector<double> sq_distances(KNN);
        kdtree->query(points_i_hat[i].data(), KNN, indices.data(), sq_distances.data());
        if (sq_distances[0] > param_.icp_max_distance * param_.icp_max_distance) {
            continue;
        }
        matches_ij[i] = std::make_pair(i, (int)indices[0]);
    }
    return matches_ij;
}

std::tuple<std::vector<Vector3d>, std::vector<double>>
LiDARCalibrateIcp::load_and_downsample_las(const std::string &path) {
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

    std::vector<Vector3d> points(num_points);
    std::vector<double> gpstime(num_points);

    BoundingBox3d bounds;
    for (int i = 0; i < num_points; ++i) {
        if (laszip_read_point(lasreader)) {
            CHECK(false) << "Read point error!";
        }
        double x = (double)point->X * lasheader->x_scale_factor + lasheader->x_offset;
        double y = (double)point->Y * lasheader->y_scale_factor + lasheader->y_offset;
        double z = (double)point->Z * lasheader->z_scale_factor + lasheader->z_offset;

        points[i] = Vector3d(x, y, z);
        gpstime[i] = point->gps_time;
        bounds.extend(points[i]);
    }

    laszip_close_reader(lasreader);
    laszip_destroy(lasreader);

    // 降采样
    {

        double voxel_size = param_.std_point;

        double inverse_voxel = 1.0 / param_.std_point;

        // 整个 volumn 的行列号大小
        Vector3i volumn = (bounds.sizes() * inverse_voxel).array().cast<int>();
        volumn += Vector3i::Ones();
        Vector3i div_mul = Eigen::Vector3i(1, volumn[0], volumn[0] * volumn[1]);

        std::unordered_set<uint64_t> indices_voxels;
        std::vector<size_t> indices;

        for (int i = 0; i < points.size(); ++i) {
            Vector3d point = points[i];
            point -= bounds.min();
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

        std::vector<Vector3d> voxel_points(indices.size());
        std::vector<double> voxel_gpstime(indices.size());
        for (int i = 0; i < indices.size(); ++i) {
            voxel_gpstime[i] = gpstime[indices[i]];
            voxel_points[i] = points[indices[i]];
        }

        points = voxel_points;
        gpstime = voxel_gpstime;
    }

    return std::make_tuple(points, gpstime);
}

} // namespace h2o
