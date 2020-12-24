/*
 * Created Date: Wednesday, December 23rd 2020, 3:55:15 pm
 * Author: Han
 * 检校 LiDAR 和 IMU 之间的误差
 * Copyright (c) 2020 Vrlab
 */

#include "lidar_calibrate.h"
#include "rapidcsv.h"

#include <base/base.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace h2o {
class GcpCostFunctor {
public:
    GcpCostFunctor(const LiDARCalibrate::GCP &gcp, const Quaterniond &q_body, const Vector3d &t_body)
        : gcp_(gcp), q_body_(q_body), t_body_(t_body) {}

    template <typename T> bool operator()(const T *const ptr_dq_body, const T *const ptr_dt_body, T *residual) const {
        using Vector3JetT = Eigen::Matrix<T, 3, 1>;
        using QuaternionJetT = Eigen::Quaternion<T>;

        QuaternionJetT dq_body(ptr_dq_body);
        Vector3JetT dt_body(ptr_dt_body);

        Vector3JetT p_lidar = gcp_.q_pos.cast<T>() * (dq_body * q_body_.cast<T>() * gcp_.p_lidar_local.cast<T>() +
                                                      t_body_.cast<T>() + dt_body) +
                              gcp_.t_pos.cast<T>();

        residual[0] = p_lidar(0) - gcp_.p_gcp(0);
        residual[1] = p_lidar(1) - gcp_.p_gcp(1);
        residual[2] = p_lidar(2) - gcp_.p_gcp(2);

        return true;
    }

private:
    const Quaterniond &q_body_;
    const Vector3d &t_body_;
    const LiDARCalibrate::GCP &gcp_;
};

class TpsCostFunctor {
public:
    TpsCostFunctor(const LiDARCalibrate::TPS &tps, const Quaterniond &q_body, const Vector3d &t_body)
        : tps_(tps), q_body_(q_body), t_body_(t_body) {}

    template <typename T> bool operator()(const T *const ptr_dq_body, const T *const ptr_dt_body, T *residual) const {
        using Vector3JetT = Eigen::Matrix<T, 3, 1>;
        using QuaternionJetT = Eigen::Quaternion<T>;

        QuaternionJetT dq_body(ptr_dq_body);
        Vector3JetT dt_body(ptr_dt_body);

        Vector3JetT p_lidar1 = tps_.q_pos1.cast<T>() * (dq_body * q_body_.cast<T>() * tps_.p_local1.cast<T>() +
                                                        t_body_.cast<T>() + dt_body) +
                               tps_.t_pos1.cast<T>();

        Vector3JetT p_lidar2 = tps_.q_pos2.cast<T>() * (dq_body * q_body_.cast<T>() * tps_.p_local2.cast<T>() +
                                                        t_body_.cast<T>() + dt_body) +
                               tps_.t_pos2.cast<T>();

        residual[0] = p_lidar1(0) - p_lidar2(0);
        residual[1] = p_lidar1(1) - p_lidar2(1);
        residual[2] = p_lidar1(2) - p_lidar2(2);

        return true;
    }

private:
    const Quaterniond &q_body_;
    const Vector3d &t_body_;
    const LiDARCalibrate::TPS &tps_;
};

class BodyRegularizationCostFunctor {
public:
    template <typename T> bool operator()(const T *const ptr_dq_body, T *residual) const {
        using Vector3JetT = Eigen::Matrix<T, 3, 1>;
        using QuaternionJetT = Eigen::Quaternion<T>;
        QuaternionJetT dq_body(ptr_dq_body);
        using AngleAxisJetT = Eigen::AngleAxis<T>;

        T theta = AngleAxisJetT(dq_body).angle();
        residual[0] = theta;

        return true;
    }

private:
};

LiDARCalibrate::LiDARCalibrate(const Pos &pos, const Config &config) : pos_(pos), param_(config), pos2_(Pos()) {
    q_body_ = dq_body_ = Quaterniond::Identity();
    t_body_ = dt_body_ = Vector3d::Zero();

    has_pos2_ = false;
}

LiDARCalibrate::LiDARCalibrate(const Pos &pos1, const Pos &pos2, const Config &config)
    : pos_(pos1), pos2_(pos2), param_(config) {
    q_body_ = dq_body_ = Quaterniond::Identity();
    t_body_ = dt_body_ = Vector3d::Zero();

    has_pos2_ = true;
}

void LiDARCalibrate::set_body(const Quaterniond &q, const Vector3d &t) {
    q_body_ = q;
    t_body_ = t;
}
void LiDARCalibrate::set_gcp(const std::vector<GCP> &gcps) {
    gcps_ = gcps;
    // 初始化 GCP 的内部信息
    for (GCP &gcp : gcps_) {
        auto [t_pos, R_pos] = pos_.getd(gcp.gpstime);
        gcp.q_pos = R_pos;
        gcp.t_pos = t_pos + pos_.get_position_offset();
        gcp.p_lidar_local = q_body_.inverse() * (R_pos.transpose() * (gcp.p_lidar - gcp.t_pos) - t_body_);
    }
}

void LiDARCalibrate::set_tps(const std::vector<TPS> &tps) {
    tps_ = tps;

    for (TPS &tps : tps_) {
        auto [t_pos1, R_pos1] = pos_.getd(tps.t1);
        tps.q_pos1 = R_pos1;
        tps.t_pos1 = t_pos1 + pos_.get_position_offset();
        tps.p_local1 = q_body_.inverse() * (R_pos1.transpose() * (tps.p_lidar1 - tps.t_pos1) - t_body_);

        if (has_pos2_) {
            auto [t_pos2, R_pos2] = pos2_.getd(tps.t2);
            tps.q_pos2 = R_pos2;
            tps.t_pos2 = t_pos2 + pos_.get_position_offset();
            tps.p_local2 = q_body_.inverse() * (R_pos2.transpose() * (tps.p_lidar2 - tps.t_pos2) - t_body_);
        } else {
            auto [t_pos2, R_pos2] = pos_.getd(tps.t2);
            tps.q_pos2 = R_pos2;
            tps.t_pos2 = t_pos2 + pos_.get_position_offset();
            tps.p_local2 = q_body_.inverse() * (R_pos2.transpose() * (tps.p_lidar2 - tps.t_pos2) - t_body_);
        }
    }
}

std::tuple<Quaterniond, Vector3d> LiDARCalibrate::calibrate() {

    blocks_tps_.clear();
    blocks_gcps_.clear();
    dq_body_ = Quaterniond::Identity();
    dt_body_ = Vector3d::Zero();

    problem_ = std::make_shared<ceres::Problem>();

    double num_gcps = gcps_.size() * 3.0;
    double num_tps = tps_.size() * 3.0;

    for (int i = 0; i < gcps_.size(); ++i) {
        ceres::CostFunction *cost =
            new ceres::AutoDiffCostFunction<GcpCostFunctor, 3, 4, 3>(new GcpCostFunctor(gcps_[i], q_body_, t_body_));

        auto block = problem_->AddResidualBlock(
            cost,
            new ceres::ScaledLoss(nullptr, 1.0 / (param_.std_lidar_point * param_.std_lidar_point * num_gcps),
                                  ceres::TAKE_OWNERSHIP),
            dq_body_.coeffs().data(), dt_body_.data());
        blocks_gcps_.push_back(block);
    }
    for (int i = 0; i < tps_.size(); ++i) {
        ceres::CostFunction *cost =
            new ceres::AutoDiffCostFunction<TpsCostFunctor, 3, 4, 3>(new TpsCostFunctor(tps_[i], q_body_, t_body_));
        auto block = problem_->AddResidualBlock(
            cost,
            new ceres::ScaledLoss(nullptr, 1.0 / (param_.std_lidar_point * param_.std_lidar_point * num_tps),
                                  ceres::TAKE_OWNERSHIP),
            dq_body_.coeffs().data(), dt_body_.data());
        blocks_tps_.push_back(block);
    }

    {
        // 加入角度信息的带权观测值
        ceres::CostFunction *cost =
            new ceres::AutoDiffCostFunction<BodyRegularizationCostFunctor, 1, 4>(new BodyRegularizationCostFunctor());
        problem_->AddResidualBlock(cost,
                                   new ceres::ScaledLoss(nullptr, 1.0 / (param_.std_body_angle * param_.std_body_angle),
                                                         ceres::TAKE_OWNERSHIP),
                                   dq_body_.coeffs().data());
    }

    ceres::LocalParameterization *quaternion_parametrization = new ceres::EigenQuaternionParameterization();
    problem_->SetParameterization(dq_body_.coeffs().data(), quaternion_parametrization);

    if (!param_.with_translate) {
        problem_->SetParameterBlockConstant(dt_body_.data());
    }

    ceres::Solver::Options op;
    op.linear_solver_type = ceres::DENSE_QR;
    op.num_threads = 1;

    ceres::Solver::Summary summary;
    ceres::Solve(op, problem_.get(), &summary);
    LOG(INFO) << summary.BriefReport();

    evaluate();

    if (summary.IsSolutionUsable()) {
        return std::make_tuple(dq_body_, dt_body_);
    } else {
        return std::make_tuple(Quaterniond::Identity(), Vector3d::Zero());
    }
}

void LiDARCalibrate::evaluate() {
    if (blocks_gcps_.size()) {
        ceres::Problem::EvaluateOptions evops;
        evops.apply_loss_function = false;
        evops.residual_blocks = blocks_gcps_;

        std::vector<double> residuals;
        problem_->Evaluate(evops, nullptr, &residuals, nullptr, nullptr);

        std::vector<double> errors(gcps_.size());
        for (int i = 0; i < gcps_.size(); ++i) {
            errors[i] = residuals[3 * i + 0] * residuals[3 * i + 0] + residuals[3 * i + 1] * residuals[3 * i + 1] +
                        residuals[3 * i + 2] * residuals[3 * i + 2];
            errors[i] = sqrt(errors[i]);
        }

        LOG(INFO) << "RMSE for GCP is: " << rmse(errors);
    }

    if (blocks_tps_.size()) {
        ceres::Problem::EvaluateOptions evops;
        evops.apply_loss_function = false;
        evops.residual_blocks = blocks_tps_;

        std::vector<double> residuals;
        problem_->Evaluate(evops, nullptr, &residuals, nullptr, nullptr);

        std::vector<double> errors(tps_.size());
        for (int i = 0; i < tps_.size(); ++i) {
            errors[i] = residuals[3 * i + 0] * residuals[3 * i + 0] + residuals[3 * i + 1] * residuals[3 * i + 1] +
                        residuals[3 * i + 2] * residuals[3 * i + 2];
            errors[i] = sqrt(errors[i]);
        }

        LOG(INFO) << "RMSE for TPS is: " << rmse(errors);
    }

    LOG(INFO) << "Rotation size of dq is: " << AngleAxisd(dq_body_).angle() * RAD2DEG * 2.0;
}

std::vector<LiDARCalibrate::GCP> LiDARCalibrate::GCP::load(const std::string &path) {
    rapidcsv::Document doc(string_utf8_to_fstream(path), rapidcsv::LabelParams(-1));

    int nrows = doc.GetRowCount();
    std::vector<LiDARCalibrate::GCP> gcps(nrows);
    for (int i = 0; i < nrows; ++i) {
        gcps[i].gpstime = doc.GetCell<double>(0, i);
        gcps[i].p_lidar.x() = doc.GetCell<double>(1, i);
        gcps[i].p_lidar.y() = doc.GetCell<double>(2, i);
        gcps[i].p_lidar.z() = doc.GetCell<double>(3, i);
        gcps[i].p_gcp.x() = doc.GetCell<double>(4, i);
        gcps[i].p_gcp.y() = doc.GetCell<double>(5, i);
        gcps[i].p_gcp.z() = doc.GetCell<double>(6, i);
    }

    return gcps;
}
std::vector<LiDARCalibrate::TPS> LiDARCalibrate::TPS::load(const std::string &path) {
    rapidcsv::Document doc(string_utf8_to_fstream(path), rapidcsv::LabelParams(-1));

    int nrows = doc.GetRowCount();
    std::vector<LiDARCalibrate::TPS> tps(nrows);
    for (int i = 0; i < nrows; ++i) {
        tps[i].t1 = doc.GetCell<double>(0, i);
        tps[i].p_lidar1.x() = doc.GetCell<double>(1, i);
        tps[i].p_lidar1.y() = doc.GetCell<double>(2, i);
        tps[i].p_lidar1.z() = doc.GetCell<double>(3, i);

        tps[i].t2 = doc.GetCell<double>(4, i);
        tps[i].p_lidar2.x() = doc.GetCell<double>(5, i);
        tps[i].p_lidar2.y() = doc.GetCell<double>(6, i);
        tps[i].p_lidar2.z() = doc.GetCell<double>(7, i);
    }

    return tps;
}
} // namespace h2o