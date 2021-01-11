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

class BodyRotationRegularizationCostFunctor {
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

class BodyTranslationRegularizationCostFunctor {
public:
    template <typename T> bool operator()(const T *const ptr_dt_body, T *residual) const {
        using Vector3JetT = Eigen::Matrix<T, 3, 1>;

        residual[0] = ptr_dt_body[0];
        residual[1] = ptr_dt_body[1];
        residual[2] = ptr_dt_body[2];

        return true;
    }
};

LiDARCalibrate::LiDARCalibrate(const Config &config) : param_(config) {
    q_body_ = dq_body_ = Quaterniond::Identity();
    t_body_ = dt_body_ = Vector3d::Zero();
}

void LiDARCalibrate::set_body(const Quaterniond &q, const Vector3d &t) {
    q_body_ = q;
    t_body_ = t;
}

void LiDARCalibrate::set_gcp(const std::vector<GCP> &gcps) {
    gcps_ = gcps;
    // 初始化 GCP 的内部信息
    for (GCP &gcp : gcps_) {
        // TODO GCP 的导入也必须要有pos 信息
        gcp.p_lidar_local = q_body_.conjugate() * (gcp.q_pos.conjugate() * (gcp.p_lidar - gcp.t_pos) - t_body_);
    }
}

void LiDARCalibrate::set_tps(const std::vector<TPS> &tps) {
    tps_ = tps;
    for (TPS &tps : tps_) {
        tps.p_local1 = q_body_.conjugate() * (tps.q_pos1.conjugate() * (tps.p_lidar1 - tps.t_pos1) - t_body_);
        tps.p_local2 = q_body_.conjugate() * (tps.q_pos2.conjugate() * (tps.p_lidar2 - tps.t_pos2) - t_body_);
    }
}

std::tuple<Quaterniond, Vector3d> LiDARCalibrate::calibrate() {

    dq_body_ = Quaterniond::Identity();
    dt_body_ = Vector3d::Zero();

    for (int epoch = 0; epoch < param_.epoch_clean; ++epoch) {
        solve();
        clean();
    }

    solve();
    evaluate();

    return std::make_tuple(dq_body_, dt_body_);
}

void LiDARCalibrate::solve() {
    blocks_tps_.clear();
    blocks_gcps_.clear();

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
        ceres::CostFunction *cost = new ceres::AutoDiffCostFunction<BodyRotationRegularizationCostFunctor, 1, 4>(
            new BodyRotationRegularizationCostFunctor());
        problem_->AddResidualBlock(cost,
                                   new ceres::ScaledLoss(nullptr, 1.0 / (param_.std_body_angle * param_.std_body_angle),
                                                         ceres::TAKE_OWNERSHIP),
                                   dq_body_.coeffs().data());
    }

    {
        // 加入偏移信息的带权观测值
        ceres::CostFunction *cost = new ceres::AutoDiffCostFunction<BodyTranslationRegularizationCostFunctor, 3, 3>(
            new BodyTranslationRegularizationCostFunctor());
        problem_->AddResidualBlock(
            cost,
            new ceres::ScaledLoss(nullptr, 1.0 / (param_.std_body_translate * param_.std_body_translate * 3),
                                  ceres::TAKE_OWNERSHIP),
            dt_body_.data());
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

    if (!summary.IsSolutionUsable()) {
        dq_body_ = Quaterniond::Identity();
        dt_body_ = Vector3d::Zero();
    }
}

void LiDARCalibrate::clean() {
    using namespace ceres;
    Problem::EvaluateOptions opt;
    opt.residual_blocks = blocks_tps_;
    opt.apply_loss_function = false;

    // 单位权中误差，因为 ceres 的 cost 是 1/2 f(x)^2 所以这里的 cost 需要乘以2
    double cost;
    std::vector<double> residuals;
    problem_->Evaluate(opt, &cost, &residuals, NULL, NULL);
    cost *= 2.0;
    // s = sqrt(vPv / r)
    double sigma0 = std::sqrt(cost / (residuals.size() - 6));
    double sigma_pixel = sigma0;
    sigma0 *= 3; // 3 sigma

    std::set<int> reserved;
    int i = 0;
    // 最大的误差，初始迭代 40 像素，后续迭代 3像素
    // double maximum = current_iter_ == 0 ? 100.0 * w_pixel_ : 1.0 * w_pixel_;

    for (int i = 0; i < tps_.size(); ++i) {
        double rx = residuals[3 * i + 0];
        double ry = residuals[3 * i + 1];
        double rz = residuals[3 * i + 2];
        double r = std::sqrt(rx * rx + ry * ry + rz * rz);
        if (r < param_.std_lidar_point) {
            continue;
        }
        if (r < sigma0) {
            reserved.insert(i);
        }
    }

    LOG(INFO) << string_printf("reserved %d/%d points with sigma0 %.2lf", reserved.size(), tps_.size(), sigma_pixel);

    // 移除对应 id 的 track
    auto tps_clone = tps_;
    tps_.clear();
    for (int id : reserved) {
        tps_.push_back(tps_clone[id]);
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
    LOG(INFO) << "Translation size of dt is: " << dt_body_.norm();
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
        // t1 x1 y1 z1 t2 x2 y2 z2 tx1 ty1 tz1 qx1 qy1 qz1 qw1 tx2 ty2 tz2 qx2 qy2 qz2 qw2
        tps[i].t1 = doc.GetCell<double>(0, i);
        tps[i].p_lidar1.x() = doc.GetCell<double>(1, i);
        tps[i].p_lidar1.y() = doc.GetCell<double>(2, i);
        tps[i].p_lidar1.z() = doc.GetCell<double>(3, i);

        tps[i].t2 = doc.GetCell<double>(4, i);
        tps[i].p_lidar2.x() = doc.GetCell<double>(5, i);
        tps[i].p_lidar2.y() = doc.GetCell<double>(6, i);
        tps[i].p_lidar2.z() = doc.GetCell<double>(7, i);

        tps[i].t_pos1.x() = doc.GetCell<double>(8, i);
        tps[i].t_pos1.y() = doc.GetCell<double>(9, i);
        tps[i].t_pos1.z() = doc.GetCell<double>(10, i);

        tps[i].q_pos1.x() = doc.GetCell<double>(11, i);
        tps[i].q_pos1.y() = doc.GetCell<double>(12, i);
        tps[i].q_pos1.z() = doc.GetCell<double>(13, i);
        tps[i].q_pos1.w() = doc.GetCell<double>(14, i);

        tps[i].t_pos2.x() = doc.GetCell<double>(15, i);
        tps[i].t_pos2.y() = doc.GetCell<double>(16, i);
        tps[i].t_pos2.z() = doc.GetCell<double>(17, i);

        tps[i].q_pos2.x() = doc.GetCell<double>(18, i);
        tps[i].q_pos2.y() = doc.GetCell<double>(19, i);
        tps[i].q_pos2.z() = doc.GetCell<double>(20, i);
        tps[i].q_pos2.w() = doc.GetCell<double>(21, i);
    }

    return tps;
}
} // namespace h2o