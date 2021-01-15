/*
 * Created Date: Wednesday, December 23rd 2020, 3:55:15 pm
 * Author: Han
 * 检校 LiDAR 和 IMU 之间的误差
 * Copyright (c) 2020 Vrlab
 */

#include "lidar_calibrate.h"
#include "lidar_calibrate_cost.h"
#include "rapidcsv.h"

#include <base/base.h>
#include <base/json.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace h2o {

template <typename T>
inline Eigen::Matrix<T, 3, 3> quaternionCovToYawPitchRollCov(const Eigen::Matrix<T, 4, 4> &covariance,
                                                             const Eigen::Quaternion<T> &q) {
    Eigen::Matrix<T, 3, 4> J = Eigen::Matrix<T, 3, 4>::Identity();

    // Generally there exist 3 cases for calculation of yaw, pitch and roll
    // from a quaternion:
    // 1. regular case
    // 2. gimbal lock upwards
    // 3. gimbal lock downwards
    // For these three cases the Jacobian is calculated

    // There exist different methods to calculate the yaw, pitch, roll angle
    // from a quaternion:
    //
    // 1. The current implementation of the Jacobian is build on the calculations
    // of yaw, pitch and roll from Eigen
    //
    // 2. A different calculation is described in the technical report
    // @TECHREPORT{blanco2010se3,
    //    author = {Blanco, Jos{\'{e}}-Luis},
    //    month = sep,
    //    title = {A tutorial on SE(3) transformation parameterizations and on-manifold optimization},
    //    year = {2010},
    //    institution = {University of Malaga}
    //}
    // http://mapir.isa.uma.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
    //
    // This calculation is simpler and faster. But since we were not sure, whether
    // it is compatible to the eigen calculation of the yaw, pitch and roll
    // it is uncommented until yet

    // the jacobian of the eigen conversion from quaternion to yaw, pitch, roll
    T a = (T)1.0 - (T)2.0 * (q.x() * q.x() + q.y() * q.y());
    T b = (T)2.0 * (q.y() * q.z() + q.w() * q.x());
    T c = sqrt(a * a + b * b);

    if (c > 1e-3) {
        // from eigen quaternionToYawPitchRoll
        // pitch = atan2(-2.0*(q.x()*q.z()-q.w()*q.y()), c)/M_PI*180.0
        // roll  = atan2(2.0*(q.z()*q.y()+q.w()*q.x()), (1.0 - 2.0*(q.x()*q.x()+q.y() * q.y())))/M_PI*180.0
        // yaw   = atan2(2.0 * (q.y() * q.x() + q.z() * q.w()) ,  (1.0 - 2.0*(q.y() * q.y() + q.z() *
        // q.z())))/M_PI*180.0

        T t1 = (T)((T)q.y() * (T)q.y());
        T t2 = 2 * t1;
        T t3 = (T)q.z() * (T)q.z();
        T t5 = 1 - t2 - 2 * t3;
        T t7 = t5 * t5;
        T t10 = (T)((T)q.w() * (T)q.z() + (T)q.x() * (T)q.y());
        T t13 = 1 / (t7 + 4 * t10 * t10);
        T t31 = (T)q.x() * (T)q.x();
        T t34 = 1 - 2 * t31 - 2 * t1;
        T t35 = t34 * t34;
        T t38 = (T)q.y() * (T)q.z() + (T)q.w() * (T)q.x();
        T t39 = (T)(t38 * t38);
        T t40 = 4 * t39;
        T t42 = sqrt(t35 + t40);
        T t47 = -(T)q.x() * (T)q.z() + (T)q.w() * (T)q.y();
        T t48 = 2 * t47 * t38;
        T t49 = 1 / t42;
        T t56 = 1 / (t35 + t40 + 4 * t47 * t47);
        T t90 = 1 - 2 * t31 - t2;
        T t92 = t90 * t90;
        T t95 = 1 / (t92 + 4 * t38 * t38);

        J << 2 * q.z() * t5 * t13, 2 * q.y() * t5 * t13, (2 * q.x() * t5 + 8 * t10 * q.y()) * t13,
            (2 * q.w() * t5 + 8 * t10 * q.z()) * t13, (2 * (T)q.y() * t42 - 4 * (T)t48 * (T)q.x() * t49) * t56,
            (-2 * (T)q.z() * t42 - (T)t47 * (-8 * t34 * (T)q.x() + 8 * (T)t38 * (T)q.w()) * t49) * t56,
            (2 * (T)q.w() * t42 - (T)t47 * (-8 * t34 * (T)q.y() + 8 * (T)t38 * (T)q.z()) * t49) * t56,
            (-2 * (T)q.x() * t42 - 4 * (T)t48 * (T)q.y() * t49) * t56, 2 * (T)q.x() * t90 * t95,
            (2 * (T)q.w() * t90 + 8 * t38 * (T)q.x()) * t95, (2 * (T)q.z() * t90 + 8 * t38 * (T)q.y()) * t95,
            2 * (T)q.y() * t90 * t95;

        // from technical report
        // pitch = asin(2.0*(q.w()*q.y()-q.x()*q.z())); (differs from eigen)
        // roll  = atan2(2.0*(q.w()*q.x()+q.y()*q.z()), (1.0 - 2.0*(q.x()*q.x()+q.y()*q.y()))); (like eigen)
        // yaw   = atan2(2.0*(q.w()*q.z()+q.x()*q.y()), (1.0 - 2.0*(q.y()*q.y()+q.z()*q.z()))); (like eigen)

        // T t1 = (T) ((T) q.y() * (T) q.y());
        // T t2 = 2 * t1;
        // T t3 = (T) ((T) q.z() * (T) q.z());
        // T t5 = 1 - t2 - 2 * t3;
        // T t7 = t5 * t5;
        // T t10 = (T) ((T) q.w() * (T) q.z() + (T) q.x() * (T) q.y());
        // T t13 = 1 / (t7 + 4 * t10 * t10);
        // T t31 = (T) ((T) q.w() * (T) q.w());
        // T t38 = (T) ((T) q.x() * (T) q.x());
        // T t42 = sqrt((T) (1 - 4 * t31 * t1 + 8 * q.w() * q.y() * q.x() * q.z() - 4 * t38 * t3));
        // T t43 = 0.1e1 / t42;
        // T t53 = 1 - 2 * t38 - t2;
        // T t55 = t53 * t53;
        // T t58 = q.w() * q.x() + q.y() * q.z();
        // T t61 = 1 / (t55 + 4 * t58 * t58);
        //
        // J.template block<3,4>(3,3) <<
        //  2 * q.z() * t5 * t13,
        //  2 * q.y() * t5 * t13,
        //  (2 * q.x() * t5 + 8 * t10 * q.y()) * t13,
        //  (2 * q.w() * t5 + 8 * t10 * q.z()) * t13,
        //  (T) (0.2e1 * (T) q.y() * t43),
        //  -(T) (0.2e1 * (T) q.z() * t43),
        //  (T) (0.2e1 * (T) q.w() * t43),
        //  -(T) (0.2e1 * (T) q.x() * t43),
        //  2 * q.x() * t53 * t61,
        //  (2 * q.w() * t53 + 8 * t58 * q.x()) * t61,
        //  (2 * q.z() * t53 + 8 * t58 * q.y()) * t61,
        //  2 * q.y() * t53 * t61;
    } else {

        // for eigen
        T t1 = q.x() * q.x();
        T t3 = q.y() * q.y();
        T t5 = (T)0.10e1 - (T)0.20e1 * t1 - (T)0.20e1 * t3;
        T t6 = t5 * t5;
        T t9 = q.y() * q.z() + q.w() * q.x();
        T t10 = t9 * t9;
        T t11 = (T)0.400e1 * t10;
        T t13 = sqrt(t6 + t11);
        T t16 = q.x() * q.z();
        T t18 = q.w() * q.y();
        T t20 = (T)-0.20e1 * t16 + (T)0.20e1 * t18;
        T t22 = (T)0.1e1 / t13;
        T t27 = t20 * t20;
        T t33 = -t16 + t18;
        T t45 = (T)0.1e1 / (t6 + t11 + (T)0.4e1 * t33 * t33);
        T t67 = (T)0.2e1 * t1;
        T t68 = (T)(q.z() * q.z());
        T t70 = (T)0.1e1 - t67 - (T)(2 * t68);
        T t72 = t70 * t70;
        T t77 = (T)-0.20e1 * q.x() * q.y() + (T)0.20e1 * q.w() * q.z();
        T t78 = t77 * t77;
        T t80 = (T)0.1e1 / (t72 + t78);
        T t96 = (T)0.1e1 - t67 - (T)0.2e1 * t3;
        T t98 = t96 * t96;

        // for technical report
        // T t1 = (T) q.w() * (T) q.w();
        // T t2 = (T) q.x() * (T) q.x();
        // T t4 = 1 / (T) (t1 + t2);

        if (q.x() * q.z() - q.w() * q.y() > 0) {
            // from eigen quaternionToYawPitchRoll
            // pitch = atan2(-2.0*(q.x()*q.z()-q.w()*q.y()), c)/M_PI*180.0
            // roll = +atan2(-2.0 * (q.x()*q.y()-q.w() * q.z()), (1.0 - 2.0*(q.x() * q.x() + q.z() * q.z())))/M_PI *
            // 180.0 yaw = 0

            J << 0, 0, 0, 0, (T)((0.20e1 * q.y() * t13 - 0.4000000000e1 * t20 * t9 * q.x() * t22) / (t6 + t11 + t27)),
                (T)((-0.2e1 * q.z() * t13 - t33 * (-0.80e1 * q.x() * t5 + 0.800e1 * t9 * q.w()) * t22) * t45),
                (T)((0.2e1 * q.w() * t13 - t33 * (-0.80e1 * t5 * q.y() + 0.800e1 * t9 * q.z()) * t22) * t45),
                (T)((-0.2e1 * q.x() * t13 - 0.8000000000e1 * t33 * t9 * q.y() * t22) * t45),
                (T)(0.20e1 * q.z() * t70 * t80), (T)((-0.20e1 * q.y() * t70 + 0.4e1 * t77 * q.x()) * t80),
                -(T)(0.20e1 * q.x() * t70 / (t72 * t70 + t78)), (T)(0.2e1 * q.y() * t96 / (t98 + 0.4e1 * t9 * t9));

            // from technical report
            // pitch = -M_PI/2.0; (differs from eigen)
            // roll  = 0; (differs from eigen)
            // yaw   = 2.0 * atan2(q.x(), q.w()); (differs from eigen)

            // J.template block<3,4>(3,3) <<
            //      -2 * q.x() * t4, 2 * q.w() * t4, 0, 0,
            //      0,0,0,0,
            //      0,0,0,0;
        } else {
            // from eigen quaternionToYawPitchRoll
            // pitch = atan2(-2.0*(q.x()*q.z()-q.w()*q.y()), c)/M_PI*180.0
            // roll = -atan2(-2.0 * (q.x()*q.y()-q.w() * q.z()), (1.0 - 2.0*(q.x() * q.x() + q.z() * q.z())))/M_PI *
            // 180.0 yaw = 0

            J << 0, 0, 0, 0,

                (T)((0.20e1 * q.y() * t13 - 0.4000000000e1 * t20 * t9 * q.x() * t22) / (t6 + t11 + t27)),
                (T)((-0.2e1 * q.z() * t13 - t33 * (-0.80e1 * q.x() * t5 + 0.800e1 * t9 * q.w()) * t22) * t45),
                (T)((0.2e1 * q.w() * t13 - t33 * (-0.80e1 * t5 * q.y() + 0.800e1 * t9 * q.z()) * t22) * t45),
                (T)((-0.2e1 * q.x() * t13 - 0.8000000000e1 * t33 * t9 * q.y() * t22) * t45),

                -(T)(0.20e1 * q.z() * t70 * t80), -(T)((-0.20e1 * q.y() * t70 + 0.4e1 * t77 * q.x()) * t80),
                (T)(0.20e1 * q.x() * t70 / (t72 * t70 + t78)), -(T)(0.2e1 * q.y() * t96 / (t98 + 0.4e1 * t9 * t9));

            // from technical report
            // pitch = M_PI/2.0; (differs from eigen)
            // roll  = 0; (differs from eigen)
            // yaw   = -2.0 * atan2(q.x(), q.w()); (differs from eigen)

            // J.template block<3,4>(3,3) <<
            //      2 * q.x() * t4, -2 * q.w() * t4, 0, 0,
            //      0,0,0,0,
            //      0,0,0,0;
        }
    }

    return J * covariance * J.transpose();
}

class GcpCostFunctor {
public:
    GcpCostFunctor(const LiDARCalibrate *_cal, int _idx) : cal(_cal), idx(_idx) {}

    template <typename T> bool operator()(const T *const ptr_dq_body, const T *const ptr_dt_body, T *residual) const {
        using Vector3JetT = Eigen::Matrix<T, 3, 1>;
        using QuaternionJetT = Eigen::Quaternion<T>;

        QuaternionJetT dq_body(ptr_dq_body);
        Vector3JetT dt_body(ptr_dt_body);

        const Quaterniond &q_body = cal->q_body_;
        const Vector3d &t_body = cal->t_body_;
        const LiDARCalibrate::GCP &gcp = cal->gcps_[idx];

        Vector3JetT p_lidar = gcp.q_pos.cast<T>() * (dq_body * q_body.cast<T>() * gcp.p_lidar_local.cast<T>() +
                                                     t_body.cast<T>() + dt_body) +
                              gcp.t_pos.cast<T>();

        residual[0] = p_lidar(0) - gcp.p_gcp(0);
        residual[1] = p_lidar(1) - gcp.p_gcp(1);
        residual[2] = p_lidar(2) - gcp.p_gcp(2);

        return true;
    }

private:
    const LiDARCalibrate *cal;
    int idx;
};

class TpsCostFunctor {
public:
    TpsCostFunctor(const LiDARCalibrate *_cal, int _idx) : cal(_cal), idx(_idx) {}

    template <typename T> bool operator()(const T *const ptr_dq_body, const T *const ptr_dt_body, T *residual) const {
        using Vector3JetT = Eigen::Matrix<T, 3, 1>;
        using QuaternionJetT = Eigen::Quaternion<T>;

        QuaternionJetT dq_body(ptr_dq_body);
        Vector3JetT dt_body(ptr_dt_body);

        const Quaterniond &q_body = cal->q_body_;
        const Vector3d &t_body = cal->t_body_;
        const LiDARCalibrate::TPS &tps = cal->tps_[idx];

        Vector3JetT p_lidar1 =
            tps.q_pos1.cast<T>() * (dq_body * q_body.cast<T>() * tps.p_local1.cast<T>() + t_body.cast<T>() + dt_body) +
            tps.t_pos1.cast<T>();

        Vector3JetT p_lidar2 =
            tps.q_pos2.cast<T>() * (dq_body * q_body.cast<T>() * tps.p_local2.cast<T>() + t_body.cast<T>() + dt_body) +
            tps.t_pos2.cast<T>();

        residual[0] = p_lidar1(0) - p_lidar2(0);
        residual[1] = p_lidar1(1) - p_lidar2(1);
        residual[2] = p_lidar1(2) - p_lidar2(2);

        return true;
    }

private:
    const LiDARCalibrate *cal;
    int idx;
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
            new ceres::AutoDiffCostFunction<GcpCostFunctor, 3, 4, 3>(new GcpCostFunctor(this, i));

        auto block = problem_->AddResidualBlock(
            cost,
            new ceres::ScaledLoss(
                nullptr, 1.0 / (param_.std_lidar_point * param_.std_lidar_point * num_gcps) * param_.lambda_gcp,
                ceres::TAKE_OWNERSHIP),
            dq_body_.coeffs().data(), dt_body_.data());
        blocks_gcps_.push_back(block);
    }
    for (int i = 0; i < tps_.size(); ++i) {
        ceres::CostFunction *cost =
            new ceres::AutoDiffCostFunction<TpsCostFunctor, 3, 4, 3>(new TpsCostFunctor(this, i));
        auto block = problem_->AddResidualBlock(
            cost,
            new ceres::ScaledLoss(nullptr,
                                  1.0 / (param_.std_lidar_point * param_.std_lidar_point * num_tps) * param_.lambda_tps,
                                  ceres::TAKE_OWNERSHIP),
            dq_body_.coeffs().data(), dt_body_.data());
        blocks_tps_.push_back(block);
    }

    {
        // 加入角度信息的带权观测值
        ceres::CostFunction *cost = new ceres::AutoDiffCostFunction<BodyRotationRegularizationCostFunctor, 1, 4>(
            new BodyRotationRegularizationCostFunctor());
        problem_->AddResidualBlock(
            cost,
            new ceres::ScaledLoss(nullptr, 1.0 / (param_.std_body_angle * param_.std_body_angle) * param_.lambda_body,
                                  ceres::TAKE_OWNERSHIP),
            dq_body_.coeffs().data());
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
            dt_body_.data());
    }

    ceres::LocalParameterization *quaternion_parametrization = new ceres::EigenQuaternionParameterization();
    problem_->SetParameterization(dq_body_.coeffs().data(), quaternion_parametrization);

    if (!param_.with_translate) {
        problem_->SetParameterBlockConstant(dt_body_.data());
    }

    ceres::Solver::Options op;
    op.linear_solver_type = ceres::DENSE_QR;
    op.num_threads = Eigen::nbThreads();

    ceres::Solver::Summary summary;
    ceres::Solve(op, problem_.get(), &summary);

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

    LOG(INFO) << string_printf("Reserved %d/%d points with sigma0 %.2lf", reserved.size(), tps_.size(), sigma_pixel);

    // 移除对应 id 的 track
    auto tps_clone = tps_;
    tps_.clear();
    for (int id : reserved) {
        tps_.push_back(tps_clone[id]);
    }
}

void LiDARCalibrate::evaluate() {
    double sigma0;
    {
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
        sigma0 = std::sqrt(cost / (residuals.size() - 6));
    }

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

    {
        // 计算 covariance
        ceres::Covariance::Options options;
        options.apply_loss_function = false;
        ceres::Covariance covariance(options);
        std::vector<std::pair<const double *, const double *>> covariance_blocks;
        covariance_blocks.push_back(std::make_pair(dq_body_.coeffs().data(), dq_body_.coeffs().data()));
        covariance_blocks.push_back(std::make_pair(dt_body_.data(), dt_body_.data()));

        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> cov_q = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>::Zero();
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cov_t = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Zero();

        covariance.Compute(covariance_blocks, problem_.get());
        covariance.GetCovarianceBlock(dt_body_.data(), dt_body_.data(), cov_t.data());
        covariance.GetCovarianceBlock(dq_body_.coeffs().data(), dq_body_.coeffs().data(), cov_q.data());

        cov_t = sigma0 * sigma0 * cov_t;
        Matrix4d cov_q2 = cov_q;
        Matrix3d cov_r = quaternionCovToYawPitchRollCov(cov_q2, dq_body_);
        cov_r = sigma0 * sigma0 * cov_r;

        LOG(INFO) << string_printf("Sigma_x for t is [%.3lf,%.3lf,%.3lf]", std::sqrt(cov_t(0, 0)),
                                   std::sqrt(cov_t(1, 1)), std::sqrt(cov_t(2, 2)));
        LOG(INFO) << string_printf("Sigma_x for r is [%.3lf,%.3lf,%.3lf]", std::sqrt(cov_r(0, 0)) * RAD2DEG,
                                   std::sqrt(cov_r(1, 1)) * RAD2DEG, std::sqrt(cov_r(2, 2)) * RAD2DEG);
    }
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
LiDARCalibrate::Config LiDARCalibrate::Config::load_config(const std::string &path) {
    LiDARCalibrate::Config config;
    nlohmann::json j = read_json_file(path);
    if (j.count("LiDARCalibrate") == 0) {
        return config;
    }

    j = j["LiDARCalibrate"];

    if (j.count("WithTranslate"))
        config.with_translate = j["WithTranslate"];
    if (j.count("Epoch"))
        config.epoch_clean = j["Epoch"];
    if (j.count("StdBodyAngle"))
        config.std_body_angle = j["StdBodyAngle"] * DEG2RAD;
    if (j.count("StdBodyTrans"))
        config.std_body_trans = j["StdBodyTrans"];
    if (j.count("StdLiDARPoint"))
        config.std_lidar_point = j["StdLiDARPoint"];
    if (j.count("LambdaGcp"))
        config.lambda_gcp = j["LambdaGcp"];
    if (j.count("LambdaTps"))
        config.lambda_tps = j["LambdaTps"];
    if (j.count("LambdaBody"))
        config.lambda_body = j["LambdaBody"];

    return config;
}
} // namespace h2o
