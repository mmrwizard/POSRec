/*
 * Created Date: Tuesday, January 12th 2021, 12:04:15 pm
 * Author: Han
 * 一些典型的 cost
 * Copyright (c) 2021 Vrlab
 */
#pragma once

#include <base/common.h>

namespace h2o {

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

} // namespace h2o
