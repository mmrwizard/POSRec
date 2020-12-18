/*!
 * \file camera_rpc_ceres.h
 *
 * \author Han
 * \date 2017/05/04
 *
 * RPC 是非线性模型，很多函数需要用到最小二乘优化
 */

#pragma once
#include <base/common.h>

#include "camera_rpc.h"

#include <ceres/ceres.h>

namespace h2o {
class RpcProjectionXY {
public:
    /*************************************************************************/
    /**
     * \brief 根据影像坐标、RPC参数、仿射变换参数、物方平面高度计算物方坐标
     * \param[in]    sample 观测值
     * \param[in]    line 观测值
     * \param[in]    height 物方高度
     * \param[in]    _rpc rpc 参数
     * \param[in]    _affine 仿射畸变参数
     */
    /*************************************************************************/
    RpcProjectionXY(double sample, double line, double height, const RpcCamera &_camera)
        : s(sample), l(line), h(height), camera(_camera) {}

    template <typename T> bool operator()(const T *const latlon, T *residuals) const {
        Eigen::Matrix<T, 20, 1> c, d, e, f;

        const auto rpc = camera.rpc;
        const auto affine = camera.rpc_aff;
        for (int i = 0; i < 20; ++i) {
            c(i) = (T)rpc.line_num_coeff(i);
            d(i) = (T)rpc.line_den_coeff(i);
            e(i) = (T)rpc.samp_num_coeff(i);
            f(i) = (T)rpc.samp_den_coeff(i);
        }

        T P, L, H;
        P = (latlon[0] - rpc.lat_off) / rpc.lat_scale;
        L = (latlon[1] - rpc.long_off) / rpc.long_scale;
        H = (T(h) - rpc.height_off) / rpc.height_scale;

        T u_data[20] = {T(1.0),    L,         P,         H,         L * P,     L * H,     P * H,
                        L * L,     P * P,     H * H,     P * L * H, L * L * L, L * P * P, L * H * H,
                        L * L * P, P * P * P, P * H * H, L * L * H, P * P * H, H * H * H};
        Eigen::Matrix<T, Eigen::Dynamic, 1> u = Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>>(u_data, 20);

        T Y = u.dot(c) / u.dot(d);
        T X = u.dot(e) / u.dot(f);
        T line = Y * rpc.line_scale + rpc.line_off;
        T sample = X * rpc.samp_scale + rpc.samp_off;

        T sample_ = affine(0, 0) * sample + affine(0, 1) * line + affine(0, 2);
        T line_ = affine(1, 0) * sample + affine(1, 1) * line + affine(1, 2);

        residuals[0] = sample_ - s;
        residuals[1] = line_ - l;

        return true;
    }

private:
    double s, l;
    double h;
    const RpcCamera &camera;
};

class RpcProjection {
public:
    RpcProjection(const Vector2d &_point, const RpcCamera &_camera) : point(_point), camera(_camera) {}

    template <typename T> bool operator()(const T *const latlon, T *residuals) const {
        Eigen::Matrix<T, 20, 1> c, d, e, f;

        const auto rpc = camera.rpc;
        const auto affine = camera.rpc_aff;
        for (int i = 0; i < 20; ++i) {
            c(i) = (T)rpc.line_num_coeff(i);
            d(i) = (T)rpc.line_den_coeff(i);
            e(i) = (T)rpc.samp_num_coeff(i);
            f(i) = (T)rpc.samp_den_coeff(i);
        }

        T P, L, H;
        P = (latlon[0] - rpc.lat_off) / rpc.lat_scale;
        L = (latlon[1] - rpc.long_off) / rpc.long_scale;
        H = (latlon[2] - rpc.height_off) / rpc.height_scale;

        T u_data[20] = {T(1.0),    L,         P,         H,         L * P,     L * H,     P * H,
                        L * L,     P * P,     H * H,     P * L * H, L * L * L, L * P * P, L * H * H,
                        L * L * P, P * P * P, P * H * H, L * L * H, P * P * H, H * H * H};
        Eigen::Matrix<T, 20, 1> u(u_data);

        T Y = u.dot(c) / u.dot(d);
        T X = u.dot(e) / u.dot(f);
        T line = Y * rpc.line_scale + rpc.line_off;
        T sample = X * rpc.samp_scale + rpc.samp_off;

        T sample_ = affine(0, 0) * sample + affine(0, 1) * line + affine(0, 2);
        T line_ = affine(1, 0) * sample + affine(1, 1) * line + affine(1, 2);

        residuals[0] = sample_ - point(0);
        residuals[1] = line_ - point(1);

        return true;
    }

private:
    Vector2d point;
    const RpcCamera &camera;
};

class RpcFitCost {
public:
    RpcFitCost(const RpcCamera &_camera, const Vector3d &_latlon, const Vector2d &_point)
        : camera(_camera), latlon(_latlon), point(_point) {}
    template <typename T> bool operator()(const T *const rpc_coeff, T *residuals) const {
        Eigen::Matrix<T, 20, 1> c, d, e, f;

        const auto rpc = camera.rpc;
        const auto affine = camera.rpc_aff;
        for (int i = 0; i < 20; ++i) {
            c(i) = rpc_coeff[i];
            d(i) = rpc_coeff[i + 20];
            e(i) = rpc_coeff[i + 40];
            f(i) = rpc_coeff[i + 60];
        }

        double P, L, H;
        P = (latlon[0] - rpc.lat_off) / rpc.lat_scale;
        L = (latlon[1] - rpc.long_off) / rpc.long_scale;
        H = (latlon[2] - rpc.height_off) / rpc.height_scale;

        double u[20] = {1.0,       L,         P,         H,         L * P,     L * H,     P * H,
                        L * L,     P * P,     H * H,     P * L * H, L * L * L, L * P * P, L * H * H,
                        L * L * P, P * P * P, P * H * H, L * L * H, P * P * H, H * H * H};

        T Y, X;
        {
            T Y_num(0.0), Y_den(0.0);
            T X_num(0.0), X_den(0.0);
            for (int i = 0; i < 20; ++i) {
                Y_num += c[i] * u[i];
                Y_den += d[i] * u[i];

                X_num += e[i] * u[i];
                X_den += f[i] * u[i];
            }
            Y = Y_num / Y_den;
            X = X_num / X_den;
        }

        T line = Y * rpc.line_scale + rpc.line_off;
        T sample = X * rpc.samp_scale + rpc.samp_off;

        residuals[0] = sample - point(0);
        residuals[1] = line - point(1);

        return true;
    }

private:
    const RpcCamera &camera;
    Vector3d latlon;
    Vector2d point;
};

/**
 * 用 RPC 投影来优化放射变换参数
 */
class RpcProjectionBa {
public:
    RpcProjectionBa(const Vector2d &_point, const RpcCamera &_camera, double w)
        : point(_point), camera(_camera), weight(w) {}
    template <typename T> bool operator()(const T *const latlon, const T *const affine, T *residuals) const {
        Eigen::Matrix<T, 20, 1> c, d, e, f;

        const auto &rpc = camera.rpc;

        for (int i = 0; i < 20; ++i) {
            c(i) = (T)rpc.line_num_coeff(i);
            d(i) = (T)rpc.line_den_coeff(i);
            e(i) = (T)rpc.samp_num_coeff(i);
            f(i) = (T)rpc.samp_den_coeff(i);
        }

        T P, L, H;
        P = (latlon[0] - rpc.lat_off) / rpc.lat_scale;
        L = (latlon[1] - rpc.long_off) / rpc.long_scale;
        H = (latlon[2] - rpc.height_off) / rpc.height_scale;

        T u_data[20] = {T(1.0),    L,         P,         H,         L * P,     L * H,     P * H,
                        L * L,     P * P,     H * H,     P * L * H, L * L * L, L * P * P, L * H * H,
                        L * L * P, P * P * P, P * H * H, L * L * H, P * P * H, H * H * H};
        Eigen::Matrix<T, 20, 1> u(u_data);

        T Y = u.dot(c) / u.dot(d);
        T X = u.dot(e) / u.dot(f);
        T line = Y * rpc.line_scale + rpc.line_off;
        T sample = X * rpc.samp_scale + rpc.samp_off;

        T sample_ = affine[0] * sample + affine[2] * line + affine[4];
        T line_ = affine[1] * sample + affine[3] * line + affine[5];

        residuals[0] = (sample_ - point(0)) * weight;
        residuals[1] = (line_ - point(1)) * weight;

        return true;
    }

private:
    double weight;
    Vector2d point;
    const RpcCamera &camera;
};

/**
 * 仿射变换参数的虚拟观测值
 */
class RpcAffineCost {
public:
    RpcAffineCost(const MatrixXd &_aff, double r, double t) : aff(_aff), w_rot(r), w_lin(t) {}
    ~RpcAffineCost() {}

    template <typename T> bool operator()(const T *const data, T *residuals) const {
        residuals[0] = (data[0] - aff(0, 0)) * w_rot;
        residuals[1] = (data[1] - aff(1, 0)) * w_rot;
        residuals[2] = (data[2] - aff(0, 1)) * w_rot;
        residuals[3] = (data[3] - aff(1, 1)) * w_rot;
        residuals[4] = (data[4] - aff(0, 2)) * w_lin;
        residuals[5] = (data[5] - aff(1, 2)) * w_lin;

        return true;
    }

private:
    // 初值
    MatrixXd aff;

    ///\brief 仿射变换参数旋转部分的权重
    double w_rot;

    ///\brief 仿射变换参数线性部分的权重
    double w_lin;
};

struct DemHeightCost {
public:
    DemHeightCost(double _height, double _w) : height(_height), weight(_w) {}
    template <typename T> bool operator()(const T *const latlon, T *residuals) const {
        residuals[0] = (latlon[2] - height) * weight;
        return true;
    }

private:
    double weight;
    double height;
};

} // namespace h2o
