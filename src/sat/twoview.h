/*!
 * \file twoview.h
 *
 * \author Han
 * \date 2017/05/04
 *
 * 两视几何处理
 */
#pragma once

#include "camera_polynomial.h"
#include "camera_rpc.h"

namespace h2o {

///\brief 采用RPC参数进行前方交会
Vector3d intersect(const Vector2d &fwd_point, const Vector2d &bwd_point, const RpcCamera &fwd_camera,
                   const RpcCamera &bwd_camera);

Vector3d intersect_rpc_ceres(const Vector2d &fwd_point, const Vector2d &bwd_point, const RpcCamera &fwd_camera,
                             const RpcCamera &bwd_camera);

///\brief 采用多项式模型相机进行前方交会
Vector3d intersect(const Vector2d &fwd_point, const Vector2d &bwd_point, const PolynomialCamera &fwd_camera,
                   const PolynomialCamera &bwd_camera);

///\brief 采用P矩阵进行前方交会
Vector3d intersect(const Vector2d &fwd_point, const Vector2d &bwd_point, const MatrixXd &fwd_camera,
                   const MatrixXd &bwd_camera);

///\brief 前方交会多个点
std::vector<Vector3d> intersect(const std::vector<Vector2d> &fwd_points, const std::vector<Vector2d> &bwd_points,
                                const RpcCamera &fwd_camera, const RpcCamera &bwd_camera);

std::vector<Vector3d> intersect(const std::vector<Vector2d> &fwd_points, const std::vector<Vector2d> &bwd_points,
                                const PolynomialCamera &fwd_camera, const PolynomialCamera &bwd_camera);

std::vector<Vector3d> intersect(const std::vector<Vector2d> &fwd_points, const std::vector<Vector2d> &bwd_points,
                                const MatrixXd &fwd_camera, const MatrixXd &bwd_camera);
} // namespace h2o
