/*!
 * \file nview.h
 *
 * \author Han
 * \date 2017/05/04
 *
 * 多视几何处理
 */
#pragma once

#include "camera_polynomial.h"
#include "camera_rpc.h"

namespace h2o {

/**
 * \brief 单个点的 rpc 相机模型的多是前方交会
 * \param points 匹配点
 * \param cameras 相机信息，和前面的匹配点事一一对应的
 * \return [lat lon h]
 */
Vector3d intersect(const std::vector<Vector2d> &points, const std::vector<RpcCamera> &cameras);

/**
 * \brief 采用多项式相机模型的多视前方交会
 * \param points 匹配点，像素
 * \param cameras 相机信息
 * \return geocentric
 */
Vector3d intersect(const std::vector<Vector2d> &points, const std::vector<PolynomialCamera> &cameras);

Vector3d intersect(const std::vector<Vector2d> &points, const std::vector<MatrixXd> &cameras);

std::vector<Vector3d> intersect(const std::vector<std::unordered_map<iid_t, Vector2d>> &points,
                                const RpcCameras &cameras);
std::vector<Vector3d> intersect_ceres(const std::vector<std::unordered_map<iid_t, Vector2d>> &points,
                                      const RpcCameras &cameras);

std::vector<Vector3d> intersect(const std::vector<std::unordered_map<iid_t, Vector2d>> &points,
                                const PolynomialCameras &cameras);

std::vector<Vector3d> intersect(const std::vector<std::unordered_map<iid_t, Vector2d>> &points,
                                const std::unordered_map<iid_t, MatrixXd> &cameras);
} // namespace h2o
