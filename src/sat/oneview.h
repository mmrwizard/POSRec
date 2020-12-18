/*!
 * \file oneview.h
 *
 * \author Han
 * \date 2017/05/04
 *
 *
 */
#pragma once
#include <base/geo_image.h>

#include "camera_polynomial.h"
#include "camera_rpc.h"

namespace h2o {

/**
 * \brief 将三维点投影到影像
 * \param latlon [lat lon h] 单位度和米
 * \param camera
 * \return 像素坐标
 */
Vector2d project(const Vector3d &latlon, const RpcCamera &camera);
std::vector<Vector2d> project(const std::vector<Vector3d> &latlon, const RpcCamera &camera);

/**
 * \brief 采用多项式模型进行投影
 * \param point 地心坐标系坐标单位米
 * \param camera
 */
Vector2d project(const Vector3d &point, const PolynomialCamera &camera);
std::vector<Vector2d> project(const std::vector<Vector3d> &points, const PolynomialCamera &camera);

/**
 * \brief 采用投影矩阵投影
 */
Vector2d project(const Vector3d &point, const MatrixXd &camera);

/**
 * \brief 从旋转矩阵R获取 rodrigues 的旋转大小
 */
Vector3d rodrigues(const Matrix3d &R);

/**
 * \brief 从 rodrigues 旋转的三个数值获取旋转矩阵
 */
Matrix3d rodrigues(const Vector3d &r);

/**
 * \brief 根据多项式模型，将一个点投影到 DEM 上
 * \return [lat lon z]
 */
Vector3d project_to_geographic_dem(const PolynomialCamera &camera, const GeoImage &dem, const Vector2d &pixel,
                                   const std::string &planet = "moon");
/**
 * \brief 将一点投影到无妨地理坐标系的一个高程z
 * \param camera 相机模型，外方位元素为地心坐标系，因此需要转换
 * \param z 物方地理坐标系下的高程值
 * \param pixel 影像像素
 * \param 椭球体
 * \return [lat, lon, z]
 */
Vector3d project_to_geographic_z(const PolynomialCamera &camera, double z, const Vector2d &pixel,
                                 const std::string &planet = "moon");
Vector3d project_to_geographic_z(const PolynomialCamera &camera, double z, const Vector2d &pixel, double a, double f);

/**
 * \brief 采用RPC参数将像方一点投影到物方
 */
Vector3d project_to_geographic_z(const RpcCamera &camera, double z, const Vector2d &pixel);
Vector3d project_to_geographic_z_ceres(const RpcCamera &camera, double z, const Vector2d &pixel);

/**
 * \brief 给定一个物方经纬度坐标系的DEM，获取影像覆盖范围
 * \param camera 相机参数
 * \param dem 物方 DEM
 */
BoundingBox3d geographic_region(const PolynomialCamera &camera, const GeoImage &dem);

///\brief 给定物方一高程值 z 和相机信息，获取覆盖的经纬度范围
BoundingBox2d geographic_region(const PolynomialCamera &camera, double z);

///\brief 直接根据 RPC 参数信息获取影像覆盖范围
BoundingBox3d geographic_region(const RpcCamera &camera);
/**
 * \brief 根据物方一平面，确定相机的单应关系
 * \param plane ax+by+cz+d=0
 * \param P 投影矩阵
 * \return [H, idx] 由于平面在椭球体下方向可以任意，因此不是简单的 XY 平面，该平面根据 idx 确定
 */
std::tuple<Matrix3d, int> geocentric_plane_homography(const Vector4d &plane, const MatrixXd &P);
} // namespace h2o
