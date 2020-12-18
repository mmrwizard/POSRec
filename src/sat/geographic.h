/*!
 * \file geographic.h
 *
 * \author Han
 * \date 2017/05/05
 *
 * 坐标转换
 */
#pragma once
#include <base/common.h>
namespace h2o {

/**
 * \brief 将经纬度坐标转换为地心坐标
 * \param latlon [lat lon h] 单位度和米
 * \param planet 椭球 moon earth mars
 */
std::vector<Vector3d> latlon_to_geocentric(const std::vector<Vector3d> &latlon, const std::string &planet);
std::vector<Vector3d> latlon_to_geocentric(const std::vector<Vector3d> &latlon, double a, double f);
Vector3d latlon_to_geocentric(const Vector3d &latlon, const std::string &planet);
Vector3d latlon_to_geocentric(const Vector3d &latlon, double a, double f);

/**
 * \brief 将地心坐标转换为经纬度坐标
 * \param geocentric [X Y Z]
 * \param planet 椭球 moon earth mars
 */
std::vector<Vector3d> geocentric_to_latlon(const std::vector<Vector3d> &geocentric, const std::string &planet);
std::vector<Vector3d> geocentric_to_latlon(const std::vector<Vector3d> &geocentric, double a, double f);
Vector3d geocentric_to_latlon(const Vector3d &geocentric, const std::string &planet);
Vector3d geocentric_to_latlon(const Vector3d &geocentric, double a, double f);
} // namespace h2o
