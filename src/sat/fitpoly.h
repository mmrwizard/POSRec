/*!
 * \file polyfit.h
 *
 * \author Han
 * \date 2017/05/05
 *
 * 多项式内插，多项式结构为 a0 + a1x + a2x^2 + ... = 0;
 */
#pragma once
#include <base/common.h>

namespace h2o {
///\brief y = a0 + a1x + a2x^2 + a3x^3 + ...
VectorXd fitpoly(const VectorXd &x, const VectorXd &y, int order);

///\brief 拟合二维直线 ax+by+c=0
Vector3d fitline(const std::vector<Vector2d> &points);
} // namespace h2o
