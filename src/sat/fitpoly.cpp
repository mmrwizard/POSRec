/*!
 * \file polyfit.cpp
 *
 * \author Han
 * \date 2017/05/05
 *
 *
 */
#include <sat/fitpoly.h>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <Eigen/SVD>

namespace h2o {
VectorXd fitpoly(const VectorXd &x, const VectorXd &y, int order) {
    CHECK(x.size() == y.size());
    MatrixXd A(x.rows(), order + 1);
    VectorXd X(order + 1);
    MatrixXd b = y;

    for (int i = 0; i < x.rows(); ++i) {
        for (int j = 0; j <= order; ++j) {
            A(i, j) = pow(x(i), j);
        }
    }
    X = A.householderQr().solve(b);

    return X;
}

Vector3d fitline(const std::vector<Vector2d> &points) {
    // 采用主成分分析拟合直线，最小特征值对应的特征向量方向为直线的法相
    Matrix2Xd mat_points(2, points.size());
    for (int i = 0; i < points.size(); ++i) {
        mat_points.col(i) = points[i];
    }
    Vector2d center = mat_points.rowwise().mean();
    mat_points = mat_points.colwise() - center;
    Eigen::JacobiSVD<Eigen::Matrix2Xd> svd = mat_points.jacobiSvd(Eigen::ComputeFullU);
    Matrix2d U = svd.matrixU();

    typedef Eigen::Hyperplane<double, 2> Line2d;
    Line2d line(U.col(1), center);
    return line.coeffs();
}
}
