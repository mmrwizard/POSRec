/*!
 * \file twoview.cpp
 *
 * \author Han
 * \date 2017/05/04
 *
 * 两视几何处理
 */

#include "twoview.h"
#include "camera_rpc_ceres.h"
#include "oneview.h"

#include <Eigen/SVD>
namespace h2o {

Vector3d intersect(const Vector2d &fwd_point, const Vector2d &bwd_point, const RpcCamera &fwd_camera,
                   const RpcCamera &bwd_camera) {

    const auto &rpc1 = fwd_camera.rpci;
    const auto &rpc2 = bwd_camera.rpci;

    int niter = 5;
    double X1, Y1, X2, Y2;
    Vector2d pt1_undistort = fwd_camera.rpc_aff_inv * fwd_point.homogeneous();
    Vector2d pt2_undistort = bwd_camera.rpc_aff_inv * bwd_point.homogeneous();
    X1 = (pt1_undistort(0) - rpc1.samp_off) / rpc1.samp_scale;
    Y1 = (pt1_undistort(1) - rpc1.line_off) / rpc1.line_scale;
    X2 = (pt2_undistort(0) - rpc2.samp_off) / rpc2.samp_scale;
    Y2 = (pt2_undistort(1) - rpc2.line_off) / rpc2.line_scale;

    double z = (rpc1.height_off + rpc2.height_off) * 0.5;
    double lat1, lat2, lon1, lon2;
    double dlat1, dlat2, dlon1, dlon2;
    // 迭代求解物方坐标系的高程Z
    bool stop = false;
    for (int i = 0; i < niter; ++i) {
        for (int o = 0; o < 2; ++o) {
            const auto &rpc = o == 0 ? rpc1 : rpc2;
            const VectorXd &c = o == 0 ? rpc1.line_num_coeff : rpc2.line_num_coeff;
            const VectorXd &d = o == 0 ? rpc1.line_den_coeff : rpc2.line_den_coeff;
            const VectorXd &e = o == 0 ? rpc1.samp_num_coeff : rpc2.samp_num_coeff;
            const VectorXd &f = o == 0 ? rpc1.samp_den_coeff : rpc2.samp_den_coeff;
            double X = o == 0 ? X1 : X2;
            double Y = o == 0 ? Y1 : Y2;
            double H = (z - rpc.height_off) / rpc.height_scale;
            double dHdZ = 1. / rpc.height_scale;

            double u_data[20] = {1,         X,         Y,         H,         X * Y,     X * H,     Y * H,
                                 X * X,     Y * Y,     H * H,     Y * X * H, X * X * X, X * Y * Y, X * H * H,
                                 X * X * Y, Y * Y * Y, Y * H * H, X * X * H, Y * Y * H, H * H * H};
            double dudh_data[20] = {0,     0, 0, 1,         0, X, Y,         0,     0,     2 * H,
                                    Y * X, 0, 0, 2 * X * H, 0, 0, 2 * Y * H, X * X, Y * Y, 3 * H * H};
            VectorXd u = Map<VectorXd>(u_data, 20, 1);
            VectorXd dudh = Map<VectorXd>(dudh_data, 20, 1);

            double &lat = o == 0 ? lat1 : lat2;
            double &lon = o == 0 ? lon1 : lon2;
            double &dlat = o == 0 ? dlat1 : dlat2;
            double &dlon = o == 0 ? dlon1 : dlon2;

            double c_u = c.dot(u);
            double d_u = d.dot(u);
            double e_u = e.dot(u);
            double f_u = f.dot(u);

            double P = c_u / d_u;
            double L = e_u / f_u;

            lat = P * rpc.lat_scale + rpc.lat_off;
            lon = L * rpc.long_scale + rpc.long_off;

            double dlatdP = rpc.lat_scale;
            double dlondL = rpc.long_scale;

            // P = c.dot(u) / d.dot(u)
            double dPdH = (c.dot(dudh) * d_u - d.dot(dudh) * c_u) / (d_u * d_u);
            double dLdH = (e.dot(dudh) * f_u - f.dot(dudh) * e_u) / (f_u * f_u);

            dlat = dlatdP * dPdH * dHdZ;
            dlon = dlondL * dLdH * dHdZ;
        }

        if (stop)
            break;

        double dZ = ((lat1 - lat2) * (dlat2 - dlat1) + (lon1 - lon2) * (dlon2 - dlon1)) /
                    ((dlat2 - dlat1) * (dlat2 - dlat1) + (dlon2 - dlon1) * (dlon2 - dlon1));

        z += dZ;
        if (dZ < 1e-10) {
            stop = true; // 不返回，还需用这个z 去重新计算 latlon
        }
    }

    // 计算 lat, lon
    Vector3d latlon;
    latlon(0) = (lat1 + lat2) / 2.0;
    latlon(1) = (lon1 + lon2) / 2.0;
    if (!stop) {
        latlon(2) = DBL_MAX;
    } else {
        latlon(2) = z;
    }

    return latlon;
}

Vector3d intersect_rpc_ceres(const Vector2d &fwd_point, const Vector2d &bwd_point, const RpcCamera &fwd_camera,
                             const RpcCamera &bwd_camera) {
    ceres::Problem problem;

    double latlon[3];
    latlon[0] = fwd_camera.rpc.lat_off;
    latlon[1] = fwd_camera.rpc.long_off;
    latlon[2] = fwd_camera.rpc.height_off;

    ceres::CostFunction *lcost =
        new ceres::AutoDiffCostFunction<RpcProjection, 2, 3>(new RpcProjection(fwd_point, fwd_camera));
    ceres::CostFunction *rcost =
        new ceres::AutoDiffCostFunction<RpcProjection, 2, 3>(new RpcProjection(bwd_point, bwd_camera));

    problem.AddResidualBlock(lcost, NULL, latlon);
    problem.AddResidualBlock(rcost, NULL, latlon);

    ceres::Solver solver;
    ceres::Solver::Options options;
    options.max_num_iterations = 5;
    options.num_threads = 1;
    options.num_linear_solver_threads = 1;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;

    solver.Solve(options, &problem, &summary);
    return Vector3d(latlon[0], latlon[1], latlon[2]);
}

Vector3d intersect(const Vector2d &fwd_point, const Vector2d &bwd_point, const PolynomialCamera &fwd_camera,
                   const PolynomialCamera &bwd_camera) {

    MatrixXd P1, P2;
    Vector2d mm1, mm2;
    mm1 = fwd_camera.to_mm(fwd_point);
    mm2 = bwd_camera.to_mm(bwd_point);

    P1 = fwd_camera.get_P_by_mm(mm1.y());
    mm1.y() = 0.0;
    P2 = bwd_camera.get_P_by_mm(mm2.y());
    mm2.y() = 0.0;

    mm1 = fwd_point;
    mm1.y() = 0.0;
    mm2 = bwd_point;
    mm2.y() = 0.0;
    return intersect(mm1, mm2, P1, P2);
}

Vector3d intersect(const Vector2d &fwd_point, const Vector2d &bwd_point, const MatrixXd &fwd_camera,
                   const MatrixXd &bwd_camera) {

    Eigen::Matrix4d A;

    A.row(0) = fwd_point(0) * fwd_camera.row(2) - fwd_camera.row(0);
    A.row(1) = fwd_point(1) * fwd_camera.row(2) - fwd_camera.row(1);
    A.row(2) = bwd_point(0) * bwd_camera.row(2) - bwd_camera.row(0);
    A.row(3) = bwd_point(1) * bwd_camera.row(2) - bwd_camera.row(1);

    Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
    return svd.matrixV().col(3).hnormalized();
}

std::vector<Vector3d> intersect(const std::vector<Vector2d> &fwd_points, const std::vector<Vector2d> &bwd_points,
                                const RpcCamera &fwd_camera, const RpcCamera &bwd_camera) {
    CHECK(fwd_points.size() == bwd_points.size());
    std::vector<Vector3d> latlon(fwd_points.size());
#pragma omp parallel for
    for (int i = 0; i < latlon.size(); ++i) {
        latlon[i] = intersect(fwd_points[i], bwd_points[i], fwd_camera, bwd_camera);
    }
    return latlon;
}
std::vector<Vector3d> intersect(const std::vector<Vector2d> &fwd_points, const std::vector<Vector2d> &bwd_points,
                                const PolynomialCamera &fwd_camera, const PolynomialCamera &bwd_camera) {

    CHECK(fwd_points.size() == bwd_points.size());
    std::vector<Vector3d> points(fwd_points.size());
#pragma omp parallel for
    for (int i = 0; i < points.size(); ++i) {
        points[i] = intersect(fwd_points[i], bwd_points[i], fwd_camera, bwd_camera);
    }
    return points;
}

std::vector<Vector3d> intersect(const std::vector<Vector2d> &fwd_points, const std::vector<Vector2d> &bwd_points,
                                const MatrixXd &fwd_camera, const MatrixXd &bwd_camera) {
    CHECK(fwd_points.size() == bwd_points.size());
    std::vector<Vector3d> points(fwd_points.size());
#pragma omp parallel for
    for (int i = 0; i < points.size(); ++i) {
        points[i] = intersect(fwd_points[i], bwd_points[i], fwd_camera, bwd_camera);
    }
    return points;
}
} // namespace h2o
