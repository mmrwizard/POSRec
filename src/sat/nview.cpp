/*!
 * \file nview.cpp
 *
 * \author Han
 * \date 2017/05/04
 *
 * 多视几何处理
 */
#include "nview.h"
#include "camera_rpc_ceres.h"
#include "oneview.h"

#include <Eigen/Eigenvalues>
#include <Eigen/QR>
namespace h2o {
Vector3d intersect(const std::vector<Vector2d> &points, const std::vector<RpcCamera> &cameras) {
    ceres::Problem problem;

    CHECK(cameras.size() > 0);
    CHECK(points.size() == cameras.size());

    double latlon[3];
    latlon[0] = cameras[0].rpc.lat_off;
    latlon[1] = cameras[0].rpc.long_off;
    latlon[2] = cameras[0].rpc.height_off;

    for (int i = 0; i < cameras.size(); ++i) {
        ceres::CostFunction *cost =
            new ceres::AutoDiffCostFunction<RpcProjection, 2, 3>(new RpcProjection(points[i], cameras[i]));
        problem.AddResidualBlock(cost, NULL, latlon);
    }

    ceres::Solver solver;
    ceres::Solver::Options options;
    options.max_num_iterations = 5;
    options.num_threads = 1;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;

    solver.Solve(options, &problem, &summary);

    return Vector3d(latlon[0], latlon[1], latlon[2]);
}

std::vector<Vector3d> intersect(const std::vector<std::unordered_map<iid_t, Vector2d>> &points,
                                const RpcCameras &cameras) {
    // nview 的前方交汇 采用 reverse rpc 还是有问题，暂时用 ceres 的
    CHECK(false);

    int num_points = points.size();
    std::vector<Vector3d> results(num_points);
    if (num_points == 0) {
        return results;
    }

    double initial = 0.0; // 平均高程
    for (const auto &camera : cameras) {
        initial += camera.second.rpc.height_off;
    }
    initial /= cameras.size();

#pragma omp parallel for
    for (int i = 0; i < num_points; i++) {
        int nviews = points[i].size();
        double z = initial;
        VectorXd A = VectorXd::Zero(2 * nviews - 2);
        VectorXd b = VectorXd::Zero(2 * nviews - 2);

        std::unordered_map<iid_t, Vector2d> latlon;
        std::unordered_map<iid_t, Vector2d> dlatlon;

        int niter = 5;
        for (int n = 0; n < niter; ++n) {
            Vector2d latlon_first;
            Vector2d dlatlon_first;
            bool first = true;
            int v = 0;
            for (const auto &pair : points[i]) {
                iid_t iid = pair.first;
                const auto &camera = cameras.at(iid);
                const auto &rpc = camera.rpci;
                // 纠正畸变
                Vector2d pixel = camera.rpc_aff_inv * pair.second.homogeneous();

                double &lat = latlon[iid](0);
                double &lon = latlon[iid](1);
                double &dlat = dlatlon[iid](0);
                double &dlon = dlatlon[iid](1);

                double X = (pixel(0) - rpc.samp_off) / rpc.samp_scale;
                double Y = (pixel(1) - rpc.line_off) / rpc.line_scale;
                double H = (z - rpc.height_off) / rpc.height_scale;
                double dHdZ = 1. / rpc.height_scale;

                double u_data[20] = {1,         X,         Y,         H,         X * Y,     X * H,     Y * H,
                                     X * X,     Y * Y,     H * H,     Y * X * H, X * X * X, X * Y * Y, X * H * H,
                                     X * X * Y, Y * Y * Y, Y * H * H, X * X * H, Y * Y * H, H * H * H};
                double dudh_data[20] = {0,     0, 0, 1,         0, X, Y,         0,     0,     2 * H,
                                        Y * X, 0, 0, 2 * X * H, 0, 0, 2 * Y * H, X * X, Y * Y, 3 * H * H};
                VectorXd u = Map<VectorXd>(u_data, 20, 1);
                VectorXd dudh = Map<VectorXd>(dudh_data, 20, 1);
                const VectorXd &c = rpc.line_num_coeff;
                const VectorXd &d = rpc.line_den_coeff;
                const VectorXd &e = rpc.samp_num_coeff;
                const VectorXd &f = rpc.samp_den_coeff;
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

                if (first) {
                    first = false;
                    latlon_first = Vector2d(lat, lon);
                    dlatlon_first = Vector2d(dlat, dlon);
                    continue;
                }

                A(2 * v) = dlat - dlatlon_first(0);
                A(2 * v + 1) = dlon - dlatlon_first(1);
                b(2 * v) = latlon_first(0) - lat;
                b(2 * v + 1) = latlon_first(1) - lon;
                v++;
            }
            double dz = A.householderQr().solve(b)(0);
            z += dz;
            if (dz < 1e-10) {
                break;
            }
        }

        Vector3d point3d = Vector3d::Zero();
        for (const auto &point2d : latlon) {
            point3d.head(2) += point2d.second;
        }
        point3d = point3d / latlon.size();
        point3d(2) = z;

        results[i] = point3d;
    }

    return results;
}

std::vector<Vector3d> intersect_ceres(const std::vector<std::unordered_map<iid_t, Vector2d>> &points,
                                      const RpcCameras &cameras) {
    int num_points = points.size();
    CHECK(num_points > 0);

    Vector3d initial(cameras.begin()->second.rpc.lat_off, cameras.begin()->second.rpc.long_off,
                     cameras.begin()->second.rpc.height_off);

    std::vector<Vector3d> latlon(points.size());
#pragma omp parallel for
    for (int i = 0; i < num_points; ++i) {
        ceres::Problem problem;
        double data[3];
        data[0] = initial(0);
        data[1] = initial(1);
        data[2] = initial(2);

        for (const auto &pair : points[i]) {
            iid_t iid = pair.first;
            Vector2d point = pair.second;

            ceres::CostFunction *cost =
                new ceres::AutoDiffCostFunction<RpcProjection, 2, 3>(new RpcProjection(point, cameras.at(iid)));
            problem.AddResidualBlock(cost, NULL, data);
        }
        ceres::Solver solver;
        ceres::Solver::Options options;
        options.max_num_iterations = 5;
        options.num_threads = 1;
        options.linear_solver_type = ceres::DENSE_QR;
        ceres::Solver::Summary summary;

        solver.Solve(options, &problem, &summary);
        latlon[i] = Vector3d(data[0], data[1], data[2]);
    }

    return latlon;
}

Vector3d intersect(const std::vector<Vector2d> &points, const std::vector<PolynomialCamera> &cameras) {
    CHECK_EQ(points.size(), cameras.size());
    auto points_zero_y = points;

    for (auto &p : points_zero_y) {
        p.y() = 0.0;
    }
    std::vector<MatrixXd> P(cameras.size());
    for (int i = 0; i < cameras.size(); ++i) {
        Vector2d mm = cameras[i].to_mm(points[i]);
        P[i] = cameras[i].get_P_by_mm(mm.y());
    }

    return intersect(points_zero_y, P);
}

std::vector<Vector3d> intersect(const std::vector<std::unordered_map<iid_t, Vector2d>> &pixels,
                                const PolynomialCameras &cameras) {

    std::vector<Vector3d> points(pixels.size());
    // 获取 所有像素，把 y 设置为0，然后获取对应的 P 矩阵，采用 P 矩阵进行前方交会
    for (int i = 0; i < points.size(); ++i) {
        const auto &observations = pixels[i];
        int num_views = observations.size();
        std::vector<Vector2d> pixels_zero_y(num_views);
        std::vector<MatrixXd> P(num_views);

        int view_id = 0;
        for (const auto &pair : observations) {
            const PolynomialCamera &camera = cameras.at(pair.first);
            Vector2d mm = camera.to_mm(pair.second);
            P[view_id] = camera.get_P_by_mm(mm.y());
            pixels_zero_y[view_id] = {pair.second.x(), 0.0};
            ++view_id;
        }

        points[i] = intersect(pixels_zero_y, P);
    }
    return points;
}

Vector3d intersect(const std::vector<Vector2d> &points, const std::vector<MatrixXd> &cameras) {
    CHECK_EQ(points.size(), cameras.size());

    int num_points = points.size();
    MatrixXd A(num_points * 2, 3);
    VectorXd b(num_points * 2);

    for (int i = 0; i < num_points; ++i) {
        const MatrixXd &P = cameras[i];
        double x = points[i].x();
        double y = points[i].y();
        A(2 * i, 0) = P(0, 0) - P(2, 0) * x;
        A(2 * i, 1) = P(0, 1) - P(2, 1) * x;
        A(2 * i, 2) = P(0, 2) - P(2, 2) * x;
        b(2 * i) = P(2, 3) * x - P(0, 3);
        A(2 * i + 1, 0) = P(1, 0) - P(2, 0) * y;
        A(2 * i + 1, 1) = P(1, 1) - P(2, 1) * y;
        A(2 * i + 1, 2) = P(1, 2) - P(2, 2) * y;
        b(2 * i + 1) = P(2, 3) * y - P(1, 3);
    }

    // 采用 QR solver 比用 eigen value的要准一些
    Vector3d X = A.householderQr().solve(b);
    return X;
}

std::vector<Vector3d> intersect(const std::vector<std::unordered_map<iid_t, Vector2d>> &pixels,
                                const std::unordered_map<iid_t, MatrixXd> &cameras) {
    std::vector<Vector3d> points(pixels.size());
    for (int i = 0; i < pixels.size(); ++i) {
        const auto &observations = pixels[i];
        Matrix4d A = Matrix4d::Zero();

        for (const auto &observation : observations) {
            iid_t iid = observation.first;
            const Vector3d point = observation.second.homogeneous().normalized();
            const Eigen::MatrixXd term = cameras.at(iid) - point * point.transpose() * cameras.at(iid);
            A += term.transpose() * term;
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen_solver(A);
        points[i] = eigen_solver.eigenvectors().col(0).hnormalized();
    }
    return points;
}
} // namespace h2o
