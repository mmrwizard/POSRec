/*!
 * \file oneview.cpp
 *
 * \author Han
 * \date 2017/05/04
 *
 * 但视几何处理
 */
#include "oneview.h"
#include "camera_rpc_ceres.h"
#include "fitpoly.h"
#include "geographic.h"

#include <base/base.h>
#include <fstream>

#include <Eigen/QR>
#include <GeographicLib/Constants.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <ceres/rotation.h>
namespace h2o {

Vector2d project(const Vector3d &latlon, const RpcCamera &camera) {
    const auto &rpc = camera.rpc;

    const VectorXd &c = rpc.line_num_coeff;
    const VectorXd &d = rpc.line_den_coeff;
    const VectorXd &e = rpc.samp_num_coeff;
    const VectorXd &f = rpc.samp_den_coeff;

    double P, L, H;
    P = (latlon(0) - rpc.lat_off) / rpc.lat_scale;
    L = (latlon(1) - rpc.long_off) / rpc.long_scale;
    H = (latlon(2) - rpc.height_off) / rpc.height_scale;

    double u_data[20] = {1,         L,         P,         H,         L * P,     L * H,     P * H,
                         L * L,     P * P,     H * H,     P * L * H, L * L * L, L * P * P, L * H * H,
                         L * L * P, P * P * P, P * H * H, L * L * H, P * P * H, H * H * H};
    VectorXd u = Map<VectorXd>(u_data, 20, 1);

    double Y = c.dot(u) / d.dot(u);
    double X = e.dot(u) / f.dot(u);

    double line = Y * rpc.line_scale + rpc.line_off;
    double sample = X * rpc.samp_scale + rpc.samp_off;

    auto affine = camera.rpc_aff;
    Vector2d pixel(sample, line);
    pixel = affine.block(0, 0, 2, 2) * pixel + affine.col(2);

    return pixel;
}

Vector2d project(const Vector3d &point, const PolynomialCamera &camera) {
    CHECK(camera.width != 0 && camera.height != 0);

    double step = 5000.0;
    double pixel_size = camera.pixel;
    double threshold = 0.1 * pixel_size;

    VectorXd start_y(int(camera.height / step) * 2);
    start_y << VectorXd::LinSpaced(camera.height / step, 0.0, camera.height / 2.0 * pixel_size),
        VectorXd::LinSpaced(camera.height / step, -camera.height / 2.0 * pixel_size, 0.0);

    bool converge = false;
    double min_error = DBL_MAX;
    Vector2d point2d;
    for (int i = 0; i < start_y.rows(); ++i) {
        double delta = 1.0;
        int iter = 0;
        point2d.x() = 0.0;
        point2d.y() = start_y(i);
        Vector3d X = point;
        while (std::abs(delta) > threshold) {
            Vector3d C;
            Matrix3d R;
            std::tie(C, R) = camera.get_eo_by_mm(point2d.y());
            Vector3d Xc = R.transpose() * (X - C);
            point2d.x() = -camera.f * Xc(0) / Xc(2);
            delta = -camera.f * Xc(1) / Xc(2);

            //             if (std::abs(delta) > pixel_size * step)
            //                 delta = delta > 0 ? pixel_size * step : pixel_size * -step;
            point2d.y() += delta;
            if (std::abs(delta) < min_error)
                min_error = std::abs(delta);
            if (iter > 20) {
                break;
            }
            iter++;
        }

        if (std::abs(delta) <= threshold) {
            // converged
            converge = true;
            break;
        }
    }

    if (!converge) {
        throw std::runtime_error("projection failed");
    }
    /*CHECK(converge == true) << "do not converge in projection with error :" << min_error / pixel_size << " pixels";*/
    return camera.to_pixel(point2d);
}

std::vector<Vector2d> project(const std::vector<Vector3d> &latlon, const RpcCamera &camera) {
    std::vector<Vector2d> points(latlon.size());

#pragma omp parallel for
    for (int i = 0; i < latlon.size(); ++i) {
        points[i] = project(latlon[i], camera);
    }
    return points;
}

std::vector<Vector2d> project(const std::vector<Vector3d> &points, const PolynomialCamera &camera) {
    std::vector<Vector2d> pixels(points.size());
    // 循环里面要 throw 所以不能用omp
    //#pragma omp parallel for
    for (int i = 0; i < points.size(); ++i) {
        pixels[i] = project(points[i], camera);
    }
    return pixels;
}

Vector2d project(const Vector3d &point, const MatrixXd &camera) {
    return (camera * point.homogeneous()).eval().hnormalized();
}

Vector3d project_to_geographic_z(const RpcCamera &camera, double z, const Vector2d &pixel) {
    const auto &rpc = camera.rpci;

    const VectorXd &c = rpc.line_num_coeff;
    const VectorXd &d = rpc.line_den_coeff;
    const VectorXd &e = rpc.samp_num_coeff;
    const VectorXd &f = rpc.samp_den_coeff;

    const auto &affine = camera.rpc_aff_inv;
    Vector2d pixel_undistort = affine * pixel.homogeneous();

    double X, Y, H;
    X = (pixel_undistort(0) - rpc.samp_off) / rpc.samp_scale;
    Y = (pixel_undistort(1) - rpc.line_off) / rpc.line_scale;
    H = (z - rpc.height_off) / rpc.height_scale;

    double u_data[20] = {1,         X,         Y,         H,         X * Y,     X * H,     Y * H,
                         X * X,     Y * Y,     H * H,     Y * X * H, X * X * X, X * Y * Y, X * H * H,
                         X * X * Y, Y * Y * Y, Y * H * H, X * X * H, Y * Y * H, H * H * H};
    VectorXd u = Map<VectorXd>(u_data, 20, 1);

    double P = c.dot(u) / d.dot(u);
    double L = e.dot(u) / f.dot(u);

    double lat = P * rpc.lat_scale + rpc.lat_off;
    double lon = L * rpc.long_scale + rpc.long_off;

    return Vector3d(lat, lon, z);
}

Vector3d project_to_geographic_z_ceres(const RpcCamera &camera, double z, const Vector2d &pixel) {
    ceres::Problem problem;
    double latlon[2] = {camera.rpc.lat_off, camera.rpc.long_off};

    ceres::CostFunction *cost =
        new ceres::AutoDiffCostFunction<RpcProjectionXY, 2, 2>(new RpcProjectionXY(pixel(0), pixel(1), z, camera));
    problem.AddResidualBlock(cost, NULL, latlon);

    ceres::Solver solver;
    ceres::Solver::Options options;
    options.max_num_iterations = 20;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;

    solver.Solve(options, &problem, &summary);

    return Vector3d(latlon[0], latlon[1], z);
}

BoundingBox3d geographic_region(const PolynomialCamera &camera, const GeoImage &dem) {
    Vector2d corner[4] = {{0.0, 0.0},
                          {(double)camera.width - 1.0, 0.0},
                          {double(camera.width - 1.0), double(camera.height - 1.0)},
                          {0.0, double(camera.height - 1.0)}};

    // [minx, maxx, miny, maxy]
    BoundingBox2d box_gcs;
    BoundingBox2i box_image;
    // [minlat, maxlat, minlon, maxlon]

    // 查找最大最小高程值
    for (int i = 0; i < 4; ++i) {
        Vector3d latlon = project_to_geographic_dem(camera, dem, corner[i], camera.planet);
        Vector2i cell = dem.get_cell_bounded(latlon(1), latlon(0));

        box_image.extend(cell);
        box_gcs.extend(latlon.head(2));
    }

    cv::Mat mat = dem.get_image();
    if (box_image.volume() == 0.0) {
        LOG(WARNING) << "dem and image have not intersections";
        return BoundingBox3d();
    }

    cv::Mat sub = mat({box_image.min().y(), box_image.max().y()}, {box_image.min().x(), box_image.max().x()});
    cv::Scalar mean, std;
    cv::meanStdDev(sub, mean, std);
    Vector2d min_max(mean(0) - 5 * std(0), mean(0) + 5 * std(0));

    BoundingBox3d bb;
    bb.min() = {box_gcs.min().x(), box_gcs.min().y(), min_max(0)};
    bb.max() = {box_gcs.max().x(), box_gcs.max().y(), min_max(1)};
    return bb;
}

BoundingBox2d geographic_region(const PolynomialCamera &camera, double z) {
    Vector2d corner[4] = {{0.0, 0.0},
                          {(double)camera.width - 1.0, 0.0},
                          {double(camera.width - 1.0), double(camera.height - 1.0)},
                          {0.0, double(camera.height - 1.0)}};
    BoundingBox2d bb;
    // 查找最大最小高程值
    for (int i = 0; i < 4; ++i) {
        Vector3d latlon = project_to_geographic_z(camera, z, corner[i], camera.planet);
        bb.extend(latlon.head(2));
    }
    return bb;
}

BoundingBox3d geographic_region(const RpcCamera &camera) {
    BoundingBox3d bb;

    double width = camera.rpc.samp_off * 2;
    double height = camera.rpc.line_off * 2;

    std::vector<Vector2d> corners = {Vector2d(0, 0), Vector2d(width, 0), Vector2d(width, height), Vector2d(0, height)};
    Vector2d minmax_z = {camera.rpc.height_off - camera.rpc.height_scale,
                         camera.rpc.height_off + camera.rpc.height_scale};
    for (int i = 0; i < corners.size(); ++i) {
        Vector3d point = project_to_geographic_z(camera, minmax_z(0), corners[i]);
        bb.extend(point);
        point = project_to_geographic_z(camera, minmax_z(1), corners[i]);
        bb.extend(point);
    }

    return bb;
}

Vector3d rodrigues(const Matrix3d &R) {
    Vector3d r;
    ceres::RotationMatrixToAngleAxis(R.data(), r.data());
    return r;
}

Matrix3d rodrigues(const Vector3d &r) {
    Matrix3d R;
    ceres::AngleAxisToRotationMatrix(r.data(), R.data());
    return R;
}

Vector3d project_to_geographic_dem(const PolynomialCamera &camera, const GeoImage &dem, const Vector2d &pixel,
                                   const std::string &planet) {
    Vector2d mm = camera.to_mm(pixel);
    MatrixXd P = camera.get_P_by_mm(mm.y());

    Vector3d C;
    Matrix3d R;
    std::tie(C, R) = camera.get_eo_by_mm(mm.y());

    Vector3d latlon = geocentric_to_latlon(C, planet);
    double z = dem.get_point_t(Vector2d(latlon(1), latlon(0)));

    Vector3d geoc = latlon_to_geocentric({latlon(0), latlon(1), z}, planet);

    double r2 = geoc.squaredNorm();

    int iter = 0;
    double delta = 100.0;
    double last = z;
    while (std::abs(delta) > 1.0 && iter < 20) {
        Vector4d plane(geoc(0), geoc(1), geoc(2), -r2);
        Matrix3d H;
        int idx;
        std::tie(H, idx) = geocentric_plane_homography(plane, P);
        H = H.inverse().eval();

        Vector3d m(pixel(0), 0.0, 1.0);
        Vector3d X = H * m;

        if (idx == 0) {
            // [Y Z 1] = Hm
            geoc[1] = X[0] / X[2];
            geoc[2] = X[1] / X[2];
            geoc[0] = -(plane[1] * geoc[1] + plane[2] * geoc[2] + plane[3]) / plane[0];
        } else if (idx == 1) {
            // [X Z 1] = Hm
            geoc[0] = X[0] / X[2];
            geoc[2] = X[1] / X[2];
            geoc[1] = -(plane[0] * geoc[0] + plane[2] * geoc[2] + plane[3]) / plane[1];
        } else if (idx == 2) {
            // [X Y 1] = Hm
            geoc[0] = X[0] / X[2];
            geoc[1] = X[1] / X[2];
            geoc[2] = -(plane[0] * geoc[0] + plane[1] * geoc[1] + plane[3]) / plane[2];
        }

        latlon = geocentric_to_latlon(geoc, planet);
        latlon(2) = dem.get_point_t(Vector2d(latlon(1), latlon(0)));

        delta = latlon(2) - last;
        last = latlon(2);

        geoc = latlon_to_geocentric(latlon, planet);
        r2 = geoc.squaredNorm();
        iter++;
    }

    if (iter >= 20) {
        LOG(WARNING) << "project to dem with " << iter << " iterations and dem error " << delta;
    }

    return latlon;
}

std::tuple<Matrix3d, int> geocentric_plane_homography(const Vector4d &plane, const MatrixXd &P) {
    double a, b, c, d;
    a = plane(0);
    b = plane(1);
    c = plane(2);
    d = plane(3);

    int idx;
    plane.head(3).cwiseAbs().maxCoeff(&idx);

    Vector3d p1, p2, p3, p4;
    p1 = P.col(0);
    p2 = P.col(1);
    p3 = P.col(2);
    p4 = P.col(3);

    Matrix3d H;
    if (idx == 2) {
        H << p1 - a / c * p3, p2 - b / c * p3, -d / c * p3 + p4;
    } else if (idx == 1) {
        H << p1 - a / b * p2, p3 - c / b * p2, -d / b * p2 + p4;
    } else {
        H << p2 - b / a * p1, p3 - c / a * p1, -d / a * p1 + p4;
    }

    return std::make_tuple(H, idx);
}

Vector3d project_to_geographic_z(const PolynomialCamera &camera, double z, const Vector2d &pixel,
                                 const std::string &planet) {
    using namespace GeographicLib;
    double a, f;
    if (planet == "moon") {
        a = MARS_RADIUS;
        f = 0.0;
    } else if (planet == "mars") {
        a = MARS_RADIUS;
        f = 0.0;
    } else if (planet == "earth") {
        a = Constants::WGS84_a();
        f = Constants::WGS84_f();
    }
    return project_to_geographic_z(camera, z, pixel, a, f);
}

Vector3d project_to_geographic_z(const PolynomialCamera &camera, double z, const Vector2d &pixel, double a, double f) {
    Vector2d mm = camera.to_mm(pixel);
    MatrixXd P = camera.get_P_by_mm(mm.y());

    Vector3d C;
    Matrix3d R;
    std::tie(C, R) = camera.get_eo_by_mm(mm.y());

    Vector3d latlon = geocentric_to_latlon(C, a, f);
    Vector3d geoc = latlon_to_geocentric({latlon(0), latlon(1), z}, a, f);

    double r2 = geoc.squaredNorm();

    int iter = 0;
    double delta = 100.0;
    Vector3d last = geoc;
    while (std::abs(delta) > 1.0 && iter < 20) {
        Vector4d plane(geoc(0), geoc(1), geoc(2), -r2);
        Matrix3d H;
        int idx;
        std::tie(H, idx) = geocentric_plane_homography(plane, P);
        H = H.inverse().eval();

        Vector3d m(pixel(0), 0.0, 1.0);
        Vector3d X = H * m;

        if (idx == 0) {
            // [Y Z 1] = Hm
            geoc[1] = X[0] / X[2];
            geoc[2] = X[1] / X[2];
            geoc[0] = -(plane[1] * geoc[1] + plane[2] * geoc[2] + plane[3]) / plane[0];
        } else if (idx == 1) {
            // [X Z 1] = Hm
            geoc[0] = X[0] / X[2];
            geoc[2] = X[1] / X[2];
            geoc[1] = -(plane[0] * geoc[0] + plane[2] * geoc[2] + plane[3]) / plane[1];
        } else if (idx == 2) {
            // [X Y 1] = Hm
            geoc[0] = X[0] / X[2];
            geoc[1] = X[1] / X[2];
            geoc[2] = -(plane[0] * geoc[0] + plane[1] * geoc[1] + plane[3]) / plane[2];
        }

        latlon = geocentric_to_latlon(geoc, a, f);
        latlon(2) = z;
        geoc = latlon_to_geocentric(latlon, a, f);

        delta = (geoc - last).norm();
        last = geoc;

        r2 = geoc.squaredNorm();
        iter++;
    }

    return latlon;
}
} // namespace h2o
