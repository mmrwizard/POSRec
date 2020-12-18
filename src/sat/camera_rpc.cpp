/*!
 * \file camera_rpc.cpp
 *
 * \author Han
 * \date 2017/05/04
 *
 * RPC 相机模型
 */

#include <base/base.h>

#include "camera_polynomial.h"
#include "camera_rpc.h"
#include "geographic.h"
#include "oneview.h"
#include "serialization.h"

#include <Eigen/LU>
#include <GeographicLib/LambertConformalConic.hpp>
#include <boost/algorithm/cxx11/all_of.hpp>
#include <boost/algorithm/string.hpp>
#include <fstream>

namespace h2o {
RpcCamera::RpcCamera() {
    width = -1;
    height = -1;
    gsd = 0.0;

    rpc_aff << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    init_inverse_transform();
}

void RpcCamera::init_inverse_transform() {
    // p' = Ap +b
    // p = A-1(p'-b)
    Matrix2d A = rpc_aff.block(0, 0, 2, 2);
    Vector2d b = rpc_aff.col(2);

    rpc_aff_inv.block(0, 0, 2, 2) = A.inverse();
    rpc_aff_inv.col(2) = -A.inverse() * b;
}

double RpcCamera::compute_gsd() {
    double a, f;
    if (string_to_lower(planet) == "mars") {
        a = MARS_RADIUS;
        f = 0.0;
    } else if (string_to_lower(planet) == "moon") {
        a = MOON_RADIUS;
        f = 0.0;
    } else if (string_to_lower(planet) == "earth") {
        a = GeographicLib::Constants::WGS84_a();
        f = GeographicLib::Constants::WGS84_f();
    }

    double lat0 = rpc.lat_off;
    double lon0 = rpc.long_off;
    GeographicLib::LambertConformalConic lcc(a, f, lat0, 1.0);

    std::vector<Vector2d> corners = {Vector2d(0, 0), Vector2d(width, 0), Vector2d(width, height), Vector2d(0, height)};
    double _gsd = 0.0;
    for (int i = 0; i < 3; ++i) {
        Vector2d point1 = corners[i];
        Vector2d point2 = corners[i + 1];
        double pixel_length = (point1 - point2).norm();

        point1 = pixel_to_latlon(point1);
        point2 = pixel_to_latlon(point2);

        lcc.Forward(lon0, point1(0), point1(1), point1(0), point1(1));
        lcc.Forward(lon0, point2(0), point2(1), point2(0), point2(1));

        double project_length = (point1 - point2).norm();

        _gsd += project_length / pixel_length;
    }
    _gsd /= 3.0;
    gsd = _gsd;
    return gsd;
}

Vector2d RpcCamera::pixel_to_latlon(const Vector2d &pixel) const {
    double z = rpc.height_off;
    return project_to_geographic_z(*this, z, pixel).head(2);
}

Vector2d RpcCamera::latlon_to_pixel(const Vector2d &latlon) const {
    double z = rpc.height_off;
    Vector3d latlon3d = {latlon(0), latlon(1), z};
    return project(latlon3d, *this);
}

bool predicate(char c) {
    // 加上\r是因为在linux环境下getline得到的字符串会在末尾存在\r，这是因为windows下\n实际上是\r\n
    return isdigit(c) || c == '+' || c == '-' || c == 'e' || c == 'E' || c == '.' || c == '\r';
}

void get_coeff(const std::string &line, double &value) {
    std::vector<std::string> spilts;
    boost::algorithm::split(spilts, line, boost::is_any_of(" "));
    for (int i = 0; i < spilts.size(); ++i) {
        if (!spilts[i].empty()) {
            boost::algorithm::trim_if(spilts[i], boost::algorithm::is_any_of("\r "));
            if (boost::algorithm::all_of(spilts[i], predicate)) {
                value = std::stod(spilts[i]);
                break;
            }
        }
    }
}

RpcCoefficient load_rpc(const std::string &rpcpath) {
    RpcCoefficient rpc;
    rpc.line_num_coeff.resize(20);
    rpc.line_den_coeff.resize(20);
    rpc.samp_num_coeff.resize(20);
    rpc.samp_den_coeff.resize(20);

    std::ifstream ifile(string_utf8_to_fstream(rpcpath));
    std::string line;

    std::getline(ifile, line);
    get_coeff(line, rpc.line_off);
    std::getline(ifile, line);
    get_coeff(line, rpc.samp_off);
    std::getline(ifile, line);
    get_coeff(line, rpc.lat_off);
    std::getline(ifile, line);
    get_coeff(line, rpc.long_off);
    std::getline(ifile, line);
    get_coeff(line, rpc.height_off);
    std::getline(ifile, line);
    get_coeff(line, rpc.line_scale);
    std::getline(ifile, line);
    get_coeff(line, rpc.samp_scale);
    std::getline(ifile, line);
    get_coeff(line, rpc.lat_scale);
    std::getline(ifile, line);
    get_coeff(line, rpc.long_scale);
    std::getline(ifile, line);
    get_coeff(line, rpc.height_scale);

    for (int i = 0; i < 20; ++i) {
        std::getline(ifile, line);
        get_coeff(line, rpc.line_num_coeff[i]);
    }
    for (int i = 0; i < 20; ++i) {
        std::getline(ifile, line);
        get_coeff(line, rpc.line_den_coeff[i]);
    }
    for (int i = 0; i < 20; ++i) {
        std::getline(ifile, line);
        get_coeff(line, rpc.samp_num_coeff[i]);
    }
    for (int i = 0; i < 20; ++i) {
        std::getline(ifile, line);
        get_coeff(line, rpc.samp_den_coeff[i]);
    }
    return rpc;
}

void save_rpc(const std::string &rpcpath, const RpcCoefficient &rpc) {
    std::ofstream ofile(string_utf8_to_fstream(rpcpath));

    ofile << string_printf("LINE_OFF: %+.15g pixels\n", rpc.line_off);
    ofile << string_printf("SAMP_OFF: %+.15g pixels\n", rpc.samp_off);
    ofile << string_printf("LAT_OFF: %+.15g degrees\n", rpc.lat_off);
    ofile << string_printf("LONG_OFF: %+.15g degrees\n", rpc.long_off);
    ofile << string_printf("HEIGHT_OFF: %+.15g meters\n", rpc.height_off);
    ofile << string_printf("LINE_SCALE: %+.15g pixels\n", rpc.line_scale);
    ofile << string_printf("SAMP_SCALE: %+.15g pixels\n", rpc.samp_scale);
    ofile << string_printf("LAT_SCALE: %+.15g degrees\n", rpc.lat_scale);
    ofile << string_printf("LONG_SCALE: %+.15g degrees\n", rpc.long_scale);
    ofile << string_printf("HEIGHT_SCALE: %+.15g meters\n", rpc.height_scale);

    for (int i = 0; i < 20; ++i)
        ofile << string_printf("LINE_NUM_COEFF_%d: %+.15g\n", i + 1, rpc.line_num_coeff[i]);
    for (int i = 0; i < 20; ++i)
        ofile << string_printf("LINE_DEN_COEFF_%d: %+.15g\n", i + 1, rpc.line_den_coeff[i]);
    for (int i = 0; i < 20; ++i)
        ofile << string_printf("SAMP_NUM_COEFF_%d: %+.15g\n", i + 1, rpc.samp_num_coeff[i]);
    for (int i = 0; i < 20; ++i)
        ofile << string_printf("SAMP_DEN_COEFF_%d: %+.15g\n", i + 1, rpc.samp_den_coeff[i]);
}

void save_rpc_camera(const std::string &filepath, const RpcCamera &camera) {
    CHECK(get_extension_name(filepath) == ".json");
    // 首先保存 rpc 参数
    std::string name = get_filename_noext(filepath);
    std::string rpcpath = join_paths(get_directory(filepath), name + "_rpc.txt");
    std::string rpcipath = join_paths(get_directory(filepath), name + "_rpci.txt");

    save_rpc(rpcpath, camera.rpc);
    save_rpc(rpcipath, camera.rpci);

    std::ofstream ofile(string_utf8_to_fstream(filepath));
    cereal::JSONOutputArchive archive(ofile);
    archive(camera);
}

RpcCamera load_rpc_camera(const std::string &filepath) {
    CHECK(get_extension_name(filepath) == ".json");

    RpcCamera camera;
    std::ifstream ifile(string_utf8_to_fstream(filepath));
    cereal::JSONInputArchive archive(ifile);
    archive(camera);

    // 读取 rpc 参数信息
    std::string name = get_filename_noext(filepath);
    std::string rpcpath = join_paths(get_directory(filepath), name + "_rpc.txt");
    std::string rpcipath = join_paths(get_directory(filepath), name + "_rpci.txt");

    camera.rpc = load_rpc(rpcpath);
    camera.rpci = load_rpc(rpcipath);

    return camera;
}

void compute_reverse_rpc(RpcCamera &camera) {
    RpcCamera camera_no_aff = camera;
    camera_no_aff.rpc_aff << 1, 0, 0, 0, 1, 0;
    camera_no_aff.init_inverse_transform();

    int row_count = 50, col_count = 50;
    int num_plane = 20;
    int num_points = col_count * row_count * num_plane;

    std::vector<Vector3d> latlon(num_points);
    std::vector<Vector2d> XY(num_points);
    std::vector<Vector3d> PLH(num_points);
    RpcCoefficient rpc = camera.rpc;
    {
        VectorXd lat = VectorXd::LinSpaced(row_count, rpc.lat_off - rpc.lat_scale, rpc.lat_off + rpc.lat_scale);
        VectorXd lon = VectorXd::LinSpaced(col_count, rpc.long_off - rpc.long_scale, rpc.long_off + rpc.long_scale);
        VectorXd hei =
            VectorXd::LinSpaced(num_plane, rpc.height_off - rpc.height_scale, rpc.height_off + rpc.height_scale);

        int i = 0;
        for (int r = 0; r < row_count; ++r) {
            for (int c = 0; c < col_count; ++c) {
                for (int p = 0; p < num_plane; ++p) {
                    latlon[i] = {lat(r), lon(c), hei(p)};
                    ++i;
                }
            }
        }

        // 拟合 reverse，只顾及原始的 rpc 参数，仿射变换系数在应用过程中需要提前纠正
        std::vector<Vector2d> pixels = project(latlon, camera_no_aff);

#pragma omp parallel for
        for (int i = 0; i < num_points; ++i) {
            XY[i].x() = (pixels[i](0) - rpc.samp_off) / rpc.samp_scale;
            XY[i].y() = (pixels[i](1) - rpc.line_off) / rpc.line_scale;
            double P, L, H;
            P = (latlon[i](0) - rpc.lat_off) / rpc.lat_scale;
            L = (latlon[i](1) - rpc.long_off) / rpc.long_scale;
            H = (latlon[i](2) - rpc.height_off) / rpc.height_scale;
            PLH[i] = {P, L, H};
        }
    }

    // [X Y X Y]
    MatrixXd A = MatrixXd::Zero(num_points * 2, 78);
    VectorXd b = VectorXd::Zero(num_points * 2);
    {
#pragma omp parallel for
        for (int i = 0; i < num_points; ++i) {
            double P, L, H;
            P = PLH[i](0);
            L = PLH[i](1);
            H = PLH[i](2);
            double X, Y;
            X = XY[i](0);
            Y = XY[i](1);

            // P = c.dot(u) / d.dot(u) => c.dot(u) - P * d.dot(u) = 0
            // L = e.dot(u) / f.dot(u) => e.dot(u) - L * f.dot(u) = 0
            double u[20] = {1.0,       X,         Y,         H,         X * Y,     X * H,     Y * H,
                            X * X,     Y * Y,     H * H,     Y * X * H, X * X * X, X * Y * Y, X * H * H,
                            X * X * Y, Y * Y * Y, Y * H * H, X * X * H, Y * Y * H, H * H * H};
            for (int j = 0; j < 20; ++j) {
                A(i, j) = u[j]; // c - [0, 20]
                if (j != 0)
                    A(i, 20 + j - 1) = -1.0 * P * u[j]; // d - [20, 39]
                A(num_points + i, 39 + j) = u[j];       // e - [39, 59]
                if (j != 0)
                    A(num_points + i, 59 + j - 1) = -1.0 * L * u[j]; // f - [59. 78]
            }

            b(i) = P;
            b(i + num_points) = L;
        }
    }

    // c, d, e, f
    VectorXd X = A.householderQr().solve(b);
    //     VectorXd res = A * X - b;
    //     double residule = (A * X - b).norm() / (num_points * 2) * (rpc.long_scale + rpc.lat_scale) / 2.0;
    //     std::string message = string_printf("the rpc fit cost for %s is %g degree", camera.filepath.c_str(),
    //     residule);
    //     LOG(INFO) << message;
    RpcCoefficient rpci = rpc;
    rpci.line_den_coeff(0) = 1.0;
    rpci.samp_den_coeff(0) = 1.0;

    for (int i = 0; i < 20; ++i) {
        rpci.line_num_coeff(i) = X(i);
        if (i != 0) {
            rpci.line_den_coeff(i) = X(i + 20 - 1);
        }
        rpci.samp_num_coeff(i) = X(i + 39);
        if (i != 0) {
            rpci.samp_den_coeff(i) = X(i + 59 - 1);
        }
    }
    camera.rpci = rpci;
}

RpcCamera fit_rpc(const PolynomialCamera &campoly, const GeoImage &dem) {

    // used for estimate geographic coords
    CHECK(!campoly.planet.empty());

    std::string planet = campoly.planet;
    Vector2d corner[4] = {{0.0, 0.0},
                          {(double)campoly.width - 1.0, 0.0},
                          {double(campoly.width - 1.0), double(campoly.height - 1.0)},
                          {0.0, double(campoly.height - 1.0)}};

    // [minx, maxx, miny, maxy]
    int bounds[4] = {INT32_MAX, -INT32_MAX, INT32_MAX, -INT32_MAX};
    // [minlat, maxlat, minlon, maxlon]
    double bounds_gcs[4] = {DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX};

    // 查找最大最小高程值
    for (int i = 0; i < 4; ++i) {
        Vector3d latlon = project_to_geographic_dem(campoly, dem, corner[i], planet);
        Vector2i cell = dem.get_cell_bounded(latlon(1), latlon(0));

        if (cell(0) < bounds[0])
            bounds[0] = cell(0);
        if (cell(0) > bounds[1])
            bounds[1] = cell(0);
        if (cell(1) < bounds[2])
            bounds[2] = cell(1);
        if (cell(1) > bounds[3])
            bounds[3] = cell(1);

        if (latlon(0) < bounds_gcs[0])
            bounds_gcs[0] = latlon(0);
        if (latlon(0) > bounds_gcs[1])
            bounds_gcs[1] = latlon(0);
        if (latlon(1) < bounds_gcs[2])
            bounds_gcs[2] = latlon(1);
        if (latlon(1) > bounds_gcs[3])
            bounds_gcs[3] = latlon(1);
    }

    cv::Mat mat = dem.get_image();
    if (bounds[1] - bounds[0] == 0 || bounds[3] - bounds[2] == 0) {
        LOG(WARNING) << "dem and image have not intersections";
        return RpcCamera();
    }

    cv::Mat sub = mat({bounds[2], bounds[3]}, {bounds[0], bounds[1]});
    cv::Scalar mean, std;
    cv::meanStdDev(sub, mean, std);
    double minmax_dem[2];
    cv::minMaxIdx(sub, &minmax_dem[0], &minmax_dem[1]);
    Vector2d min_max(minmax_dem[0] - 5 * std(0), minmax_dem[1] + 5 * std(0));

    RpcCoefficient rpc;
    rpc.line_off = (campoly.height - 1) / 2.0;
    rpc.line_scale = (campoly.height - 1) / 2.0;
    rpc.samp_off = (campoly.width - 1) / 2.0;
    rpc.samp_scale = (campoly.width - 1) / 2.0;
    rpc.height_off = (min_max[0] + min_max[1]) / 2.0;
    rpc.height_scale = std::abs(min_max[0] - min_max[1]) / 2.0;
    rpc.lat_off = (bounds_gcs[1] + bounds_gcs[0]) / 2.0;
    rpc.lat_scale = (bounds_gcs[1] - bounds_gcs[0]) / 2.0;
    rpc.long_off = (bounds_gcs[3] + bounds_gcs[2]) / 2.0;
    rpc.long_scale = (bounds_gcs[3] - bounds_gcs[2]) / 2.0;

    int row_count = 50, col_count = 50;
    int num_plane = 20;
    int num_points = col_count * row_count * num_plane;

    std::vector<Vector3d> latlon(num_points);
    std::vector<Vector2d> XY(num_points);
    std::vector<Vector3d> PLH(num_points);
    {
        VectorXd lat = VectorXd::LinSpaced(row_count, rpc.lat_off - rpc.lat_scale, rpc.lat_off + rpc.lat_scale);
        VectorXd lon = VectorXd::LinSpaced(col_count, rpc.long_off - rpc.long_scale, rpc.long_off + rpc.long_scale);
        VectorXd hei =
            VectorXd::LinSpaced(num_plane, rpc.height_off - rpc.height_scale, rpc.height_off + rpc.height_scale);

        int i = 0;
        for (int r = 0; r < row_count; ++r) {
            for (int c = 0; c < col_count; ++c) {
                for (int p = 0; p < num_plane; ++p) {
                    latlon[i] = {lat(r), lon(c), hei(p)};
                    ++i;
                }
            }
        }
        std::vector<Vector3d> geoc = latlon_to_geocentric(latlon, planet);
        std::vector<Vector2d> pixels = project(geoc, campoly);

#pragma omp parallel for
        for (int i = 0; i < num_points; ++i) {
            XY[i].x() = (pixels[i](0) - rpc.samp_off) / rpc.samp_scale;
            XY[i].y() = (pixels[i](1) - rpc.line_off) / rpc.line_scale;
            double P, L, H;
            P = (latlon[i](0) - rpc.lat_off) / rpc.lat_scale;
            L = (latlon[i](1) - rpc.long_off) / rpc.long_scale;
            H = (latlon[i](2) - rpc.height_off) / rpc.height_scale;
            PLH[i] = {P, L, H};
        }
    }

    // [x y x y]
    MatrixXd A = MatrixXd::Zero(num_points * 2, 78);
    VectorXd b = VectorXd::Zero(num_points * 2);
    {
#pragma omp parallel for
        for (int i = 0; i < num_points; ++i) {
            double P, L, H;
            P = PLH[i](0);
            L = PLH[i](1);
            H = PLH[i](2);

            double u[20] = {1.0,       L,         P,         H,         L * P,     L * H,     P * H,
                            L * L,     P * P,     H * H,     P * L * H, L * L * L, L * P * P, L * H * H,
                            L * L * P, P * P * P, P * H * H, L * L * H, P * P * H, H * H * H};
            // Y = c.dot(u) / d.dot(u) => c.dot(u) - Y * d.dot(u) = 0
            // X = e.dot(u) / f.dot(u) => e.dot(u) - X * f.dot(u) = 0
            double X = XY[i](0);
            double Y = XY[i](1);
            for (int j = 0; j < 20; ++j) {
                A(i, j) = u[j]; // c - [0, 20]
                if (j != 0)
                    A(i, 20 + j - 1) = -1.0 * Y * u[j]; // d - [20, 39]
                A(num_points + i, 39 + j) = u[j];       // e - [39, 59]
                if (j != 0)
                    A(num_points + i, 59 + j - 1) = -1.0 * X * u[j]; // f - [59. 78]
            }

            b(i) = Y;
            b(i + num_points) = X;
        }
    }

    // c, d, e, f
    VectorXd X = A.householderQr().solve(b);
    //     VectorXd res = A * X - b;
    //     double residule = (A * X - b).norm() / (num_points * 2) * (rpc.line_scale + rpc.samp_scale) / 2.0;
    //     std::string message = string_printf("the rpc fit cost for %s is %.4lf pixels", campoly.filepath.c_str(),
    //     residule);
    //     LOG(INFO) << message;

    rpc.line_num_coeff.resize(20);
    rpc.line_den_coeff.resize(20);
    rpc.samp_num_coeff.resize(20);
    rpc.samp_den_coeff.resize(20);
    rpc.line_den_coeff(0) = 1.0;
    rpc.samp_den_coeff(0) = 1.0;

    for (int i = 0; i < 20; ++i) {
        rpc.line_num_coeff(i) = X(i);
        if (i != 0) {
            rpc.line_den_coeff(i) = X(i + 20 - 1);
        }
        rpc.samp_num_coeff(i) = X(i + 39);
        if (i != 0) {
            rpc.samp_den_coeff(i) = X(i + 59 - 1);
        }
    }

    RpcCamera camera;
    camera.rpc = rpc;
    camera.width = campoly.width;
    camera.height = campoly.height;
    camera.filepath = campoly.filepath;
    camera.planet = campoly.planet;
    camera.rpc = rpc;

    compute_reverse_rpc(camera);

    return camera;
}

RpcCamera refit_rpc(const RpcCamera &camera) {
    RpcCoefficient rpc = camera.rpc;

    int row_count = 50, col_count = 50;
    int num_plane = 20;
    int num_points = col_count * row_count * num_plane;

    std::vector<Vector3d> latlon(num_points);
    std::vector<Vector2d> XY(num_points);
    std::vector<Vector3d> PLH(num_points);
    {
        VectorXd lat = VectorXd::LinSpaced(row_count, rpc.lat_off - rpc.lat_scale, rpc.lat_off + rpc.lat_scale);
        VectorXd lon = VectorXd::LinSpaced(col_count, rpc.long_off - rpc.long_scale, rpc.long_off + rpc.long_scale);
        VectorXd hei =
            VectorXd::LinSpaced(num_plane, rpc.height_off - rpc.height_scale, rpc.height_off + rpc.height_scale);

        int i = 0;
        for (int r = 0; r < row_count; ++r) {
            for (int c = 0; c < col_count; ++c) {
                for (int p = 0; p < num_plane; ++p) {
                    latlon[i] = {lat(r), lon(c), hei(p)};
                    ++i;
                }
            }
        }
        std::vector<Vector2d> pixels = project(latlon, camera);

#pragma omp parallel for
        for (int i = 0; i < num_points; ++i) {
            XY[i].x() = (pixels[i](0) - rpc.samp_off) / rpc.samp_scale;
            XY[i].y() = (pixels[i](1) - rpc.line_off) / rpc.line_scale;
            double P, L, H;
            P = (latlon[i](0) - rpc.lat_off) / rpc.lat_scale;
            L = (latlon[i](1) - rpc.long_off) / rpc.long_scale;
            H = (latlon[i](2) - rpc.height_off) / rpc.height_scale;
            PLH[i] = {P, L, H};
        }
    }

    // [x y x y]
    MatrixXd A = MatrixXd::Zero(num_points * 2, 78);
    VectorXd b = VectorXd::Zero(num_points * 2);
    {
#pragma omp parallel for
        for (int i = 0; i < num_points; ++i) {
            double P, L, H;
            P = PLH[i](0);
            L = PLH[i](1);
            H = PLH[i](2);

            double u[20] = {1.0,       L,         P,         H,         L * P,     L * H,     P * H,
                            L * L,     P * P,     H * H,     P * L * H, L * L * L, L * P * P, L * H * H,
                            L * L * P, P * P * P, P * H * H, L * L * H, P * P * H, H * H * H};
            // Y = c.dot(u) / d.dot(u) => c.dot(u) - Y * d.dot(u) = 0
            // X = e.dot(u) / f.dot(u) => e.dot(u) - X * f.dot(u) = 0
            double X = XY[i](0);
            double Y = XY[i](1);
            for (int j = 0; j < 20; ++j) {
                A(i, j) = u[j]; // c - [0, 20]
                if (j != 0)
                    A(i, 20 + j - 1) = -1.0 * Y * u[j]; // d - [20, 39]
                A(num_points + i, 39 + j) = u[j];       // e - [39, 59]
                if (j != 0)
                    A(num_points + i, 59 + j - 1) = -1.0 * X * u[j]; // f - [59. 78]
            }

            b(i) = Y;
            b(i + num_points) = X;
        }
    }

    // c, d, e, f
    VectorXd X = A.householderQr().solve(b);
    VectorXd res = A * X - b;
    double residule = (A * X - b).norm() / (num_points * 2) * (rpc.line_scale + rpc.samp_scale) / 2.0;
    std::string message = string_printf("the refit rpc fit cost for %s is %.4lf pixels", camera.name.c_str(), residule);
    LOG(INFO) << message;

    rpc.line_num_coeff.resize(20);
    rpc.line_den_coeff.resize(20);
    rpc.samp_num_coeff.resize(20);
    rpc.samp_den_coeff.resize(20);
    rpc.line_den_coeff(0) = 1.0;
    rpc.samp_den_coeff(0) = 1.0;

    for (int i = 0; i < 20; ++i) {
        rpc.line_num_coeff(i) = X(i);
        if (i != 0) {
            rpc.line_den_coeff(i) = X(i + 20 - 1);
        }
        rpc.samp_num_coeff(i) = X(i + 39);
        if (i != 0) {
            rpc.samp_den_coeff(i) = X(i + 59 - 1);
        }
    }

    RpcCamera camera2;
    camera2.rpc = rpc;
    camera2.width = camera.width;
    camera2.height = camera.height;
    camera2.filepath = camera.filepath;
    camera2.planet = camera.planet;
    compute_reverse_rpc(camera2);
    camera2.rpc_aff << 1, 0, 0, 0, 1, 0;
    camera2.init_inverse_transform();

    return camera2;
}
} // namespace h2o
