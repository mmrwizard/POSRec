/*!
 * \file camera_polynomial.cpp
 *
 * \author Han
 * \date 2017/05/04
 *
 * 多项式外方位元素的相机模型
 */
#include <base/base.h>

#include "camera_polynomial.h"
#include "fitpoly.h"
#include "oneview.h"
#include "serialization.h"

#include <Eigen/Geometry>
#include <Eigen/LU>

namespace h2o {
void PolynomialCamera::init_io_param() {
    io(0, 0) = pixel;
    io(0, 1) = 0.0;
    io(0, 2) = -(width - 1) / 2.0 * pixel - x0;

    io(1, 0) = 0.0;
    io(1, 1) = -pixel;
    io(1, 2) = (height - 1) / 2.0 * pixel;

    iob.leftCols(2) = io.leftCols(2).inverse();
    iob.col(2) = -io.leftCols(2).inverse() * io.col(2);
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d> PolynomialCamera::get_eo_by_mm(double y) const {
    Vector3d C;
    Vector3d r;
    Matrix3d R;

    VectorXd eo = VectorXd::Zero(EO_COUNT);
    for (int i = 0; i < EO_COUNT; ++i) {
        int order = orders[i];
        for (int j = 0; j <= order; ++j) {
            eo[i] += eos[i][j] * std::pow(y, j);
        }
    }

    C = eo.head(3);
    r = eo.tail(3);
    R = rodrigues(r);

    return std::make_tuple(C, R);
}

std::tuple<Vector3d, Vector3d> PolynomialCamera::get_Cr_by_mm(double y) const {
    Vector3d C;
    Vector3d r;

    VectorXd eo = VectorXd::Zero(EO_COUNT);
    for (int i = 0; i < EO_COUNT; ++i) {
        int order = orders[i];
        for (int j = 0; j <= order; ++j) {
            eo[i] += eos[i][j] * std::pow(y, j);
        }
    }

    C = eo.head(3);
    r = eo.tail(3);

    return std::make_tuple(C, r);
}

MatrixXd PolynomialCamera::get_P_by_mm(double y) const {
    Vector3d C;
    Matrix3d R;
    std::tie(C, R) = get_eo_by_mm(y);
    // 摄影测量转换到计算机视觉，旋转矩阵需要改变
    Matrix3d R_x = Eigen::AngleAxisd(M_PI, Vector3d::UnitX()).toRotationMatrix();
    Matrix3d R_cv = R * R_x;
    R_cv.transposeInPlace();

    Matrix3d K = Matrix3d::Zero();
    K(0, 0) = K(1, 1) = f / pixel;
    K(0, 2) = (width - 1) / 2.0 + x0 / pixel;
    K(1, 2) = 0.0;
    K(2, 2) = 1.0;

    MatrixXd P = MatrixXd::Zero(3, 4);
    P << R_cv, (-R_cv) * C;
    P = K * P;
    return P;
}

Vector3d PolynomialCamera::get_view_direction(double y) const {
    // 相机坐标系下的 (0, 0, 1) 在世界坐标系的坐标

    Vector3d C;
    Matrix3d R;
    std::tie(C, R) = get_eo_by_mm(y);
    Matrix3d R_x = Eigen::AngleAxisd(M_PI, Vector3d::UnitX()).toRotationMatrix();
    Matrix3d R_cv = R * R_x;
    R_cv.transposeInPlace();

    Vector3d z_axis = {0.0, 0.0, 1.0};
    Vector3d dir = R_cv * z_axis;
    dir.normalize();

    return dir;
}

Vector2d PolynomialCamera::to_mm(const Vector2d &pixel) const {
    Vector2d mm = io.leftCols(2) * pixel + io.col(2);
    return mm;
}

Vector2d PolynomialCamera::to_pixel(const Vector2d &mm) const {
    Vector2d pixel = iob.leftCols(2) * mm + iob.col(2);
    return pixel;
}

void save_poly_camera(const std::string &path, const PolynomialCamera &camera) {
    CHECK(get_extension_name(path) == ".json");
    std::ofstream ofile(string_utf8_to_fstream(path));
    cereal::JSONOutputArchive archive(ofile);
    archive(camera);
}

PolynomialCamera load_poly_camera(const std::string &path) {
    CHECK(get_extension_name(path) == ".json");
    PolynomialCamera camera;
    std::ifstream ifile(string_utf8_to_fstream(path));
    cereal::JSONInputArchive archive(ifile);
    archive(camera);
    return camera;
}

void fit_polynomial(PolynomialCamera &camera, std::vector<Vector3d> &Cs, const std::vector<Matrix3d> &Rs,
                    CameraCoordinate type) {
    CHECK_EQ(Cs.size(), Rs.size());
    CHECK_EQ(Cs.size(), camera.height);

    std::vector<Vector3d> rs;
    rs.reserve(Rs.size());

    Matrix3d O = Matrix3d::Identity();

    switch (type) {
    case XRightYUp:
        break;
    case XDownYRight:
        //绕z轴旋转90°
        O = AngleAxisd(M_PI / 2.0, Vector3d::UnitZ()).toRotationMatrix();
        break;
    case XDownYLeft: {
        // 绕 x 轴旋转180°绕 z 轴旋转90° 因为后面要转至，所以这里顺序反着乘
        auto O1 = AngleAxisd(M_PI / 2.0, Vector3d::UnitZ()).toRotationMatrix();
        auto O2 = AngleAxisd(M_PI, Vector3d::UnitX()).toRotationMatrix();
        O = O1 * O2;
        break;
    }
    case XUpYLeft:
        // 绕z轴旋转 -90°
        O = AngleAxisd(-M_PI / 2.0, Vector3d::UnitZ()).toRotationMatrix();
        break;
    case XUpYRight: {
        // 绕 x 轴旋转180°绕 z 轴旋转-90° 因为后面要转至，所以这里顺序反着乘
        auto O1 = AngleAxisd(-M_PI / 2.0, Vector3d::UnitZ()).toRotationMatrix();
        auto O2 = AngleAxisd(M_PI, Vector3d::UnitX()).toRotationMatrix();
        O = O1 * O2;
    }
    default:
        break;
    }
    for (Matrix3d R : Rs) {
        R = R * O.transpose();
        rs.push_back(rodrigues(R));
    }

    int rows = camera.height;
    std::vector<std::array<double, 6>> eo_data(rows);
    for (int i = 0; i < rows; ++i) {
        eo_data[i][0] = Cs[i][0];
        eo_data[i][1] = Cs[i][1];
        eo_data[i][2] = Cs[i][2];
        eo_data[i][3] = rs[i][0];
        eo_data[i][4] = rs[i][1];
        eo_data[i][5] = rs[i][2];
    }
    // 拟合多项式
    // x 是 line 的毫米值，y 是对应的 EO 元素值

    for (int i = 0; i < EO_COUNT; ++i) {
        VectorXd x(rows), y(rows);
        for (int j = 0; j < rows; ++j) {
            Vector2d pixel(0, j);
            Vector2d mm = camera.to_mm(pixel);
            x[j] = mm.y();
            y[j] = eo_data[j][i];
        }
        VectorXd poly = fitpoly(x, y, camera.orders[i]);
        camera.eos[i] = poly;
    }
}
} // namespace h2o
