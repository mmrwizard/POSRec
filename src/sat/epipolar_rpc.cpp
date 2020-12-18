/*!
 * \file epipolar_rpc.cpp
 *
 * \author Han
 * \date 2017/05/11
 *
 * 采用 RPC 参数进行核线纠正
 */
#include <base/base.h>

#include <sat/epipolar.h>
#include <sat/oneview.h>
#include <sat/twoview.h>

namespace h2o {
class EpipolarRectifierRpc;
RpcCamera fit_epipolar_rpc(const RpcCamera &camera, const EpipolarRectifierRpc *rectifier, bool is_forward);

class EpipolarRectifierRpc : public EpipolarRectifier {
public:
    EpipolarRectifierRpc(const RpcCamera &fwd_camera, const RpcCamera &bwd_camera)
        : fwd_camera_(fwd_camera), bwd_camera_(bwd_camera) {
        P_epipolar_to_lambert_ = Matrix2d::Identity();
        P_lambert_to_epipolar_ = Matrix2d::Identity();
    }

    std::tuple<cv::Mat, cv::Mat> rectify(const cv::Mat &fwd_image, const cv::Mat &bwd_image) override;
    Vector3d intersect_latlon(const Vector2d &fwd_pixel, const Vector2d &bwd_pixel) const override;

protected:
    virtual Vector2i get_camera_size(bool is_forward = true) const override;
    virtual Vector2d project_latlon(const Vector3d &latlon, bool is_forward) const override;
    virtual Vector2d backproject_latlon(const Vector2d &pixel, double z, bool is_forward) const override;

    void init_epipolar_rpc();

private:
    RpcCamera fwd_camera_;
    RpcCamera bwd_camera_;

    RpcCamera fwd_epipolar_camera_;
    RpcCamera bwd_epipolar_camera_;
};

std::tuple<cv::Mat, cv::Mat> EpipolarRectifierRpc::rectify(const cv::Mat &fwd_image, const cv::Mat &bwd_image) {
    cv::Mat fwd_eimage, bwd_eimage;
    init_dominant_direction();
    init_epipolar_image();
    init_epipolar_rpc();

    fwd_eimage = warp_image(fwd_image, true);
    bwd_eimage = warp_image(bwd_image, false);

    return std::make_tuple(fwd_eimage, bwd_eimage);
}

Vector3d EpipolarRectifierRpc::intersect_latlon(const Vector2d &fwd_pixel, const Vector2d &bwd_pixel) const {
    return intersect(fwd_pixel, bwd_pixel, fwd_epipolar_camera_, bwd_epipolar_camera_);
}

Vector2i EpipolarRectifierRpc::get_camera_size(bool is_forward) const {
    if (is_forward) {
        return {fwd_camera_.width, fwd_camera_.height};
    } else {
        return {bwd_camera_.width, bwd_camera_.height};
    }
}
Vector2d EpipolarRectifierRpc::project_latlon(const Vector3d &latlon, bool is_forward) const {
    if (is_forward) {
        return project(latlon, fwd_camera_);
    } else {
        return project(latlon, bwd_camera_);
    }
}
Vector2d EpipolarRectifierRpc::backproject_latlon(const Vector2d &pixel, double z, bool is_forward) const {
    if (is_forward) {
        return project_to_geographic_z(fwd_camera_, z, pixel).head(2);
    } else {
        return project_to_geographic_z(bwd_camera_, z, pixel).head(2);
    }
}

void EpipolarRectifierRpc::init_epipolar_rpc() {
#pragma omp parallel sections
    {
#pragma omp section
        fwd_epipolar_camera_ = fit_epipolar_rpc(fwd_camera_, this, true);
#pragma omp section
        bwd_epipolar_camera_ = fit_epipolar_rpc(bwd_camera_, this, false);
    }
}

std::shared_ptr<EpipolarRectifier> create_epipolar_rectifier_rpc_camera(const RpcCamera &fwd_camera,
                                                                        const RpcCamera &bwd_camera) {
    auto rectifier = std::make_shared<EpipolarRectifierRpc>(fwd_camera, bwd_camera);
    return rectifier;
}

RpcCamera fit_epipolar_rpc(const RpcCamera &camera, const EpipolarRectifierRpc *rectifier, bool is_forward) {
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

        for (int i = 0; i < num_points; ++i) {
            Vector2d pixel = pixels[i];
            // 转换到核线影像坐标系
            pixel = rectifier->original_to_epipolar(pixel, is_forward);

            XY[i].x() = (pixel(0) - rpc.samp_off) / rpc.samp_scale;
            XY[i].y() = (pixel(1) - rpc.line_off) / rpc.line_scale;
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
    VectorXd X = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
    {
        VectorXd res = A * X - b;
        double residule = (A * X - b).norm() / (num_points * 2) * (rpc.line_scale + rpc.samp_scale) / 2.0;
        std::string message =
            string_printf("the epipolar fit cost for %s is %.4lf pixels", camera.name.c_str(), residule);
        LOG(INFO) << message;
    }

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
