/*!
 * \file epipolar.cpp
 *
 * \author Han
 * \date 2017/05/11
 *
 * 核线纠正的基类，可派生生成多项式或 RPC 的核线类
 */
#include <base/common.h>
#include <base/imgproc.h>

#include "epipolar.h"
#include "fitpoly.h"

#include <Eigen/LU>
#include <Eigen/QR>
#include <GeographicLib/LambertConformalConic.hpp>
#include <opencv2/imgproc.hpp>

namespace h2o {
EpipolarRectifier::EpipolarRectifier() {
    min_epipolar_ = {0.0, 0.0};
    max_epipolar_ = {0.0, 0.0};
    min_max_ = {0.0, 0.0};
}
std::tuple<cv::Mat, cv::Mat> EpipolarRectifier::rectify(const cv::Mat &fwd_image, const cv::Mat &bwd_image) {
    cv::Mat fwd_eimage, bwd_eimage;
    init_dominant_direction();
    init_epipolar_image();
    fwd_eimage = warp_image(fwd_image, true);
    bwd_eimage = warp_image(bwd_image, false);

    return std::make_tuple(fwd_eimage, bwd_eimage);
}

Vector2d EpipolarRectifier::epipolar_to_original(const Vector2d &epixel, bool is_forward) const {
    Vector2d epipolar = epixel * gsd_;
    Vector2d lambert = P_epipolar_to_lambert_ * (epipolar + min_epipolar_);
    Vector2d latlon = lambert_to_latlon(lambert);
    return project_latlon({latlon(0), latlon(1), z0_}, is_forward);
}

Vector2d EpipolarRectifier::original_to_epipolar(const Vector2d &opixel, bool is_forward) const {
    Vector2d latlon = backproject_latlon(opixel, z0_, is_forward);
    Vector2d lambert = latlon_to_lambert(latlon);
    Vector2d epipolar = P_lambert_to_epipolar_ * lambert - min_epipolar_;
    Vector2d epixel = epipolar / gsd_;
    return epixel;
}

void EpipolarRectifier::init_dominant_direction() {
    std::vector<Vector2d> fwd_pixels;
    std::vector<Vector2d> bwd_pixels;
    // 在最低 最高平面之间依次迭代投影和反投影，确定主核线
    {
        Vector2i image_size = get_camera_size(true);
        Vector2d center = (image_size.cast<double>() - Vector2d(1, 1)) / 2;
        bool is_forward = true;
        bool is_min_height = true;
        Vector3d latlon;
        latlon.head(2) = backproject_latlon(center, min_max_(0), is_forward);
        latlon(2) = min_max_(0);
        Vector2d pixel = center;
        fwd_pixels.push_back(center);

        auto add_half_line = [&]() {
            int niter = 0;
            while (true) {
                std::tie(pixel, latlon) = one_step(latlon, is_forward, is_min_height, min_max_);
                is_forward = !is_forward;
                is_min_height = !is_min_height;
                Vector2i camera_size = get_camera_size(is_forward);
                if (niter < 20) {
                    if (is_forward) {
                        fwd_pixels.push_back(pixel);
                    } else {
                        bwd_pixels.push_back(pixel);
                    }
                    ++niter;
                } else {
                    break;
                }
            }
        };
        add_half_line();

        is_forward = true;
        is_min_height = false;
        latlon.head(2) = backproject_latlon(center, min_max_(1), is_forward);
        latlon(2) = min_max_(1);
        pixel = center;

        add_half_line();
    }

    std::vector<Vector2d> lambert_line;
    {
        // 把 forward 的影像核线投影到 hoff 上
        // 并且把经纬度坐标转换成 lambert
        for (const auto &pixel : fwd_pixels) {
            Vector2d latlon = backproject_latlon(pixel, z0_, true);
            Vector2d lambert;
            converter_->Forward(lon0_, latlon(0), latlon(1), lambert(0), lambert(1));
            lambert_line.push_back(lambert);
        }
    }

    Vector3d line;
    line = fitline(lambert_line);
    //构建新的平面坐标系统，坐标系统中，x轴是直线方向的正交方向，y轴是直线方向。
    Matrix2d P;
    Vector2d axis_y(line(0), line(1));
    axis_y.normalize();
    if (axis_y(1) > 0.0)
        axis_y = -axis_y;
    Vector2d axis_x(-axis_y(1), axis_y(0));
    P << axis_x, axis_y;
    P_epipolar_to_lambert_ = P;
    P_lambert_to_epipolar_ = P_epipolar_to_lambert_.inverse();
}

void EpipolarRectifier::init_epipolar_image() {
    // 1 把影像的四个角点投影到核线平面
    BoundingBox2d boxes[2];
    bool is_forward[2] = {true, false};
    Vector2d sizes[2] = {get_camera_size(true).cast<double>(), get_camera_size(false).cast<double>()};

    for (int i = 0; i < 2; ++i) {
        std::vector<Vector2d> corners = {Vector2d(0, 0), Vector2d(sizes[i](0), 0), Vector2d(sizes[i](0), sizes[i](1)),
                                         Vector2d(0, sizes[i](1))};
        for (int j = 0; j < corners.size(); ++j) {
            Vector2d point = backproject_latlon(corners[j], z0_, is_forward[i]);
            // 投影到 lambert
            converter_->Forward(lon0_, point(0), point(1), point(0), point(1));
            // 投影到核线影像的平面
            point = P_lambert_to_epipolar_ * point;
            boxes[i].extend(point);
        }
    }

    BoundingBox2d box_epipolar;
    box_epipolar = boxes[0].intersection(boxes[1]);
    overlapped_ = !box_epipolar.isEmpty();
    CHECK(overlapped_) << "images should overlap in the epipolar space, otherwise the epipolar image is empty";

    min_epipolar_ = box_epipolar.min();
    max_epipolar_ = box_epipolar.max();
}

class EpipolarRectifierInvoker : public cv::ParallelLoopBody {
public:
    EpipolarRectifierInvoker(const cv::Mat &_src, cv::Mat &_dst, EpipolarRectifier *_rectifier, bool _is_forward)
        : src(_src), dst(_dst), rectifier(_rectifier), is_forward(_is_forward) {}

    void operator()(const cv::Range &range) const {
        const int BLOCK_SZ = 32;
        float X_DATA[BLOCK_SZ * BLOCK_SZ];
        float Y_DATA[BLOCK_SZ * BLOCK_SZ];

        int width = dst.cols, height = dst.rows;

        int bh0 = std::min(BLOCK_SZ / 2, height);
        int bw0 = std::min(BLOCK_SZ * BLOCK_SZ / bh0, width);
        bh0 = std::min(BLOCK_SZ * BLOCK_SZ / bw0, height);

        for (int r = range.start; r < range.end; r += bh0) {
            for (int c = 0; c < width; c += bw0) {
                int bw = std::min(bw0, width - c);
                int bh = std::min(bh0, range.end - r); // height
                bh = std::min(bh, height - r);

                cv::Mat mapx(bh, bw, CV_32F, X_DATA);
                cv::Mat mapy(bh, bw, CV_32F, Y_DATA);

                cv::Mat dst_sub = dst(cv::Rect(c, r, bw, bh));
                std::vector<Vector2d> corner_epipolar = {{(double)c, (double)r},
                                                         {(double)c + bw, (double)r},
                                                         {(double)c + bw, (double)r + bh},
                                                         {(double)c, (double)r + bh}};

                std::vector<Vector2d> corner_origin;
                for (const auto &corner : corner_epipolar) {
                    corner_origin.push_back(rectifier->epipolar_to_original(corner, is_forward));
                }

                int pos = 0;
                for (int y1 = 0; y1 < bh; y1++) {
                    for (int x1 = 0; x1 < bw; x1++) {
                        //双线性内插
                        double wx = (double)(bw - x1) / bw;
                        double wy = (double)(bh - y1) / bh;

                        auto pt_r0 = corner_origin[0] * wx + corner_origin[1] * (1 - wx);
                        auto pt_rbh = corner_origin[3] * wx + corner_origin[2] * (1 - wx);

                        X_DATA[pos] = (pt_r0 * wy + pt_rbh * (1 - wy)).x();
                        Y_DATA[pos] = (pt_r0 * wy + pt_rbh * (1 - wy)).y();
                        ++pos;
                    }
                }

                remap_ipp(src, mapx, mapy, cv::INTER_NEAREST).copyTo(dst_sub);
            }
        }
    }

    cv::Mat &dst;
    const cv::Mat &src;
    EpipolarRectifier *rectifier;
    bool is_forward;
};

cv::Mat EpipolarRectifier::warp_image(const cv::Mat &src, bool forward) {
    cv::Mat dst;
    if (overlapped_) {
        Vector2d diag = max_epipolar_ - min_epipolar_;
        int cols = int(diag(0) / gsd_);
        int rows = int(diag(1) / gsd_);

        cv::Mat blur = src;
        int blur_size = std::max(src.cols / cols, src.rows / rows);
        blur_size = std::max(blur_size, 1);
        cv::blur(blur, blur, {blur_size, blur_size}, {-1, -1}, cv::BORDER_REPLICATE);

        {
            // 处理无效值
            cv::Mat mask = src != 0;
            cv::blur(mask, mask, {blur_size, blur_size}, {-1, -1}, cv::BORDER_REPLICATE);
            blur.setTo(cv::Scalar(0), mask != 255);
        }

        dst = cv::Mat::zeros(rows, cols, src.type());
        EpipolarRectifierInvoker invoker(blur, dst, this, forward);
        cv::parallel_for_({0, rows}, invoker, Eigen::nbThreads());
    }
    return dst;
}

std::tuple<Vector2d, Vector3d> EpipolarRectifier::one_step(Vector3d latlon, bool is_forward, bool is_min_height,
                                                           const Vector2d &min_max) {
    Vector2d pixel;
    // 如果是前视的点，则投影到后视，反之亦然
    pixel = project_latlon(latlon, !is_forward);

    // 如果当前点是在 min height 则下个点反投影到 max ； 反之亦然
    double z = is_min_height ? min_max[1] : min_max[0];
    latlon.head(2) = backproject_latlon(pixel, z, !is_forward);
    latlon(2) = z;

    return std::make_tuple(pixel, latlon);
}
Vector2d EpipolarRectifier::latlon_to_lambert(const Vector2d &latlon) const {
    Vector2d lambert;
    converter_->Forward(lon0_, latlon(0), latlon(1), lambert(0), lambert(1));
    return lambert;
}
Vector2d EpipolarRectifier::lambert_to_latlon(const Vector2d &lambert) const {
    Vector2d latlon;
    converter_->Reverse(lon0_, lambert(0), lambert(1), latlon(0), latlon(1));
    return latlon;
}
} // namespace h2o
