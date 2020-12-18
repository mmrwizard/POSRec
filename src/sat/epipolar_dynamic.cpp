/*
 * @Author: Han
 * @Date: 2018-03-27 18:39:04
 * 根据影像范围动态生成核线信息
 */

#include <base/base.h>

#include "camera_rpc.h"
#include "epipolar_dynamic.h"
#include "fitpoly.h"
#include "oneview.h"
#include "twoview.h"

#include <GeographicLib/UTMUPS.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace h2o {

EpipolarDynamicRectifier::EpipolarDynamicRectifier(const std::string &forward, const std::string &backward,
                                                   double gsd) {
    path_bakward_ = forward;
    path_bakward_ = backward;
    gsd_ = gsd;
    delta_ = 0.0;

    DiskImage::build_overviews(forward);
    DiskImage::build_overviews(backward);

#pragma omp parallel sections
    {
#pragma omp section
        {
            forward_ = std::make_shared<DiskImage>(forward);
            forward_->calculate_lut(Vector2i(0, 255));
        }
#pragma omp section
        {
            bakward_ = std::make_shared<DiskImage>(backward);
            bakward_->calculate_lut(Vector2i(0, 255));
        }
    }

    // 初始化 RPC 信息

#pragma omp parallel sections
    {
#pragma omp section
        {
            std::string name = get_filename_noext(forward);
            std::string directory = get_directory(forward);
            std::string rpc_path_forward = join_paths(directory, name + "_rpc.txt");
            rpc_forward_.rpc = load_rpc(rpc_path_forward);
            rpc_forward_.gsd = gsd_;
            rpc_forward_.name = name;
            rpc_forward_.filepath = forward;
            rpc_forward_.width = forward_->get_size()(0);
            rpc_forward_.height = forward_->get_size()(1);
            compute_reverse_rpc(rpc_forward_);
        }
#pragma omp section
        {
            std::string name = get_filename_noext(backward);
            std::string directory = get_directory(backward);
            std::string rpc_path_backward = join_paths(directory, name + "_rpc.txt");
            rpc_bakward_.rpc = load_rpc(rpc_path_backward);
            rpc_bakward_.gsd = gsd_;
            rpc_bakward_.name = name;
            rpc_bakward_.filepath = backward;
            rpc_bakward_.width = bakward_->get_size()(0);
            rpc_bakward_.height = bakward_->get_size()(1);
            compute_reverse_rpc(rpc_bakward_);
        }
    }

    min_max_[0] = rpc_forward_.rpc.height_off - rpc_forward_.rpc.height_scale;
    min_max_[1] = rpc_forward_.rpc.height_off + rpc_forward_.rpc.height_scale;

    double lat0 = rpc_forward_.rpc.lat_off;
    double lon0 = rpc_forward_.rpc.long_off;
    double hei0 = rpc_forward_.rpc.height_off;

    zone_ = GeographicLib::UTMUPS::StandardZone(lat0, lon0);
    north_ = lat0 > 0.0;
    z0_ = hei0;
}

EpipolarDynamicRectifier::~EpipolarDynamicRectifier() {}

void EpipolarDynamicRectifier::set_epipolar_offset_by_point(const Vector3d &latlon) {
    Vector2d forward_point = project(latlon, rpc_forward_epipolar_);
    Vector2d bakward_point = project(latlon, rpc_bakward_epipolar_);
    // 视差
    double disp = bakward_point.x() - forward_point.x();
    delta_ = -disp / 2;
}

Vector3d EpipolarDynamicRectifier::get_cursor_point(const Vector2d &cursor) const {
    // 直接用核线影像的 RPC 进行前方交汇
    Vector2d forward_point, bakward_point;
    forward_point.x() = cursor.x() + delta_;
    forward_point.y() = cursor.y();
    bakward_point.x() = cursor.x() - delta_;
    bakward_point.y() = cursor.y();

    return intersect_rpc_ceres(forward_point, bakward_point, rpc_forward_epipolar_, rpc_bakward_epipolar_);
    // return intersect(forward_point, bakward_point, rpc_forward_epipolar_, rpc_bakward_epipolar_);
}

void EpipolarDynamicRectifier::compute() {
    init_dominant_direction();
    init_epipolar_image();
    init_epipolar_rpc();
}

EpipolarDynymicResult EpipolarDynamicRectifier::get_epipolar(const BoundingBox2i &roi, const Vector2i &target) const {
    EpipolarDynymicResult result;
    result.epipolar_forward = cv::Mat::zeros(target(1), target(0), CV_8U);
    result.epipolar_bakward = cv::Mat::zeros(target(1), target(0), CV_8U);

    Vector2i offset = roi.min();
    Vector2i size = roi.sizes();

    // 估计影像的变换参数
    std::vector<cv::Point2f> origin2d = {cv::Point2f(offset(0) - 0.5, offset(1) - 0.5),
                                         cv::Point2f(offset(0) - 0.5 + size(0), offset(1) - 0.5),
                                         cv::Point2f(offset(0) - 0.5 + size(0), offset(1) - 0.5 + size(1))};
    std::vector<cv::Point2f> target2d = {cv::Point2f(-0.5, -0.5), cv::Point2f(target(0) - 0.5, -0.5),
                                         cv::Point2f(target(0) - 0.5, target(1) - 0.5)};
    cv::Mat M = cv::getAffineTransform(target2d, origin2d);
    cv::cv2eigen(M, result.xform);

    //裁剪原始影像
    cv::Mat origin_forward, origin_backward;
    Matrix23d origin_forward_xform, origin_backward_xform;

    {
        std::tie(origin_forward_xform, origin_forward) = subset_origin_image(roi, target, true);
        std::tie(origin_backward_xform, origin_backward) = subset_origin_image(roi, target, false);
    }

    // 从原始影像 warp 到对应的核线影像
    result.epipolar_forward =
        warp_origin_to_epipolar(origin_forward, origin_forward_xform, roi, target, result.xform, true);
    result.epipolar_bakward =
        warp_origin_to_epipolar(origin_backward, origin_backward_xform, roi, target, result.xform, false);

    return result;
}

std::tuple<Matrix23d, cv::Mat> EpipolarDynamicRectifier::get_anaglyph(const BoundingBox2i &roi,
                                                                      const Vector2i &target) const {
    EpipolarDynymicResult result = get_epipolar(roi, target);

    if (result.epipolar_forward.empty() || result.epipolar_bakward.empty()) {
        return std::make_tuple(result.xform, cv::Mat());
    }

    int rows = result.epipolar_forward.rows;
    int cols = result.epipolar_forward.cols;
    CHECK(result.epipolar_bakward.rows == rows && result.epipolar_bakward.cols == cols)
        << "Epipolar images have different size";

    // 生成立体图

    cv::Mat anaglyph = cv::Mat::zeros(rows, cols, CV_8UC3);
    // 拷贝影像

    const uint8_t *forward_data = result.epipolar_forward.data;
    const uint8_t *bakward_data = result.epipolar_bakward.data;
    Vector3b *anaglyph_data = (Vector3b *)anaglyph.data;

#pragma omp parallel for
    for (int i = 0; i < rows * cols; ++i) {
        Vector3b &pixel = anaglyph_data[i];
        // R,G,B, 为了OPENGL 方便，默认是RGB
        pixel(0) = forward_data[i];
        pixel(1) = bakward_data[i];
        pixel(2) = bakward_data[i];
    }

    return std::make_tuple(result.xform, anaglyph);
}

Vector2d EpipolarDynamicRectifier::epipolar_to_original(const Vector2d &epixel, bool is_forward) const {
    Vector2d epipolar = epixel;
    epipolar = epipolar * gsd_;
    Vector2d utm = P_epipolar_to_utm_ * (epipolar + min_epipolar_);
    Vector2d latlon = utm_to_latlon(utm);
    return project_latlon({latlon(0), latlon(1), z0_}, is_forward);
}

Vector2d EpipolarDynamicRectifier::original_to_epipolar(const Vector2d &opixel, bool is_forward) const {
    Vector2d latlon = backproject_latlon(opixel, z0_, is_forward);
    Vector2d utm = latlon_to_utm(latlon);
    Vector2d epipolar = P_utm_to_epipolar_ * utm - min_epipolar_;
    epipolar = epipolar / gsd_;
    return epipolar;
}

void EpipolarDynamicRectifier::init_dominant_direction() {
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
        BoundingBox2d box_region;
        box_region.extend(Vector2d(0, 0));
        box_region.extend(Vector2d(image_size(0), image_size(1)));
        auto add_half_line = [&]() {
            int niter = 0;
            while (true) {
                std::tie(pixel, latlon) = one_step(latlon, is_forward, is_min_height, min_max_);
                is_forward = !is_forward;
                is_min_height = !is_min_height;
                Vector2i camera_size = get_camera_size(is_forward);

                if (niter < 100) {
                    if (is_forward) {
                        if (!box_region.contains(pixel)) {
                            break;
                        }
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

    std::vector<Vector2d> projection_line;
    {
        // 把 forward 的影像核线投影到 hoff 上
        // 并且把经纬度坐标转换成 lambert
        for (const auto &pixel : fwd_pixels) {
            Vector2d latlon = backproject_latlon(pixel, z0_, true);
            Vector2d projection = latlon_to_utm(latlon);
            projection_line.push_back(projection);
        }
    }

    Vector3d line;
    line = fitline(projection_line);
    //构建新的平面坐标系统，坐标系统中，x轴是直线方向的正交方向，y轴是直线方向。
    Matrix2d P;
    Vector2d axis_y(line(0), line(1));
    axis_y.normalize();
    if (axis_y(1) > 0.0)
        axis_y = -axis_y;
    Vector2d axis_x(-axis_y(1), axis_y(0));
    P << axis_x, axis_y;
    P_epipolar_to_utm_ = P;
    P_utm_to_epipolar_ = P_epipolar_to_utm_.inverse();
}

void EpipolarDynamicRectifier::init_epipolar_image() {
    // 1 把影像的四个角点投影到核线平面
    BoundingBox2d boxes[2];
    bool is_forward[2] = {true, false};
    Vector2d sizes[2] = {get_camera_size(true).cast<double>(), get_camera_size(false).cast<double>()};

    for (int i = 0; i < 2; ++i) {
        std::vector<Vector2d> corners = {Vector2d(0, 0), Vector2d(sizes[i](0), 0), Vector2d(sizes[i](0), sizes[i](1)),
                                         Vector2d(0, sizes[i](1))};
        for (int j = 0; j < corners.size(); ++j) {
            Vector2d point = backproject_latlon(corners[j], z0_, is_forward[i]);
            point = latlon_to_utm(point);

            // 投影到核线影像的平面
            point = P_utm_to_epipolar_ * point;
            boxes[i].extend(point);
        }
    }

    BoundingBox2d box_epipolar;
    box_epipolar = boxes[0].intersection(boxes[1]);
    overlapped_ = !box_epipolar.isEmpty();
    CHECK(overlapped_) << "images should overlap in the epipolar space, otherwise the epipolar image is empty";

    min_epipolar_ = box_epipolar.min();
    max_epipolar_ = box_epipolar.max();

    epipolar_size_(0) = (max_epipolar_(0) - min_epipolar_(0)) / gsd_ + 0.5;
    epipolar_size_(1) = (max_epipolar_(1) - min_epipolar_(1)) / gsd_ + 0.5;
    max_epipolar_(0) = min_epipolar_(0) + epipolar_size_(0) * gsd_;
    max_epipolar_(1) = min_epipolar_(1) + epipolar_size_(1) * gsd_;
}

void EpipolarDynamicRectifier::init_epipolar_rpc() {

#pragma omp parallel sections
    {
#pragma omp section
        rpc_forward_epipolar_ = fit_epipolar_rpc(rpc_forward_, this, true);
#pragma omp section
        rpc_bakward_epipolar_ = fit_epipolar_rpc(rpc_bakward_, this, false);
    }
}

std::tuple<Vector2d, Vector3d> EpipolarDynamicRectifier::one_step(Vector3d latlon, bool is_forward, bool is_min_height,
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

Vector2d EpipolarDynamicRectifier::project_latlon(const Vector3d &latlon, bool is_forward) const {
    if (is_forward) {
        return project(latlon, rpc_forward_);
    } else {
        return project(latlon, rpc_bakward_);
    }
}

Vector2d EpipolarDynamicRectifier::backproject_latlon(const Vector2d &pixel, double z, bool is_forward) const {
    if (is_forward) {
        return project_to_geographic_z(rpc_forward_, z, pixel).head(2);
    } else {
        return project_to_geographic_z(rpc_bakward_, z, pixel).head(2);
    }
}

Vector2i EpipolarDynamicRectifier::get_camera_size(bool is_forward /*= true*/) const {
    return is_forward ? forward_->get_size() : bakward_->get_size();
}

Vector2d EpipolarDynamicRectifier::latlon_to_utm(const Vector2d &latlon) const {
    Vector2d utm;
    // 投影到 平面
    int zone2;
    bool north2;
    GeographicLib::UTMUPS::Forward(latlon(0), latlon(1), zone2, north2, utm(0), utm(1), zone_);
    GeographicLib::UTMUPS::Transfer(zone2, north2, utm(0), utm(1), zone_, north_, utm(0), utm(1), zone2);
    return utm;
}

Vector2d EpipolarDynamicRectifier::utm_to_latlon(const Vector2d &utm) const {
    Vector2d latlon;
    GeographicLib::UTMUPS::Reverse(zone_, north_, utm(0), utm(1), latlon(0), latlon(1));
    return latlon;
}

std::tuple<Matrix23d, cv::Mat>
EpipolarDynamicRectifier::subset_origin_image(const BoundingBox2i &roi, const Vector2i &target, bool is_forward) const {

    Matrix23d xform;
    cv::Mat mat;

    BoundingBox2i roi_origin;
    for (int i = 0; i < 4; ++i) {
        Vector2d corner = roi.corner(BoundingBox2i::CornerType(i)).cast<double>();
        corner = epipolar_to_original(corner, is_forward);
        roi_origin.extend(Vector2i(corner(0) + 0.5, corner(1) + 0.5));
    }

    BoundingBox2i roi_origin_margin;
    roi_origin_margin.extend(Vector2i(0, 0));
    roi_origin_margin.extend(get_camera_size(is_forward));

    roi_origin = roi_origin.intersection(roi_origin_margin);
    if (roi_origin.isEmpty()) {
        return std::make_tuple(xform, mat);
    }

    // 前面的 0.5 是为了计算准确， 后面 0.5 是为了取整
    double scale = (double)roi.sizes()(0) / target(0);
    Vector2i target_origin(((double)roi_origin.sizes()(0) + 0.5) / scale - 0.5 + 0.5,
                           ((double)roi_origin.sizes()(1) + 0.5) / scale - 0.5 + 0.5);

    if (is_forward) {
        return forward_->get_patch_8u(roi_origin, target_origin);
    } else {
        return bakward_->get_patch_8u(roi_origin, target_origin);
    }
}

Matrix23d inverse_affine(const Matrix23d &xform) {
    Matrix23d ixform;
    // p' = Ap +b
    // p = A-1(p'-b)
    Matrix2d A = xform.block(0, 0, 2, 2);
    Vector2d b = xform.col(2);

    ixform.block(0, 0, 2, 2) = A.inverse();
    ixform.col(2) = -A.inverse() * b;
    return ixform;
}

cv::Mat EpipolarDynamicRectifier::warp_origin_to_epipolar(const cv::Mat &origin, const Matrix23d &xform,
                                                          const BoundingBox2i &roi, const Vector2i &target,
                                                          const Matrix23d &epipolar_xform, bool is_forward) const {
    // 计算逆变换
    Matrix23d ixform = inverse_affine(xform);

    cv::Mat dst = cv::Mat::zeros(target(1), target(0), CV_8U);

    const int BLOCK_SZ = 32;

    int width = dst.cols, height = dst.rows;

    int bh0 = std::min(BLOCK_SZ / 2, height);
    int bw0 = std::min(BLOCK_SZ * BLOCK_SZ / bh0, width);
    bh0 = std::min(BLOCK_SZ * BLOCK_SZ / bw0, height);

#pragma omp parallel for
    for (int r = 0; r < height; r += bh0) {
        for (int c = 0; c < width; c += bw0) {
            int bw = std::min(bw0, width - c);
            int bh = std::min(bh0, height - r); // height
            bh = std::min(bh, height - r);

            float X_DATA[BLOCK_SZ * BLOCK_SZ];
            float Y_DATA[BLOCK_SZ * BLOCK_SZ];

            cv::Mat mapx(bh, bw, CV_32F, X_DATA);
            cv::Mat mapy(bh, bw, CV_32F, Y_DATA);

            cv::Mat dst_sub = dst(cv::Rect(c, r, bw, bh));
            std::vector<Vector2d> corner_epipolar = {{(double)c, (double)r},
                                                     {(double)c + bw, (double)r},
                                                     {(double)c + bw, (double)r + bh},
                                                     {(double)c, (double)r + bh}};

            std::vector<Vector2d> corner_origin(4);
            // 把核线影像转换到原始影像
            for (int i = 0; i < 4; ++i) {
                Vector2d pt = corner_epipolar[i];
                // 变换到原始核线分辨率
                pt = epipolar_xform * pt.homogeneous();
                // 变换到原始影像
                pt = epipolar_to_original(pt, is_forward);
                // 变换到原始影像的重采样
                pt = ixform * pt.homogeneous();
                corner_origin[i] = pt;
            }

            int pos = 0;
            for (int y1 = 0; y1 < bh; y1++) {
                for (int x1 = 0; x1 < bw; x1++) {
                    //双线性内插
                    double wx = (double)(bw - x1) / bw;
                    double wy = (double)(bh - y1) / bh;

                    Vector2d pt_r0 = corner_origin[0] * wx + corner_origin[1] * (1 - wx);
                    Vector2d pt_rbh = corner_origin[3] * wx + corner_origin[2] * (1 - wx);
                    Vector2d pt = (pt_r0 * wy + pt_rbh * (1 - wy));

                    X_DATA[pos] = pt.x();
                    Y_DATA[pos] = pt.y();
                    ++pos;
                }
            }

            remap_ipp(origin, mapx, mapy, cv::INTER_CUBIC).copyTo(dst_sub);
        }
    }

    return dst;
}

RpcCamera fit_epipolar_rpc(const RpcCamera &camera, const EpipolarDynamicRectifier *rectifier, bool is_forward) {
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
