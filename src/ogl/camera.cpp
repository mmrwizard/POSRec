/*!
 * \file camera.cpp
 *
 * \author Han
 * \date 2017/04/06
 *
 * 相机操作
 */
#include <ogl/camera.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
namespace h2o {

Eigen::Vector3f project(const Eigen::Vector3f &obj, const Eigen::Matrix4f &model, const Eigen::Matrix4f &proj,
                        const Vector2i &viewportSize) {
    Eigen::Vector4f tmp;
    tmp << obj, 1;

    tmp = model * tmp;

    tmp = proj * tmp;

    tmp = tmp.array() / tmp(3);
    tmp = tmp.array() * 0.5f + 0.5f;
    tmp(0) = tmp(0) * viewportSize.x();
    tmp(1) = tmp(1) * viewportSize.y();

    return tmp.head(3);
}

Eigen::Vector3f unproject(const Eigen::Vector3f &win, const Eigen::Matrix4f &model, const Eigen::Matrix4f &proj,
                          const Vector2i &viewportSize) {
    Eigen::Matrix4f Inverse = (proj * model).inverse();

    Eigen::Vector4f tmp;
    tmp << win, 1;
    tmp(0) = tmp(0) / viewportSize.x();
    tmp(1) = tmp(1) / viewportSize.y();
    tmp = tmp.array() * 2.0f - 1.0f;

    Eigen::Vector4f obj = Inverse * tmp;
    obj /= obj(3);

    return obj.head(3);
}

Eigen::Matrix4f lookAt(const Eigen::Vector3f &origin, const Eigen::Vector3f &target, const Eigen::Vector3f &up) {
    Eigen::Vector3f f = (target - origin).normalized();
    Eigen::Vector3f s = f.cross(up).normalized();
    Eigen::Vector3f u = s.cross(f);

    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result(0, 0) = s(0);
    result(0, 1) = s(1);
    result(0, 2) = s(2);
    result(1, 0) = u(0);
    result(1, 1) = u(1);
    result(1, 2) = u(2);
    result(2, 0) = -f(0);
    result(2, 1) = -f(1);
    result(2, 2) = -f(2);
    result(0, 3) = -s.transpose() * origin;
    result(1, 3) = -u.transpose() * origin;
    result(2, 3) = f.transpose() * origin;
    return result;
}

Eigen::Matrix4f ortho(float left, float right, float bottom, float top, float nearVal, float farVal) {
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result(0, 0) = 2.0f / (right - left);
    result(1, 1) = 2.0f / (top - bottom);
    result(2, 2) = -2.0f / (farVal - nearVal);
    result(0, 3) = -(right + left) / (right - left);
    result(1, 3) = -(top + bottom) / (top - bottom);
    result(2, 3) = -(farVal + nearVal) / (farVal - nearVal);
    return result;
}

Eigen::Matrix4f frustum(float left, float right, float bottom, float top, float nearVal, float farVal) {
    Eigen::Matrix4f result = Eigen::Matrix4f::Zero();
    result(0, 0) = (2.0f * nearVal) / (right - left);
    result(1, 1) = (2.0f * nearVal) / (top - bottom);
    result(0, 2) = (right + left) / (right - left);
    result(1, 2) = (top + bottom) / (top - bottom);
    result(2, 2) = -(farVal + nearVal) / (farVal - nearVal);
    result(3, 2) = -1.0f;
    result(2, 3) = -(2.0f * farVal * nearVal) / (farVal - nearVal);
    return result;
}

Eigen::Matrix4f scale(const Eigen::Vector3f &v) { return Eigen::Affine3f(Eigen::Scaling(v)).matrix(); }

Eigen::Matrix4f translate(const Eigen::Vector3f &v) {
    return Eigen::Affine3f(Eigen::Translation<float, 3>(v)).matrix();
}

Eigen::Matrix4f perspective(float vfov, float aspect, float znear, float zfar) {
    float const tanHalfFovy = tan(vfov / static_cast<float>(2));

    Eigen::Matrix4f P = Matrix4f::Zero();

    P(0, 0) = 1.0f / (aspect * tanHalfFovy);
    P(1, 1) = 1.0f / tanHalfFovy;
    P(2, 2) = -(znear + zfar) / (zfar - znear);
    P(2, 3) = -2 * znear * zfar / (zfar - znear);
    P(3, 2) = -1;
    P(3, 3) = 0.0;
    return P;
}

Vector3f CameraParameters::convert_2d_to_3d(const Vector2f &point2d) {
    Matrix4f model, view, proj;
    std::tie(model, view, proj) = get_mvp();

    Vector2i viewport_size = (size.cast<float>() * pixel_ratio_).array().round().cast<int>();

    // 输入的是 device independent 的屏幕像素，转换到 opengl 需要乘缩放
    float zval = project(scene_bbox.center(), view * model, proj, viewport_size).z();
    Vector3f screen2d = Vector3f(point2d.x() * pixel_ratio_, point2d.y() * pixel_ratio_, zval);

    // size 也是 device independent的值，而不是opengl 的viewport
    Vector3f point3d = unproject(Eigen::Vector3f(screen2d.x(), viewport_size.y() - screen2d.y(), screen2d.z()),
                                 view * model, proj, viewport_size);
    return point3d;
}

Vector2f CameraParameters::convert_3d_to_2d(const Vector3f &point3d) {
    Eigen::Matrix4f model, view, proj;
    std::tie(model, view, proj) = get_mvp();
    Vector2i viewport_size = (size.cast<float>() * pixel_ratio_).array().round().cast<int>();
    Vector3f screen2d = project(point3d.cast<float>(), view * model, proj, viewport_size);
    screen2d.y() = viewport_size.y() - screen2d.y();
    screen2d *= 1.0f / pixel_ratio_; // 从 viewport 变换 屏幕尺寸
    return screen2d.head(2);
}

std::tuple<Matrix4f, Matrix4f, Matrix4f> CameraParameters::get_mvp() {
    Matrix4f model, view, proj;
    view = lookAt(eye, center, up);

    proj = perspective(view_angle * DEG2RAD, (double)size.x() / (double)size.y(), znear, zfar);

    Matrix4f rotation = arcball.matrix();
    model = rotation * scale(Vector3f::Constant(zoom * model_zoom)) * translate(model_translation);

    return std::make_tuple(model, view, proj);
}

void CameraParameters::update_depth() {
    // 在相机坐标系的 bounding box
    BoundingBox3f bbox_in_camera;

    Matrix4f model, view, proj;
    std::tie(model, view, proj) = get_mvp();

    Matrix4f mv = view * model;

    for (int i = 0; i < 8; ++i) {
        Vector3f corner = scene_bbox.corner(BoundingBox3f::CornerType(i));
        corner = (mv * corner.homogeneous()).eval().hnormalized();
        bbox_in_camera.extend(corner);
    }

    double depth = bbox_in_camera.sizes().z();
    double extend = std::abs(depth * 0.5);

    double minz = std::min(std::abs(bbox_in_camera.min().z()), std::abs(bbox_in_camera.max().z()));
    double maxz = std::max(std::abs(bbox_in_camera.min().z()), std::abs(bbox_in_camera.max().z()));

    znear = std::max(0.0, minz - extend);
    zfar = std::max(0.0, maxz + extend);
}

Matrix3f CameraParameters::get_world_to_screen_xform() {
    // 屏幕的四个角点和世界坐标系的四个角点估计一个单应矩阵

    std::vector<cv::Point2d> point2d(4), screen2d(4);
    std::vector<Vector2f> corners = {Vector2f(0, 0), Vector2f(size(0), 0), Vector2f(size(0), size(1)),
                                     Vector2f(0, size(1))};
    for (int i = 0; i < 4; ++i) {
        Vector2f corner = corners[i];
        screen2d[i] = cv::Point2d(corner(0), corner(1));
        corner = convert_2d_to_3d(corner).head(2);
        point2d[i] = cv::Point2d(corner(0), corner(1));
    }
    // dst = H * src;
    cv::Mat H = cv::findHomography(point2d, screen2d);
    Matrix3f eigen_H;
    cv::cv2eigen(H, eigen_H);
    return eigen_H;
}

void CameraParameters::start_translate(const Vector2f &screen_point) {
    model_translation_start = model_translation;
    screen_translate_start = screen_point;
    is_dragging = true;
}

void CameraParameters::motion_translate(const Vector2f &screen_point) {
    if (!is_dragging)
        return;

    Vector3f translate = convert_2d_to_3d(screen_point) - convert_2d_to_3d(screen_translate_start);
    model_translation = model_translation_start + translate;
}

void CameraParameters::end_translate(const Vector2f &screen_point) {
    motion_translate(screen_point);
    is_dragging = false;
    screen_translate_start = Vector2f::Zero();
    model_translation_start = Vector3f::Zero();
}

void CameraParameters::start_rotate(const Vector2f &screen_point) {
    arcball.button(screen_point.cast<int>(), true);
    is_rotate = true;
}

void CameraParameters::motion_rotate(const Vector2f &screen_point) {
    if (!is_rotate)
        return;
    arcball.motion(screen_point.cast<int>());
}

void CameraParameters::end_rotate(const Vector2f &screen_point) {
    motion_rotate(screen_point);

    arcball.button(screen_point.cast<int>(), false);
    is_rotate = false;
}

} // namespace h2o
