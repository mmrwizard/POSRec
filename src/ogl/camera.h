/*!
 * \file camera.h
 *
 * \author Han
 * \date 2017/04/06
 *
 *
 */
#pragma once
#include <base/common.h>

namespace h2o {

/**
* \struct Arcball glutil.h nanogui/glutil.h
*
* \brief Arcball helper class to interactively rotate objects on-screen.
*/
struct Arcball {
    Arcball(float speedFactor = 2.0f)
        : mActive(false), mLastPos(Vector2i::Zero()), mSize(Vector2i::Zero()), mQuat(Quaternionf::Identity()),
          mIncr(Quaternionf::Identity()), mSpeedFactor(speedFactor) {}

    Arcball(const Quaternionf &quat)
        : mActive(false), mLastPos(Vector2i::Zero()), mSize(Vector2i::Zero()), mQuat(quat),
          mIncr(Quaternionf::Identity()), mSpeedFactor(2.0f) {}

    Quaternionf &state() { return mQuat; }

    void setState(const Quaternionf &state) {
        mActive = false;
        mLastPos = Vector2i::Zero();
        mQuat = state;
        mIncr = Quaternionf::Identity();
    }

    void setSize(Vector2i size) { mSize = size; }
    const Vector2i &size() const { return mSize; }
    void setSpeedFactor(float speedFactor) { mSpeedFactor = speedFactor; }
    float speedFactor() const { return mSpeedFactor; }
    bool active() const { return mActive; }

    void button(Vector2i pos, bool pressed) {
        mActive = pressed;
        mLastPos = pos;
        if (!mActive)
            mQuat = (mIncr * mQuat).normalized();
        mIncr = Quaternionf::Identity();
    }

    bool motion(Vector2i pos) {
        if (!mActive)
            return false;

        /* Based on the rotation controller form AntTweakBar */
        float invMinDim = 1.0f / mSize.minCoeff();
        float w = (float)mSize.x(), h = (float)mSize.y();

        float ox = (mSpeedFactor * (2 * mLastPos.x() - w) + w) - w - 1.0f;
        float tx = (mSpeedFactor * (2 * pos.x() - w) + w) - w - 1.0f;
        float oy = (mSpeedFactor * (h - 2 * mLastPos.y()) + h) - h - 1.0f;
        float ty = (mSpeedFactor * (h - 2 * pos.y()) + h) - h - 1.0f;

        ox *= invMinDim;
        oy *= invMinDim;
        tx *= invMinDim;
        ty *= invMinDim;

        Vector3f v0(ox, oy, 1.0f), v1(tx, ty, 1.0f);
        if (v0.squaredNorm() > 1e-4f && v1.squaredNorm() > 1e-4f) {
            v0.normalize();
            v1.normalize();
            Vector3f axis = v0.cross(v1);
            float sa = std::sqrt(axis.dot(axis)), ca = v0.dot(v1), angle = std::atan2(sa, ca);
            if (tx * tx + ty * ty > 1.0f)
                angle *= 1.0f + 0.2f * (std::sqrt(tx * tx + ty * ty) - 1.0f);
            mIncr = Eigen::AngleAxisf(angle, axis.normalized());
            if (!std::isfinite(mIncr.norm()))
                mIncr = Quaternionf::Identity();
        }
        return true;
    }

    Matrix4f matrix() const {
        Matrix4f result2 = Matrix4f::Identity();
        result2.block<3, 3>(0, 0) = (mIncr * mQuat).toRotationMatrix();
        return result2;
    }

protected:
    bool mActive;
    Vector2i mLastPos;
    Vector2i mSize;
    Quaternionf mQuat, mIncr;
    float mSpeedFactor;
};

struct CameraParameters {
    ///\brief 控制场景 model 矩阵的旋转大小
    Arcball arcball;

    ///\brief 控制 model 矩阵的 zoom 大小
    float zoom = 1.0f;

    ///\brief 控制 projection 矩阵
    float view_angle = 45.0f;

    ///\brief 控制 projection 矩阵
    float znear = 0.05f, zfar = 12.0f;

    ///\brief screen 大小
    Vector2i size = {0, 0};

    // opengl 的像素尺寸和屏幕尺寸之比
    float pixel_ratio_ = 1.0f;

    ///\brief 控制 view matrix
    Vector3f eye = {0.0f, 0.0f, 5.0f};
    Vector3f center = {0.0f, 0.0f, 0.0f};
    Vector3f up = {0.0f, 1.0f, 0.0f};

    ///\brief model 矩阵的平移
    Vector3f model_translation = Vector3f::Zero();

    ///\brief 交互式控制平移的开始位置
    Vector3f model_translation_start = Vector3f::Zero();

    ///\brief 开始平移时，鼠标点击的位置
    Vector2f screen_translate_start = Vector2f::Zero();

    ///\brief 和 zoom 类似
    float model_zoom = 1.0;

    ///\brief 从二维屏幕坐标转换到三维场景坐标
    Vector3f convert_2d_to_3d(const Vector2f &point2d);

    ///\brief 从三维场景坐标转换到 二维屏幕
    Vector2f convert_3d_to_2d(const Vector3f &point3d);

    ///\brief 获取 model view projection 矩阵
    std::tuple<Matrix4f, Matrix4f, Matrix4f> get_mvp();

    ///\brief 根据场景的 bbox 和 当前的 model view projection 矩阵 来计算 near far
    void update_depth();

    Matrix3f get_world_to_screen_xform();

    ///\brief 场景包围盒，用于计算二三维变换和平移
    BoundingBox3f scene_bbox;

    ///\brief 开始平移操作
    void start_translate(const Vector2f &screen_point);

    /// brief 正在移动
    void motion_translate(const Vector2f &screen_point);

    ///\brief 结束平移操作
    void end_translate(const Vector2f &screen_point);

    ///\brief 开始旋转
    void start_rotate(const Vector2f &screen_point);
    ///\brief 正在旋转时移动
    void motion_rotate(const Vector2f &screen_point);
    ///\brief 结束旋转
    void end_rotate(const Vector2f &screen_point);

    ///\brief 是否采用 正摄投影
    bool is_ortho = true;

    ///\brief 是否正在平移
    bool is_dragging = false;

    ///\brief 是否正在旋转
    bool is_rotate = false;
};

/**
* \brief Projects the vector ``obj`` into the specified viewport.
*
* Performs a homogeneous transformation of a vector into "screen space", as
* defined by the provided model and projection matrices, and the dimensions
* of the viewport.
*
* \param obj
*     The vector being transformed.
*
* \param model
*     The model matrix.
*
* \param proj
*     The projection matrix.
*
* \param viewportSize
*     The dimensions of the viewport to project into.
*/
Vector3f project(const Vector3f &obj, const Matrix4f &model, const Matrix4f &proj, const Vector2i &viewportSize);

/**
* \brief Unprojects the vector ``win`` out of the specified viewport.
*
* The reverse transformation of \ref project --- use the same matrices and
* viewport dimensions to easily transition between the two spaces.
*
* \param win
*     The vector being transformed out of "screen space".
*
* \param model
*     The model matrix.
*
* \param proj
*     The projection matrix.
*
* \param viewportSize
*     The dimensions of the viewport to project out of.
*/
Vector3f unproject(const Vector3f &win, const Matrix4f &model, const Matrix4f &proj, const Vector2i &viewportSize);

/**
* \brief Creates a "look at" matrix that describes the position and
* orientation of e.g. a camera
*
* \param origin
*     The position of the camera.
*
* \param target
*     The gaze target of the camera.
*
* \param up
*     The up vector of the camera.
*
* \rst
* .. warning::
*    These are used to form an orthonormal basis.  The first basis vector is
*    defined as ``f = (target - origin).normalized()``.
* \endrst
*/
Matrix4f lookAt(const Vector3f &origin, const Vector3f &target, const Vector3f &up);

/**
* Creates an orthographic projection matrix.
*
* \param left
*     The left border of the viewport.
*
* \param right
*     The right border of the viewport.
*
* \param bottom
*     The bottom border of the viewport.
*
* \param top
*     The top border of the viewport.
*
* \param nearVal
*     The near plane.
*
* \param farVal
*     The far plane.
*/
extern Matrix4f ortho(float left, float right, float bottom, float top, float nearVal, float farVal);

/**
* Creates a perspective projection matrix.
*
* \param left
*     The left border of the viewport.
*
* \param right
*     The right border of the viewport.
*
* \param bottom
*     The bottom border of the viewport.
*
* \param top
*     The top border of the viewport.
*
* \param nearVal
*     The near plane.
*
* \param farVal
*     The far plane.
*/
extern Matrix4f frustum(float left, float right, float bottom, float top, float nearVal, float farVal);
/**
* \brief Construct homogeneous coordinate scaling matrix
*
* Returns a 3D homogeneous coordinate matrix that scales the X, Y, and Z
* components with the corresponding entries of the 3D vector ``v``. The ``w``
* component is left unchanged
*
* \param v
*     The vector representing the scaling for each axis.
*/
Matrix4f scale(const Vector3f &v);

/**
* \brief Construct homogeneous coordinate translation matrix
*
* Returns a 3D homogeneous coordinate matrix that translates the X, Y, and Z
* components by the corresponding entries of the 3D vector ``v``. The ``w``
* component is left unchanged
*
* \param v
*     The vector representing the translation for each axis.
*/
Matrix4f translate(const Vector3f &v);

} // end h2o
