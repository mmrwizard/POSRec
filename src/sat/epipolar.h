/*!
 * \file epipolar.h
 *
 * \author Han
 * \date 2017/05/11
 *
 * 核线纠正的基类，可派生生成多项式或 RPC 的核线类
 */
#pragma once
#include <base/common.h>

#include "camera_polynomial.h"
#include "camera_rpc.h"

namespace GeographicLib {
class LambertConformalConic;
}
namespace h2o {
/**
 * \brief 核线影像纠正的基类
 *
 * 从原始影像O到核线影像E的关系如下:
 * E(i, j) => P(i, j) => XYZ => latlonh => O(i, j)
 * 其中P为投影坐标系下的值, XYZ为增加高程信息z0，根据 lambert 投影坐标转换到经纬度，依据相机模型投影到原始影像。
 * 其中从E(i, j) => P(i, j) 需要确定一条主核线，是根据最低值和最高值依次投影获取，他们之间是一个仿射变换。
 * E(i, j) => P(i, j)
 */
class EpipolarRectifier {
public:
    EpipolarRectifier();
    ~EpipolarRectifier(){};

    virtual std::tuple<cv::Mat, cv::Mat> rectify(const cv::Mat &fwd_image, const cv::Mat &bwd_image);

    ///\brief 设置核线影像的分辨率
    virtual void set_gsd(double gsd) { gsd_ = gsd; }

    ///\brief 设置求取核线需要用到的最低和最高高程平面
    void set_minmax(const Vector2d &min_max) {
        min_max_ = min_max;
        z0_ = (min_max[0] + min_max[1]) * 0.5;
    }

    ///\brief 核线纠正是在投影坐标系下的，因此需要从经纬度转换到投影坐标系下
    virtual void set_projection(std::shared_ptr<GeographicLib::LambertConformalConic> converter, double lon0) {
        converter_ = converter;
        lon0_ = lon0;
    }

    ///\brief
    auto get_projection() { return converter_; }

    ///\brief 核线影像和原始影像的像素坐标值互转
    Vector2d epipolar_to_original(const Vector2d &epixel, bool is_forward) const;
    Vector2d original_to_epipolar(const Vector2d &opixel, bool is_forward) const;
    virtual Vector3d intersect_latlon(const Vector2d &fwd_pixel, const Vector2d &bwd_pixel) const {
        return Vector3d::Zero();
    };

protected:
    ///\brief 求主核线
    void init_dominant_direction();

    ///\brief 初始化核线影像的大小，核线影像区域为前视和后视影像区域的交集
    void init_epipolar_image();

    ///\brief 利用求得的参数纠正影像
    cv::Mat warp_image(const cv::Mat &image, bool forward);

    /**
     * \brief 往主核线中添加一个点，在左右影像和最低最高高程之间来回切换
     * \param latlon 当前的三维点
     * \param is_forward 当前三维点是否是从前视影像反投影获得的
     * \param is_min_height 当前三维点是否是从最低高程平面获取的
     * \param min_max 最低和最高高程值
     * \return [pixel, latlon] 用当前 latlon 投影获取的影像像素值；和用该像素值反投影获取的物方三维点
     */
    std::tuple<Vector2d, Vector3d> one_step(Vector3d latlon, bool is_forward, bool is_min_height,
                                            const Vector2d &min_max);

    ///\brief 这个函数需要根据不同的相机模型确定，将一个经纬度的三维点投影到影像上得到pixel值
    virtual Vector2d project_latlon(const Vector3d &latlon, bool is_forward) const = 0;

    /**
     * \brief 将像方一个点，依据不同的相机模型和物方的高程z，得到物方的一个经纬度的值
     * \param pixel 像素值
     * \param z 物方地理坐标系下的高程
     * \param is_forward 用前视或后视相机
     * \return [lat, lon]
     */
    virtual Vector2d backproject_latlon(const Vector2d &pixel, double z, bool is_forward) const = 0;

    /**
     * \brief 获取影像的大小
     * \param is_forward 是否是前视相机
     */
    virtual Vector2i get_camera_size(bool is_forward = true) const = 0;

    ///\brief 从经纬度转换到 lambert
    Vector2d latlon_to_lambert(const Vector2d &latlon) const;

    ///\brief 从 lambert 转换到经纬度
    Vector2d lambert_to_latlon(const Vector2d &lambert) const;

protected:
    ///\brief 经纬度到投影坐标系
    std::shared_ptr<GeographicLib::LambertConformalConic> converter_;

    ///\brief 投影坐标系的中央经线
    double lon0_;

    ///\brief 投影到投影坐标系下的平面高程
    double z0_;

    ///\brief 区域的最高和最低边界，用于迭代投影，确定核线的主方向，根据这个确定z0
    Vector2d min_max_;

    ///\brief 地面分辨率
    double gsd_;

    ///\brief 从 lambert 投影坐标系转换到核线影像所在坐标系的变换矩阵 lambert = P * epipolar
    Matrix2d P_epipolar_to_lambert_;

    // brief 上面变换参数的逆变换
    Matrix2d P_lambert_to_epipolar_;

    ///\brief 在核线影像坐标系下两影像交迭处的最大和最小 corner
    Vector2d min_epipolar_;
    Vector2d max_epipolar_;

    ///\brief 判断两影像是否相交
    bool overlapped_;
};

///\brief 创建由多项式模型表达的立体相对的核线纠正模型
std::shared_ptr<EpipolarRectifier> create_epipolar_rectifier_poly_camera(const PolynomialCamera &fwd_camera,
                                                                         const PolynomialCamera &bwd_camera);

///\brief 创建由 RPC 模型表达的立体像对的核线纠正模型
std::shared_ptr<EpipolarRectifier> create_epipolar_rectifier_rpc_camera(const RpcCamera &fwd_camera,
                                                                        const RpcCamera &bwd_camera);

} // namespace h2o
