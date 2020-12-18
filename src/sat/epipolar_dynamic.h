/*
 * @Author: Han
 * @Date: 2018-03-27 18:38:51
 * 根据影像范围动态生成核线信息
 */
#pragma once
#include <base/common.h>
#include <base/disk_image.h>

#include "camera_rpc.h"

namespace h2o {
struct EpipolarDynymicResult {
    // 核线影像信息，可能已经经过降采样了
    cv::Mat epipolar_forward;
    cv::Mat epipolar_bakward;

    // origin = xform * target
    Matrix23d xform;
};

class EpipolarDynamicRectifier {
public:
    // 在创建立体时的对应关系 forward - left/red, backward - right/cyan
    EpipolarDynamicRectifier(const std::string &forward, const std::string &backward, double gsd);
    ~EpipolarDynamicRectifier();

    // 设置基准的高程值
    void set_z0(double z) { z0_ = z; }

    // 这个offset 只是影响可视化因素，并不会影响核线影像和 anaglyph的计算
    // 设置影像左右偏移量
    void set_epipolar_offset(double delta) { delta_ = delta; }
    // 根据经纬度的点自适应调整视差左右偏移量
    void set_epipolar_offset_by_point(const Vector3d &latlon);
    // 获取鼠标点的三维经纬度坐标
    Vector3d get_cursor_point(const Vector2d &cursor) const;
    // 获取 offset 的数值
    double get_offset() const { return delta_; }

    void compute();

    Vector2i get_size() const { return epipolar_size_; }

    // 获取核线影像对应范围处的核线影像，目标大小为target，可能经过降采样
    EpipolarDynymicResult get_epipolar(const BoundingBox2i &roi, const Vector2i &target) const;

    // 直接生成Red/Cyan 立体影像
    std::tuple<Matrix23d, cv::Mat> get_anaglyph(const BoundingBox2i &roi, const Vector2i &target) const;

    ///\brief 核线影像和原始影像的像素坐标值互转，这两个变换是不顾及偏移量的
    Vector2d epipolar_to_original(const Vector2d &epixel, bool is_forward) const;
    Vector2d original_to_epipolar(const Vector2d &opixel, bool is_forward) const;

    const RpcCamera &get_forward_camera() const { return rpc_forward_; }
    const RpcCamera &get_bakward_camera() const { return rpc_bakward_; }
    const RpcCamera &get_forward_epipolar_camera() const { return rpc_forward_epipolar_; }
    const RpcCamera &get_bakward_epipolar_camera() const { return rpc_bakward_epipolar_; }

protected:
    ///\brief 求主核线
    void init_dominant_direction();

    ///\brief 初始化核线影像的大小，核线影像区域为前视和后视影像区域的交集
    void init_epipolar_image();

    ///\brief 拟合核线影像的rpc信息
    void init_epipolar_rpc();

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
    inline Vector2d project_latlon(const Vector3d &latlon, bool is_forward) const;

    /**
     * \brief 将像方一个点，依据不同的相机模型和物方的高程z，得到物方的一个经纬度的值
     * \param pixel 像素值
     * \param z 物方地理坐标系下的高程
     * \param is_forward 用前视或后视相机
     * \return [lat, lon]
     */
    inline Vector2d backproject_latlon(const Vector2d &pixel, double z, bool is_forward) const;

    /**
     * \brief 获取影像的大小
     * \param is_forward 是否是前视相机
     */
    inline Vector2i get_camera_size(bool is_forward = true) const;

    ///\brief 从经纬度转换到 投影坐标
    inline Vector2d latlon_to_utm(const Vector2d &latlon) const;

    ///\brief 从 lambert 转换到经纬度
    inline Vector2d utm_to_latlon(const Vector2d &lambert) const;

    // 根据区域和大小，从原始影像裁剪一个影像
    std::tuple<Matrix23d, cv::Mat> subset_origin_image(const BoundingBox2i &roi, const Vector2i &target,
                                                       bool is_forward) const;
    cv::Mat warp_origin_to_epipolar(const cv::Mat &origin, const Matrix23d &xform, const BoundingBox2i &roi,
                                    const Vector2i &target, const Matrix23d &epipolar_xform, bool is_forward) const;

protected:
    std::string path_forward_;
    std::string path_bakward_;

    Vector2i epipolar_size_;

    int zone_;
    bool north_;

    DiskImagePtr forward_;
    DiskImagePtr bakward_;

    // 原始影像的rpc参数
    RpcCamera rpc_forward_;
    RpcCamera rpc_bakward_;
    // 核线影像的rpc参数
    RpcCamera rpc_forward_epipolar_;
    RpcCamera rpc_bakward_epipolar_;

    ///\brief 投影到投影坐标系下的平面高程
    double z0_;

    // 左像右，右像左移限速
    double delta_;

    ///\brief 区域的最高和最低边界，用于迭代投影，确定核线的主方向，根据这个确定z0
    Vector2d min_max_;

    ///\brief 地面分辨率
    double gsd_;

    ///\brief 从 lambert 投影坐标系转换到核线影像所在坐标系的变换矩阵 utm = P * epipolar
    Matrix2d P_epipolar_to_utm_;

    // brief 上面变换参数的逆变换
    Matrix2d P_utm_to_epipolar_;

    ///\brief 在核线影像坐标系下两影像交迭处的最大和最小 corner
    Vector2d min_epipolar_;
    Vector2d max_epipolar_;

    ///\brief 判断两影像是否相交
    bool overlapped_;
};
typedef std::shared_ptr<EpipolarDynamicRectifier> EpipolarDynamicRectifierPtr;
RpcCamera fit_epipolar_rpc(const RpcCamera &camera, const EpipolarDynamicRectifier *rectifier, bool is_forward);
} // namespace h2o
