/*!
 * \file camera_polynomial.h
 *
 * \author Han
 * \date 2017/05/04
 *
 * 多项式外方位元素的相机模型
 */
#pragma once
#include <base/common.h>

#include <array>

namespace h2o {
enum EOElements { EO_POS_X = 0, EO_POS_Y, EO_POS_Z, EO_ROT_1, EO_ROT_2, EO_ROT_3, EO_COUNT };
enum CameraCoordinate { XRightYUp = 0, XDownYRight, XDownYLeft, XUpYLeft, XUpYRight };

struct PolynomialCamera {

    ///\brief 主距 mm
    double f = 0.0;

    ///\brief 像素大小 mm
    double pixel = 0.0;

    ///\brief 像主点偏移 mm
    double x0 = 0.0;

    ///\brief 影像列数
    int width = 0;

    ///\brief 影像行数
    int height = 0;

    ///\brief 不同 eo 多项式的阶数
    std::array<int, EO_COUNT> orders;

    ///\brief 外方位元素, 多项式的 y 是 io 坐标系下的，原点在中心，x 为右 y 为上
    std::array<VectorXd, EO_COUNT> eos;

    ///\brief 影像路径
    std::string filepath;

    ///\brief 地面分辨率
    double gsd = 0.0;

    /**
    * \brief 把像素坐标转换成影像平面的毫米坐标
    *
    * xmm = io(0, 0) * p + io(0, 1) * l + io(0, 2)
    * ymm = io(1, 0) * p + io(1, 1) * l + io(1, 2)
    */
    Matrix23d io;

    /**
    * \brief 把毫米坐标转换成像素坐标
    */
    Matrix23d iob;

    std::string planet;

    std::string name;

    ///\brief 设定了主距、像素大小、主点就可以计算 io 了
    void init_io_param();

    /**
     * \brief 获取摄影测量系统的 EO 信息的中心和旋转矩阵
     * \param y mm坐标系的坐标值
     */
    std::tuple<Vector3d, Matrix3d> get_eo_by_mm(double y) const;

    /**
     * \brief 获取摄影测量系统的 EO 信息的中心和 rodrigues 旋转角
     * \return [C, r]
     */
    std::tuple<Vector3d, Vector3d> get_Cr_by_mm(double y) const;

    ///\brief 获取当前行的 P 矩阵
    MatrixXd get_P_by_mm(double y) const;
    //获取当前行的 view 方向
    Vector3d get_view_direction(double y) const;

    ///\brief 像素坐标转换到 mm 坐标，需初始化 io，由于 io 中已经考虑到主点偏移，因此这里得到的就是无主点偏移的值
    Vector2d to_mm(const Vector2d &pixel) const;

    ///\brief mm 坐标转换到像素坐标，需初始化 iob
    Vector2d to_pixel(const Vector2d &mm) const;
};

/**
 * \brief 保存多项式信息倒文件
 * \param path
 * \param camera
 */
void save_poly_camera(const std::string &path, const PolynomialCamera &camera);

/**
 * \brief 从文件读取多项式信息，会同时读取阶数
 * \param path
 */
PolynomialCamera load_poly_camera(const std::string &path);

/**
* \brief 根据旋转矩阵和中心，拟合外方位元素
* \param C 中心，单位米，地心坐标系
* \param R 旋转矩阵
* \param type 像平面模型，根据此参数决定一个旋转矩阵，旋转到XRightYUp的坐标
*/
void fit_polynomial(PolynomialCamera &camera, std::vector<Vector3d> &C, const std::vector<Matrix3d> &R,
                    CameraCoordinate type);

typedef std::unordered_map<iid_t, PolynomialCamera> PolynomialCameras;
}
