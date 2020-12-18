/*!
 * \file camera_rpc.h
 *
 * \author Han
 * \date 2017/05/04
 *
 * RPC 相机模型
 */
#pragma once
#include <base/common.h>
#include <base/geo_image.h>

#include <sat/camera_polynomial.h>

namespace h2o {

struct RpcCoefficient {
    double line_off;
    double samp_off;
    double lat_off;
    double long_off;
    double height_off;
    double line_scale;
    double samp_scale;
    double lat_scale;
    double long_scale;
    double height_scale;

    VectorXd line_num_coeff;
    VectorXd line_den_coeff;
    VectorXd samp_num_coeff;
    VectorXd samp_den_coeff;
};

///\brief 从文件中读取 rpc 参数，通常为 影像名_rpc.txt 的格式
RpcCoefficient load_rpc(const std::string &rpcpath);

///\brief 保存 rpc 参数到文件，通常为 影像名_rpc.txt 的格式
void save_rpc(const std::string &rpcpath, const RpcCoefficient &rpc);

///\brief RPC 相机结构
struct RpcCamera {
    RpcCamera();
    void init_inverse_transform();
    // 根据四个角点位置在 投影坐标系下的长度，计算平均gsd
    double compute_gsd();

    Vector2d pixel_to_latlon(const Vector2d &pixel) const;
    Vector2d latlon_to_pixel(const Vector2d &latlon) const;

    RpcCoefficient rpc;  // 正向RPC，Y = c.dot(u) / d.dot(u), X = e.dot(u) / f.dot(u)
    RpcCoefficient rpci; // 逆向RPC，P = c.dot(u) / d.dot(u), L = e.dot(u) / f.dot(u)
    ///\brief 从三维点投影到影像后用这个参数纠正
    Matrix23d rpc_aff;
    Matrix23d rpc_aff_inv;

    int width = 0;
    int height = 0;
    double gsd = 0.0;

    std::string name;
    std::string filepath;
    std::string planet;
};

/**
 * \brief 保存 rpc 相机信息，其中 rpc 参数单独保存一个 name_rpc.txt 的文件
 */
void save_rpc_camera(const std::string &filepath, const RpcCamera &camera);

///\brief 读取 rpc 相机信息，需要确保 rpc 参数单独保存在一个文件中
RpcCamera load_rpc_camera(const std::string &filepath);

/**
 * \brief 根据多项式模型拟合RPC参数
 * \param campoly 多项式模型，坐标系为地心坐标系
 * \param dem 坐标系为经纬度坐标系其中，经度为 x 方向
 * \param planet 决定椭球体，在地球上采用 WGS-84，其他均采用球体
 */
RpcCamera fit_rpc(const PolynomialCamera &campoly, const GeoImage &dem);
// 清除仿射变换参数
RpcCamera refit_rpc(const RpcCamera &camera);
// 计算RPC变换的逆变换
void compute_reverse_rpc(RpcCamera &camera);

typedef std::unordered_map<iid_t, RpcCamera> RpcCameras;
} // namespace h2o
