/*
 * Created Date: Friday, January 15th 2021, 4:42:25 pm
 * Author: Han
 * 把点云渲染到影像
 * Copyright (c) 2021 Vrlab
 */
#include "lidar_render.h"

#include <base/base.h>
#include <laszip/laszip_api.h>
#include <open3d/Open3D.h>
#include <open3d/visualization/utility/GLHelper.h>

namespace h2o {
void LiDARRender::load_las(const std::string &path) {
    {
        LasHeader header = pc_read_header(path);
        offset_ = header.offset;
    }

    laszip_POINTER lasreader;
    laszip_create(&lasreader);

    std::string local = string_utf8_to_fstream(path);
    laszip_BOOL compressed;
    laszip_open_reader(lasreader, local.c_str(), &compressed);

    laszip_header *lasheader;
    laszip_get_header_pointer(lasreader, &lasheader);
    laszip_I64 num_points = (lasheader->number_of_point_records ? lasheader->number_of_point_records
                                                                : lasheader->extended_number_of_point_records);
    laszip_point *point;
    laszip_get_point_pointer(lasreader, &point);

    PointCloudPtr pcd = std::make_shared<open3d::geometry::PointCloud>();
    pcd->points_.resize(num_points);
    pcd->colors_.resize(num_points);

    for (int i = 0; i < num_points; ++i) {
        if (laszip_read_point(lasreader)) {
            CHECK(false) << "Read point error!";
        }
        double x = (double)point->X * lasheader->x_scale_factor + lasheader->x_offset;
        double y = (double)point->Y * lasheader->y_scale_factor + lasheader->y_offset;
        double z = (double)point->Z * lasheader->z_scale_factor + lasheader->z_offset;

        x -= offset_.x();
        y -= offset_.y();
        z -= offset_.z();

        double intensity = point->intensity / 65535.0;

        pcd->points_[i] = Vector3d(x, y, z);
        pcd->colors_[i] = Vector3d(intensity, intensity, intensity);
    }

    laszip_close_reader(lasreader);
    laszip_destroy(lasreader);

    pcd_ = pcd;
}

cv::Mat LiDARRender::draw_color(const LadybugImage &camera) {
    open3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow("", camera.height, camera.height, 0, 0, false);
    vis.AddGeometry(pcd_);

    auto &view_control = vis.GetViewControl();

    auto [q, t] = camera.get_extrinsic();

    // 这里用的旋转是逆矩阵
    // Xw = RXc + T => Xc = Rt(Xw - T)
    // 也就是 Xc = R(Xw - C)
    Matrix3d R = q.conjugate().toRotationMatrix();
    Vector3d dir = R.transpose() * Vector3d(-1, 0, 0);
    dir.normalize();

    // used to compute the view matrix in ogl
    /* The up direction is (0, -1, 0) in object space
     * ------->
     * |
     * |
     * \/
     */
    Vector3d eye = t - offset_;
    LOG(INFO) << eye;
    Vector3d target = eye + dir * 10.0;
    Vector3d up = R.transpose() * Vector3d(0, 0, 1);
    LOG(INFO) << up;

    view_control.ChangeWindowSize(camera.height, camera.height);
    view_control.ChangeFieldOfView(90.0);

    // PW = RPc + t
    Eigen::Matrix3d O1 = Eigen::Quaterniond::FromTwoVectors(Vector3d(0, 0, 1), Vector3d(0, -1, 0)).toRotationMatrix();
    Eigen::Matrix3d O2 = Eigen::Quaterniond::FromTwoVectors(Vector3d(1, 0, 0), Vector3d(0, 1, 0)).toRotationMatrix();

    Matrix4d M_ctow = Matrix4d::Identity();
    M_ctow.block<3, 3>(0, 0) = q.toRotationMatrix() * O2 * O1.transpose();
    M_ctow.block<3, 1>(0, 3) = t - offset_;

    open3d::camera::PinholeCameraParameters pinhole;
    view_control.ConvertToPinholeCameraParameters(pinhole);
    pinhole.extrinsic_ = M_ctow.inverse();
    // pinhole.extrinsic_.block<3, 3>(0, 0) *= O;
    view_control.ConvertFromPinholeCameraParameters(pinhole);

    vis.CaptureScreenImage("test.jpg");

    return cv::Mat();
}
} // namespace h2o
