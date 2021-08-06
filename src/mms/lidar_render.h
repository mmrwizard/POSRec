/*
 * Created Date: Friday, January 15th 2021, 4:42:25 pm
 * Author: Han
 * 把点云渲染到影像
 * Copyright (c) 2021 Vrlab
 */
#pragma once

#include "open3d_fwd.h"
#include "ladybug_stream.h"

namespace h2o {
class LiDARRender {
public:
    LiDARRender() {}
    void load_las(const std::string &path);

    cv::Mat draw_color(const LadybugImage&camera);

    PointCloudPtr pcd_;
    std::vector<double> gps_times_, intensitys_;
    std::vector<Eigen::Vector3d> points_;

protected:
    
    Vector3d offset_;
};
} // namespace h2o
