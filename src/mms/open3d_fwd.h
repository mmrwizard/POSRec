/*
 * Created Date: Friday, January 15th 2021, 5:24:37 pm
 * Author: Han
 * 定义 Open3D 类型
 * Copyright (c) 2021 Vrlab
 */
#pragma once

#include <base/common.h>

// forward declaration
namespace open3d {
namespace geometry {
class PointCloud;
} // namespace geometry
namespace pipelines {
namespace registration {
class Feature;
} // namespace registration
} // namespace pipelines
} // namespace open3d

namespace h2o {
using PointCloud = open3d::geometry::PointCloud;
using PointCloudPtr = std::shared_ptr<open3d::geometry::PointCloud>;
using PointIndices = std::vector<int>;
using PointFeature = open3d::pipelines::registration::Feature;
using PointFeaturePtr = std::shared_ptr<PointFeature>;

PointCloudPtr load_las(const std::string &path);
} // namespace h2o
