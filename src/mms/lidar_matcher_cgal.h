/*
 * Created Date: Saturday, January 9th 2021, 5:04:06 pm
 * Author: Han
 * 利用 CGAL 接口进行匹配
 * Copyright (c) 2021 Vrlab
 */
#pragma once

#include "lidar_matcher_proxy.h"

namespace h2o {
// target = xform * source
Matrix4d cgal_match_opengr(const PointCloudPtr &source, const PointCloudPtr &target);
} // namespace h2o