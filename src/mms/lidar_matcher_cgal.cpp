/*
 * Created Date: Saturday, January 9th 2021, 5:04:06 pm
 * Author: Han
 * 利用 CGAL 接口进行匹配
 * Copyright (c) 2021 Vrlab
 */

#define CGAL_LINKED_WITH_OPENGR
#include <CGAL/OpenGR/compute_registration_transformation.h> // 必须在前面，避免 glog 的冲突
#include <CGAL/OpenGR/register_point_sets.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/property_map.h>

#include "lidar_matcher_cgal.h"

namespace h2o {
using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Pwn = std::pair<Point_3, Vector_3>;
using Point_map = CGAL::First_of_pair_property_map<Pwn>;
using Normal_map = CGAL::Second_of_pair_property_map<Pwn>;
namespace params = CGAL::parameters;

Matrix4d cgal_match_opengr(const PointCloudPtr &source, const PointCloudPtr &target) {
    std::vector<Pwn> cgal_source(source->points_.size()), cgal_target(target->points_.size());
    for (int i = 0; i < cgal_source.size(); ++i)
        cgal_source[i] = {Point_3(source->points_[i].x(), source->points_[i].y(), source->points_[i].z()),
                          Vector_3(source->normals_[i].x(), source->normals_[i].y(), source->normals_[i].z())};

    for (int i = 0; i < cgal_target.size(); ++i)
        cgal_target[i] = {Point_3(target->points_[i].x(), target->points_[i].y(), target->points_[i].z()),
                          Vector_3(target->normals_[i].x(), target->normals_[i].y(), target->normals_[i].z())};

    std::pair<Kernel::Aff_transformation_3, double> res =
        CGAL::OpenGR::compute_registration_transformation(cgal_target, cgal_source,
                                                          params::point_map(Point_map())
                                                              .normal_map(Normal_map())
                                                              .number_of_samples(100)
                                                              .maximum_running_time(60)
                                                              .accuracy(0.3)
                                                              .overlap(1.0),
                                                          params::point_map(Point_map()).normal_map(Normal_map()));
    auto xform = res.first;
    Matrix4d t = Matrix4d::Identity();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j)
            t(i, j) = xform.m(i, j);

    return t;
}
} // namespace h2o
