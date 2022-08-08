#pragma once

#include <base/base.h>
#include <base/common.h>
#include <laszip/laszip_api.h>
#include <open3d/Open3D.h>
#include <open3d/visualization/utility/GLHelper.h>
#include "mms/open3d_fwd.h"

using namespace h2o;
namespace h2o {
    void load_las(const std::string &path) {
    { LasHeader header = pc_read_header(path); }

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

        double intensity = point->intensity / 65535.0;

        pcd->points_[i] = Vector3d(x, y, z);
        pcd->colors_[i] = Vector3d(intensity, intensity, intensity);
    }

    laszip_close_reader(lasreader);
    laszip_destroy(lasreader);
    
    }
} // namespace h2o