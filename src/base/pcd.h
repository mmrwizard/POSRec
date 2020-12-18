/*
 * Filename: /home/han/Repo/LInSAR/src/base/pcd.h
 * Path: /home/han/Repo/LInSAR/src/base
 * Created Date: Saturday, December 7th 2019, 10:44:04 pm
 * Author: han
 *
 * support for point clouds operations
 */
#pragma once

#include <base/base.h>

namespace h2o {

// Las IO functions

struct LasHeader {
    int num_points;
    Vector3d offset;
    BoundingBox3d bbox;
    std::string wkt;
};

LasHeader pc_read_header(const std::string &path);
Vector3d pc_read_offset(const std::string &path);
MatrixXf pc_read(const std::string &path, const Vector3d &offset = Vector3d::Zero());
MatrixXf pc_read_last_and_single(const std::string &path, const Vector3d &offset = Vector3d::Zero());
void pc_save(const std::string &path, const MatrixXf &points, const Vector3d &offset = Vector3d::Zero(),
             const std::string &wkt = "");

// Interpolation functions

GeoImagePtr pc_dem_interpolation(const MatrixXf &points, double gsd, bool allow_nodata = true,
                                 BoundingBox3f bb = BoundingBox3f());
// 这里的mask 是内插无效值的地方，有效的不内插
void pc_dem_interpolation(const MatrixXf &points, GeoImagePtr &dem, const cv::Mat &mask = cv::Mat());
void pc_dem_interpolation_nodata(const MatrixXf &points, const cv::Mat &mask, GeoImage &dem);
void pc_dem_interpolation_nodata(const cv::Mat &mask, GeoImage &dem);
void pc_dem_interpolation_nodata(const cv::Mat &mask, cv::Mat &dem);
void pc_dem_interpolation_nodata(GeoImage &dem, const std::vector<double> &nodata_values);

// Other simple functions

BoundingBox3f pc_bounding_box(const MatrixXf &points, const std::vector<uint32_t> &indices = std::vector<uint32_t>());
std::vector<uint32_t> pc_remove_indices(const std::vector<uint32_t> &src, const std::vector<uint32_t> &removed);
MatrixXf pc_extract_points(const MatrixXf &points, const std::vector<uint32_t> &indicies);

} // namespace h2o
