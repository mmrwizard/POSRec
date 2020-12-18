/*
 * Filename: /home/han/Repo/LInSAR/src/base/pcd.cpp
 * Path: /home/han/Repo/LInSAR/src/base
 * Created Date: Saturday, December 7th 2019, 10:44:09 pm
 * Author: han
 *
 * support for point clouds operations
 */

#include <base/base.h>
#include <base/pcd.h>

#include <laszip/laszip_api.h>

#include <chrono>
#include <ctime>

namespace h2o {

LasHeader pc_read_header(const std::string &path) {
    h2o::LasHeader header;

    laszip_POINTER lasreader;
    laszip_create(&lasreader);

    std::string local = string_utf8_to_fstream(path);
    laszip_BOOL compressed;
    laszip_open_reader(lasreader, local.c_str(), &compressed);

    laszip_header *lasheader;
    laszip_get_header_pointer(lasreader, &lasheader);

    header.num_points =
        std::max((uint32_t)lasheader->number_of_point_records, (uint32_t)lasheader->extended_number_of_point_records);
    header.offset.x() = lasheader->x_offset;
    header.offset.y() = lasheader->y_offset;
    header.offset.z() = lasheader->z_offset;
    header.bbox.min().x() = lasheader->min_x;
    header.bbox.min().y() = lasheader->min_y;
    header.bbox.min().z() = lasheader->min_z;
    header.bbox.max().x() = lasheader->max_x;
    header.bbox.max().y() = lasheader->max_y;
    header.bbox.max().z() = lasheader->max_z;

    int nvlr =
        std::max(lasheader->number_of_variable_length_records, lasheader->number_of_extended_variable_length_records);
    for (int i = 0; i < nvlr; ++i) {
        laszip_vlr_struct vlr = lasheader->vlrs[i];
        if (strcmp(vlr.user_id, "LASF_Projection") == 0 && vlr.record_id == 2112) {
            header.wkt = (char *)vlr.data;
            break;
        }
    }

    laszip_close_reader(lasreader);
    laszip_destroy(lasreader);
    return header;
}

Vector3d pc_read_offset(const std::string &path) {
    Vector3d offset;

    laszip_POINTER lasreader;
    laszip_create(&lasreader);

    std::string local = string_utf8_to_fstream(path);
    laszip_BOOL compressed;
    laszip_open_reader(lasreader, local.c_str(), &compressed);

    laszip_header *lasheader;
    laszip_get_header_pointer(lasreader, &lasheader);

    offset.x() = lasheader->min_x;
    offset.y() = lasheader->min_y;
    offset.z() = lasheader->min_z;

    laszip_close_reader(lasreader);
    laszip_destroy(lasreader);

    return offset;
}

MatrixXf pc_read(const std::string &path, const Vector3d &offset /*= Vector3d::Zero()*/) {

    laszip_POINTER lasreader;
    laszip_create(&lasreader);

    std::string local = string_utf8_to_fstream(path);
    laszip_BOOL compressed;
    laszip_open_reader(lasreader, local.c_str(), &compressed);

    laszip_header *lasheader;
    laszip_get_header_pointer(lasreader, &lasheader);
    laszip_I64 num_points = (lasheader->number_of_point_records ? lasheader->number_of_point_records
                                                                : lasheader->extended_number_of_point_records);
    MatrixXf points(3, num_points);
    laszip_point *point;
    laszip_get_point_pointer(lasreader, &point);

    for (int i = 0; i < num_points; ++i) {
        if (laszip_read_point(lasreader)) {
            CHECK(false) << "Read point error!";
        }
        double x = (double)point->X * lasheader->x_scale_factor + lasheader->x_offset;
        double y = (double)point->Y * lasheader->y_scale_factor + lasheader->y_offset;
        double z = (double)point->Z * lasheader->z_scale_factor + lasheader->z_offset;

        x -= offset.x();
        y -= offset.y();
        z -= offset.z();

        points(0, i) = x;
        points(1, i) = y;
        points(2, i) = z;
    }

    laszip_close_reader(lasreader);
    laszip_destroy(lasreader);

    return points;
}

MatrixXf pc_read_last_and_single(const std::string &path, const Vector3d &offset) {
    laszip_POINTER lasreader;
    laszip_create(&lasreader);

    std::string local = string_utf8_to_fstream(path);
    laszip_BOOL compressed;
    laszip_open_reader(lasreader, local.c_str(), &compressed);

    laszip_header *lasheader;
    laszip_get_header_pointer(lasreader, &lasheader);
    laszip_I64 num_points = (lasheader->number_of_point_records ? lasheader->number_of_point_records
                                                                : lasheader->extended_number_of_point_records);
    MatrixXf points(3, num_points);
    laszip_point *point;
    laszip_get_point_pointer(lasreader, &point);

    int index = 0;
    for (int i = 0; i < num_points; ++i) {
        if (laszip_read_point(lasreader)) {
            CHECK(false) << "Read point error!";
        }

        bool single = false;
        bool last = false;
        if (lasheader->version_minor == 4) {
            // single or last
            if (point->extended_return_number == point->extended_number_of_returns) {
                last = true;
            }
            if (point->extended_number_of_returns == 1) {
                single = true;
            }
        } else if (lasheader->version_minor < 4) {
            // single or last
            if (point->return_number == point->number_of_returns) {
                last = true;
            }
            if (point->number_of_returns == 1) {
                single = true;
            }
        }

        if (!single && !last) {
            continue;
        }

        double x = (double)point->X * lasheader->x_scale_factor + lasheader->x_offset;
        double y = (double)point->Y * lasheader->y_scale_factor + lasheader->y_offset;
        double z = (double)point->Z * lasheader->z_scale_factor + lasheader->z_offset;

        x -= offset.x();
        y -= offset.y();
        z -= offset.z();

        points(0, index) = x;
        points(1, index) = y;
        points(2, index) = z;
        index++;
    }

    laszip_close_reader(lasreader);
    laszip_destroy(lasreader);

    points.conservativeResize(3, index);

    return points;
}

void pc_save(const std::string &path, const MatrixXf &points, const Vector3d &offset /*= Vector3d::Zero()*/,
             const std::string &wkt) {
    tm local_tm;
    {
        using namespace std::chrono;
        auto now = std::chrono::system_clock::now();
        time_t tt = system_clock::to_time_t(now);
        local_tm = *localtime(&tt);
    }

    BoundingBox3f bounds = pc_bounding_box(points);

    laszip_POINTER writer;
    laszip_create(&writer);

    laszip_BOOL compress = (strstr(path.c_str(), ".laz") != 0);

    laszip_header *header;
    laszip_get_header_pointer(writer, &header);

    header->file_source_ID = 4711;
    header->global_encoding = (1 << 0) | (1 << 4); // see LAS specification for details
    header->version_major = 1;
    header->version_minor = 4;
    strncpy(header->system_identifier, "LInSAR", 32);
    header->file_creation_day = local_tm.tm_yday;
    header->file_creation_year = local_tm.tm_year + 1900;
    header->header_size = 375;
    header->offset_to_point_data = 375;
    header->point_data_format = 6;
    header->point_data_record_length = 30;
    header->number_of_point_records = 0;
    for (int i = 0; i < 5; ++i) {
        header->number_of_points_by_return[i] = 0;
    }
    header->extended_number_of_point_records = points.cols();
    header->extended_number_of_points_by_return[0] = points.cols();
    header->x_scale_factor = 0.01;
    header->y_scale_factor = 0.01;
    header->z_scale_factor = 0.01;
    header->x_offset = offset.x();
    header->y_offset = offset.y();
    header->z_offset = offset.z();
    header->min_x = bounds.min().x() + offset.x();
    header->min_y = bounds.min().y() + offset.y();
    header->min_z = bounds.min().z() + offset.z();
    header->max_x = bounds.max().x() + offset.x();
    header->max_y = bounds.max().y() + offset.y();
    header->max_z = bounds.max().z() + offset.z();
    laszip_auto_offset(writer);

    if (!wkt.empty()) {
        CHECK(wkt.size() < 2048);
        uint8_t *data = new uint8_t[2048];
        memcpy(data, wkt.c_str(), wkt.size());
        data[wkt.size() + 1] = '\0';
        if (laszip_add_vlr(writer, "LASF_Projection", 2112, wkt.size() + 1, "OGC Transformation Record", data)) {
            char *err;
            laszip_get_error(writer, &err);
            CHECK(false) << err;
        }
    }

    laszip_BOOL request = 1;
    if (laszip_request_compatibility_mode(writer, request)) {
        char *err;
        laszip_get_error(writer, &err);
        CHECK(false) << err;
    }

    if (laszip_open_writer(writer, string_utf8_to_fstream(path).c_str(), compress)) {
        char *err;
        laszip_get_error(writer, &err);
        CHECK(false) << err;
    };

    laszip_point *point;
    if (laszip_get_point_pointer(writer, &point)) {
        char *err;
        laszip_get_error(writer, &err);
        CHECK(false) << err;
    }

    for (int i = 0; i < points.cols(); i++) {

        Vector3d p3d(points(0, i) + offset.x(), points(1, i) + offset.y(), points(2, i) + offset.z());

        laszip_set_coordinates(writer, p3d.data());
        point->classification = 0;
        point->extended_return_number = 1;
        point->extended_number_of_returns = 1;

        if (laszip_write_point(writer)) {
            CHECK(false) << "Write point failed!";
        }
    }

    laszip_close_writer(writer);
    laszip_destroy(writer);
}

GeoImagePtr pc_dem_interpolation(const MatrixXf &points, double gsd, bool allow_nodata, BoundingBox3f bb) {
    constexpr int K = 12;
    GeoImagePtr dem = std::make_shared<GeoImage>();
    KDTreeAdaptor kdtree(2, points);

    // 如果 bb 是无效值，则重新从点云计算
    if (bb.isEmpty()) {
        bb = pc_bounding_box(points);
    }

    int rows = bb.sizes().y() / gsd + 1;
    int cols = bb.sizes().x() / gsd + 1;

    Matrix23d xform;
    xform << gsd, 0.0, bb.min().x(), 0.0, -gsd, bb.max().y();
    dem->init_image(rows, cols, CV_32F, xform);

    float nodata = -9999.0f;
    // 至少5倍GSD内需要有点
    double max_neareast_distance = 25 * gsd * gsd;
    double regularization = gsd * gsd * 0.25;
    regularization = std::max(regularization, 1.0);
#pragma omp parallel for
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            Vector2d pixel(c, r);
            Vector2f projection = dem->pixel_to_projection(pixel).cast<float>();

            std::vector<Eigen::Index> indices(K);
            std::vector<float> squared_distances(K);
            kdtree.query(projection.data(), K, indices.data(), squared_distances.data());

            if (squared_distances[0] > max_neareast_distance && allow_nodata) {
                dem->set_point<float>(r, c, nodata);
            } else {
                VectorXd z(indices.size());
                VectorXd w(squared_distances.size());
                for (int i = 0; i < indices.size(); ++i) {
                    z(i) = points.col(indices[i])(2);
                    w(i) = 1.0 / (squared_distances[i] + regularization);
                }
                w = w / w.sum();
                double dem_z = w.dot(z);
                dem->set_point<float>(r, c, dem_z);
            }
        }
    }

    return dem;
}

void pc_dem_interpolation(const MatrixXf &points, GeoImagePtr &dem, const cv::Mat &mask /*= cv::Mat()*/) {
    bool has_mask = !mask.empty();
    const int K = 12;
    KDTreeAdaptor kdtree(2, points);
    double gsd = (std::abs(dem->get_xform()(0, 0)) + std::abs(dem->get_xform()(1, 1))) / 2.0;

    float regularization = gsd * gsd * 0.25;
    regularization = std::max(regularization, 1.0f);

    int rows = dem->get_image().rows;
    int cols = dem->get_image().cols;

#pragma omp parallel for
    for (int r = 0; r < rows; ++r) {
        const uint8_t *mask_data = nullptr;
        if (has_mask) {
            mask_data = mask.ptr<uint8_t>(r);
        }

        for (int c = 0; c < cols; ++c) {
            if (has_mask && mask_data[c]) {
                continue;
            }
            Vector2d pixel(c, r);
            Vector2f projection = dem->pixel_to_projection(pixel).cast<float>();

            std::vector<Eigen::Index> indices(K);
            std::vector<float> squared_distances(K);
            kdtree.query(projection.data(), K, indices.data(), squared_distances.data());

            VectorXf z(indices.size());
            VectorXf w(squared_distances.size());
            for (int i = 0; i < indices.size(); ++i) {
                z(i) = points(2, indices[i]);
                w(i) = 1.0 / (squared_distances[i] + regularization);
            }
            w = w / w.sum();
            double dem_z = w.dot(z);
            dem->set_point<float>(r, c, dem_z);
        }
    }
}

void pc_dem_interpolation_nodata(const MatrixXf &points, const cv::Mat &mask, GeoImage &dem) {
    const int K = 12;
    KDTreeAdaptor kdtree(2, points);
    double gsd = (std::abs(dem.get_xform()(0, 0)) + std::abs(dem.get_xform()(1, 1))) / 2.0;

    double max_neareast_distance = 25 * gsd * gsd;
    double regularization = gsd * gsd * 0.25;
    regularization = std::max(regularization, 1.0);

    int rows = dem.get_image().rows;
    int cols = dem.get_image().cols;

#pragma omp parallel for
    for (int r = 0; r < rows; ++r) {
        const uint8_t *mask_data = mask.ptr<uint8_t>(r);
        for (int c = 0; c < cols; ++c) {
            if (mask_data[c]) {
                continue;
            }
            Vector2d pixel(c, r);
            Vector2f projection = dem.pixel_to_projection(pixel).cast<float>();

            std::vector<Eigen::Index> indices(K);
            std::vector<float> squared_distances(K);
            kdtree.query(projection.data(), K, indices.data(), squared_distances.data());

            VectorXd z(indices.size());
            VectorXd w(squared_distances.size());
            for (int i = 0; i < indices.size(); ++i) {
                z(i) = points.col(indices[i])(2);
                w(i) = 1.0 / (squared_distances[i] + regularization);
            }
            w = w / w.sum();
            double dem_z = w.dot(z);
            dem.set_point<float>(r, c, dem_z);
        }
    }
}

void pc_dem_interpolation_nodata(const cv::Mat &mask, GeoImage &dem) {
    int num_points = cv::countNonZero(mask);
    if (num_points - mask.rows * mask.cols == 0) {
        return; // 没有无效值
    }
    MatrixXf points(3, num_points);
    int idx = 0;
    for (int r = 0; r < mask.rows; ++r) {
        const uint8_t *mask_ptr = mask.ptr<uint8_t>(r);
        const float *dem_ptr = dem.get_image().ptr<float>(r);
        for (int c = 0; c < mask.cols; ++c) {
            if (mask_ptr[c] == NoDataDem) {
                continue;
            }
            Vector2d pixel(c, r);
            pixel = dem.pixel_to_projection(pixel);
            points(0, idx) = pixel(0);
            points(1, idx) = pixel(1);
            points(2, idx) = dem_ptr[c];
            idx++;
        }
    }

    pc_dem_interpolation_nodata(points, mask, dem);
}

void pc_dem_interpolation_nodata(const cv::Mat &mask, cv::Mat &dem) {
    int num_points = cv::countNonZero(mask);
    if (num_points - mask.rows * mask.cols == 0) {
        return; // 没有无效值
    }
    MatrixXf points(3, num_points);
    int idx = 0;
    for (int r = 0; r < mask.rows; ++r) {
        const uint8_t *mask_ptr = mask.ptr<uint8_t>(r);
        const float *dem_ptr = dem.ptr<float>(r);
        for (int c = 0; c < mask.cols; ++c) {
            if (mask_ptr[c] == NoDataDem) {
                continue;
            }
            points(0, idx) = c;
            points(1, idx) = r;
            points(2, idx) = dem_ptr[c];
            idx++;
        }
    }

    const int K = 12;
    KDTreeAdaptor kdtree(2, points);
    double gsd = 1;

    double max_neareast_distance = 25 * gsd * gsd;
    double regularization = gsd * gsd * 0.25;
    regularization = std::max(regularization, 1.0);

    int rows = dem.rows;
    int cols = dem.cols;

#pragma omp parallel for
    for (int r = 0; r < rows; ++r) {
        const uint8_t *mask_data = mask.ptr<uint8_t>(r);
        for (int c = 0; c < cols; ++c) {
            if (mask_data[c]) {
                continue;
            }
            Vector2f pixel(c, r);

            std::vector<Eigen::Index> indices(K);
            std::vector<float> squared_distances(K);
            kdtree.query(pixel.data(), K, indices.data(), squared_distances.data());

            VectorXd z(indices.size());
            VectorXd w(squared_distances.size());
            for (int i = 0; i < indices.size(); ++i) {
                z(i) = points.col(indices[i])(2);
                w(i) = 1.0 / (squared_distances[i] + regularization);
            }
            w = w / w.sum();
            double dem_z = w.dot(z);
            dem.at<float>(r, c) = dem_z;
        }
    }
}

void pc_dem_interpolation_nodata(GeoImage &dem, const std::vector<double> &nodata_values) {
    // 更新 nodata values
    int rows = dem.get_image().rows;
    int cols = dem.get_image().cols;

    cv::Mat nodata_mask = cv::Mat(rows, cols, CV_8U, ValidDataDem);

    for (double nodata_value : nodata_values) {
        const cv::Mat &mat = dem.get_image();
        double threshold = std::abs(nodata_value) * 1e-6;
        threshold = std::max(1e-3, threshold);
        cv::Mat index = cv::abs(mat - nodata_value) < threshold;
        nodata_mask.setTo(NoDataDem, index);
    }

    {
        // 判断 NAN
        const cv::Mat &mat = dem.get_image();
        cv::Mat mask_nan = mat != mat;
        if (cv::countNonZero(mask_nan)) {
            nodata_mask.setTo(NoDataDem, mask_nan);
        }
    }

    int num_valid = cv::countNonZero(nodata_mask);
    CHECK(num_valid > 100);
    if (num_valid == nodata_mask.total()) {
        return;
    }

    // 创建点云
    MatrixXf points = MatrixXf::Zero(3, num_valid);
    int idx = 0;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (nodata_mask.at<uint8_t>(r, c) != NoDataDem) {
                Vector2d projection = dem.pixel_to_projection(Vector2d(c, r));
                float z = dem.get_point<float>(r, c);
                points.col(idx++) = Vector3f(projection(0), projection(1), z);
            }
        }
    }

    // 如果存在无效值
    if (num_valid != nodata_mask.total()) {
        pc_dem_interpolation_nodata(points, nodata_mask, dem);
    }
    dem.set_mask(nodata_mask);
}

BoundingBox3f pc_bounding_box(const MatrixXf &points, const std::vector<uint32_t> &indices) {
    CHECK(points.rows() == 3);
    BoundingBox3f bbox;

    bool has_indices = false;
    int npoints;
    if (indices.empty()) {
        npoints = points.cols();
        has_indices = false;
    } else {
        npoints = indices.size();
        has_indices = true;
    }

    for (int i = 0; i < npoints; ++i) {
        Vector3f point;
        if (has_indices) {
            point = points.col(indices[i]);
        } else {
            point = points.col(i);
        }
        bbox.extend(point);
    }

    return bbox;
}

std::vector<uint32_t> pc_remove_indices(const std::vector<uint32_t> &src, const std::vector<uint32_t> &removed) {
    std::vector<uint32_t> src_sorted = src;
    std::vector<uint32_t> removed_sorted = removed;

    std::sort(begin(src_sorted), end(src_sorted));
    std::sort(begin(removed_sorted), end(removed_sorted));

    std::vector<uint32_t> difference;
    difference.resize(src_sorted.size());
    auto end_iterator = std::set_difference(begin(src_sorted), end(src_sorted), begin(removed_sorted),
                                            end(removed_sorted), begin(difference));
    difference.erase(end_iterator, end(difference));

    return difference;
}
MatrixXf pc_extract_points(const MatrixXf &points, const std::vector<uint32_t> &indicies) {
    MatrixXf extracted(3, indicies.size());
    for (int i = 0; i < indicies.size(); ++i) {
        extracted.col(i) = points.col(indicies[i]);
    }
    return extracted;
}
} // namespace h2o
