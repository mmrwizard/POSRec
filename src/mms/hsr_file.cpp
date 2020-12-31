/*
 * @Author: Han Hu
 * @Date: 2020-12-18 21:27:51
 * @Description: 读取 HSR 文件，结合 POS 输出信息
 */
#include "hsr_file.h"

#include <HsRawiLSP.h>
#include <base/base.h>
#include <base/json.h>
#include <fstream>
#include <laszip/laszip_api.h>

namespace h2o {
HsrFile::HsrFileConfig HsrFile::load_config(const string &path) {
    HsrFile::HsrFileConfig config;

    nlohmann::json j = read_json_file(path);
    if (!j.count("HsrFileConfig")) {
        return config;
    }

    j = j["HsrFileConfig"];
    if (j.count("Speed")) {
        config.speed = j["Speed"];
    }
    if (j.count("RangeTime")) {
        config.rtime = j["RangeTime"].get<std::vector<double>>();
    }
    if (j.count("RangeDist")) {
        config.rdist = j["RangeDist"].get<std::vector<double>>();
    }
    if (j.count("WKT")) {
        config.wkt = j["WKT"].get<std::string>();
    }

    return config;
}

HsrFile::HsrFile(const std::string &path) : path_(path) {
    header.resize(1024);
    {
        std::ifstream ifile(string_utf8_to_fstream(path), std::ios::binary);
        ifile.read((char *)header.data(), 1024);
    }

    hsfile_ = std::make_shared<HsRawiLSP>();

    hsfile_->readFileHeader(header.data());
    hsfile_->setFilter(false);

    R_body = Matrix3d::Identity();
    t_body = Vector3d::Zero();
}

HsrFile::~HsrFile() { hsfile_->closeFile(); }

std::tm HsrFile::get_file_date() {
    TIME hstime;
    hsfile_->getFileTime(hstime);

    std::tm cpptm;
    cpptm.tm_year = hstime.Year - 1900;
    cpptm.tm_mon = hstime.Month - 1;
    cpptm.tm_mday = hstime.Day - 1;
    cpptm.tm_hour = hstime.Hour;
    cpptm.tm_min = hstime.Minute;
    cpptm.tm_sec = hstime.Second;

    return cpptm;
}

void HsrFile::load() {
    CHECK(hsfile_->openFile(string_utf8_to_fstream(path_)));

    std::vector<std::vector<HSPoint>> hspoints;

    while (!hsfile_->getRingData(hspoints)) {
        for (int i = 0; i < hspoints.size(); ++i) {
            for (int j = 0; j < hspoints[i].size(); ++j) {
                double gpstime = hspoints[i][j].lfGpsSec;
                double range = hspoints[i][j].lfRange;

                if (!is_point_good(range, gpstime)) {
                    continue;
                }

                points_.push_back(Vector3f(hspoints[i][j].pt.x, hspoints[i][j].pt.y, hspoints[i][j].pt.z));
                intens_.push_back(hspoints[i][j].nReflection);
                gpssecs_.push_back(hspoints[i][j].lfGpsSec);
            }
        }
    }

    double clip_min, clip_max;
    clip_min = percentile(intens_, 5);
    clip_max = percentile(intens_, 95);

    clip_min = std::max(0.0, clip_min);
    clip_max = std::min(clip_max, 65535.0);

    double scale = 1.0 / (clip_max - clip_min);
    for (int i = 0; i < intens_.size(); ++i) {
        double value = (intens_[i] - clip_min) * scale;
        value = clip<double>(value * 65535, 0, 65535);
        intens_[i] = floor(value);
    }

    hsfile_->closeFile();
}

void HsrFile::load_with_pos(const Pos &pos) {
    CHECK(hsfile_->openFile(string_utf8_to_fstream(path_)));

    std::vector<std::vector<HSPoint>> hspoints;

    while (!hsfile_->getRingData(hspoints)) {
        for (int i = 0; i < hspoints.size(); ++i) {
            for (int j = 0; j < hspoints[i].size(); ++j) {
                double gpstime = hspoints[i][j].lfGpsSec;
                double range = hspoints[i][j].lfRange;
                double speed = pos.get_speed(gpstime);

                if (!is_point_good(range, gpstime, speed)) {
                    continue;
                }

                gpssecs_.push_back(hspoints[i][j].lfGpsSec);
                points_.push_back(Vector3f(hspoints[i][j].pt.x, hspoints[i][j].pt.y, hspoints[i][j].pt.z));
                intens_.push_back(hspoints[i][j].nReflection);
            }
        }
    }

    double clip_min, clip_max;
    clip_min = percentile(intens_, 5);
    clip_max = percentile(intens_, 95);

    clip_min = std::max(0.0, clip_min);
    clip_max = std::min(clip_max, 65535.0);

    double scale = 1.0 / (clip_max - clip_min);
    for (int i = 0; i < intens_.size(); ++i) {
        double value = (intens_[i] - clip_min) * scale;
        value = clip<double>(value * 65535, 0, 65535);
        intens_[i] = floor(value);
    }

    hsfile_->closeFile();
}

void HsrFile::export_las(const std::string &path, const Pos &pos) const {
    BoundingBox3d bounds = pos.get_bounds();
    Vector3d offset = pos.get_position_offset();

    laszip_POINTER writer = create_lasfile_header(path, offset, bounds);
    laszip_point *laspoint;
    if (laszip_get_point_pointer(writer, &laspoint)) {
        CHECK(false) << "Create lasfile failed";
    }

    constexpr int NSEG = 1e7;
    for (int i = 0; i < points_.size(); i += NSEG) {
        int nsegments = std::min(NSEG, (int)points_.size() - i);

        std::vector<Vector3d> points(nsegments);
        std::vector<uint16_t> intensities(nsegments);
        std::vector<double> gpss(nsegments);

        for (int p = 0; p < nsegments; ++p) {
            double t = gpssecs_[i + p];
            auto [t_pos, R_pos] = pos.getd(t);

            Vector3d point = points_[i + p].cast<double>();
            uint16_t intensity = intens_[i + p];

            point = R_pos * (R_body * point + t_body) + t_pos;
            point += offset;

            points[p] = point;
            intensities[p] = intensity;
            gpss[p] = t;

            bounds.extend(points[p]);
        }

        for (int p = 0; p < nsegments; ++p) {
            laszip_set_coordinates(writer, points[p].data());
            laspoint->classification = 0;
            laspoint->extended_return_number = 1;
            laspoint->extended_number_of_returns = 1;
            laspoint->gps_time = gpss[p];
            laspoint->intensity = intensities[p];

            if (laszip_update_inventory(writer)) {
                CHECK(false) << "Write point failed!";
            }

            if (laszip_write_point(writer)) {
                CHECK(false) << "Write point failed!";
            }
        }
    }

    laszip_close_writer(writer);
    laszip_destroy(writer);
}

void HsrFile::export_las_ooc(const string &path, const Pos &pos) const {
    Vector2d rintensity = get_intensity_range();
    double scale = 1.0 / rintensity.norm();

    CHECK(hsfile_->openFile(string_utf8_to_fstream(path_)));

    std::vector<std::vector<HSPoint>> hspoints;

    BoundingBox3d bounds = pos.get_bounds();
    Vector3d offset = pos.get_position_offset();

    laszip_POINTER writer = create_lasfile_header(path, offset, bounds);
    laszip_point *laspoint;
    if (laszip_get_point_pointer(writer, &laspoint)) {
        CHECK(false) << "Create lasfile failed";
    }

    while (!hsfile_->getRingData(hspoints)) {
        std::vector<Vector3d> points;
        std::vector<uint16_t> intensities;
        std::vector<double> gpss;

        for (int i = 0; i < hspoints.size(); ++i) {
            for (int j = 0; j < hspoints[i].size(); ++j) {
                double gpstime = hspoints[i][j].lfGpsSec;
                double range = hspoints[i][j].lfRange;
                double speed = pos.get_speed(gpstime);

                if (!is_point_good(range, gpstime, speed)) {
                    continue;
                }

                auto [t_pos, R_pos] = pos.getd(gpstime);

                Vector3d point(hspoints[i][j].pt.x, hspoints[i][j].pt.y, hspoints[i][j].pt.z);
                uint16_t intensity = hspoints[i][j].nReflection;

                point = R_pos * (R_body * point + t_body) + t_pos;
                point += offset;

                double value = (intensity - rintensity.x()) * scale;
                value = clip<double>(value * 65535, 0, 65535);
                intensity = floor(value);

                gpss.push_back(gpstime);
                points.push_back(point);
                intensities.push_back(intensity);
            }
        }

        for (int p = 0; p < points.size(); ++p) {
            laszip_set_coordinates(writer, points[p].data());
            laspoint->classification = 0;
            laspoint->extended_return_number = 1;
            laspoint->extended_number_of_returns = 1;
            laspoint->gps_time = gpss[p];
            laspoint->intensity = intensities[p];

            if (laszip_update_inventory(writer)) {
                CHECK(false) << "Write point failed!";
            }

            if (laszip_write_point(writer)) {
                CHECK(false) << "Write point failed!";
            }
        }
    }

    hsfile_->closeFile();
    laszip_close_writer(writer);
    laszip_destroy(writer);
}

void HsrFile::export_colored_las_ooc(const std::string &path, const Pos &pos, const LadybugStreamPtr &ladybug) const {
    std::vector<LadybugImage> images = ladybug->get_image_infos();
    if (images.empty()) {
        return;
    }
    // 存储每个影像包含的点云影像时间，往后面内插半个曝光间隔
    std::vector<double> timestamps(images.size());
    for (int i = 0; i < images.size(); ++i) {
        double t0 = images[i].time;
        double t1 = i == images.size() - 1 ? images.back().time + 100 : images[i + 1].time;
        timestamps[i] = (t0 + t1) / 2.0;
    }

    Vector2d rintensity = get_intensity_range();
    double scale = 1.0 / rintensity.norm();

    CHECK(hsfile_->openFile(string_utf8_to_fstream(path_)));

    BoundingBox3d bounds = pos.get_bounds();
    Vector3d offset = pos.get_position_offset();

    laszip_POINTER writer = create_lasfile_header(path, offset, bounds);
    laszip_point *laspoint;
    if (laszip_get_point_pointer(writer, &laspoint)) {
        CHECK(false) << "Create lasfile failed";
    }

    for (int index = 0; index < timestamps.size(); ++index) {
        // 缓存前后
        std::vector<uint32_t> caches;
        for (int offset : {0, 1, -1, 2, -2}) {
            int current = offset + index;
            if (current >= 0 && current < timestamps.size()) {
                caches.push_back(uint32_t(current));
            }
        }

        ladybug->cache_images(caches);

        std::vector<std::vector<HSPoint>> hspoints;
        bool exceed_time = false;

        while (!hsfile_->getRingData(hspoints)) {
            std::vector<Vector3d> points;
            std::vector<uint16_t> intensities;
            std::vector<double> gpss;
            std::vector<Vector3b> colors;

            for (const auto &ring : hspoints) {
                for (const auto &hspoint : ring) {
                    double gpstime = hspoint.lfGpsSec;
                    double range = hspoint.lfRange;
                    double speed = pos.get_speed(gpstime);

                    if (gpstime > timestamps[index]) {
                        exceed_time = true;
                    }

                    if (!is_point_good(range, gpstime, speed)) {
                        continue;
                    }

                    auto [t_pos, R_pos] = pos.getd(gpstime);

                    Vector3d point(hspoint.pt.x, hspoint.pt.y, hspoint.pt.z);
                    uint16_t intensity = hspoint.nReflection;

                    point = R_pos * (R_body * point + t_body) + t_pos;
                    point += offset;

                    double value = (intensity - rintensity.x()) * scale;
                    value = clip<double>(value * 65535, 0, 65535);
                    intensity = floor(value);

                    Vector3b color = ladybug->colorize_on_images(point, caches);

                    gpss.push_back(gpstime);
                    points.push_back(point);
                    intensities.push_back(intensity);
                    colors.push_back(color);
                }
            }

            for (int p = 0; p < points.size(); ++p) {
                laszip_set_coordinates(writer, points[p].data());
                laspoint->classification = 0;
                laspoint->extended_return_number = 1;
                laspoint->extended_number_of_returns = 1;
                laspoint->gps_time = gpss[p];
                laspoint->intensity = intensities[p];

                laspoint->rgb[0] = colors[p].x();
                laspoint->rgb[1] = colors[p].y();
                laspoint->rgb[2] = colors[p].z();

                if (laszip_update_inventory(writer)) {
                    CHECK(false) << "Write point failed!";
                }

                if (laszip_write_point(writer)) {
                    CHECK(false) << "Write point failed!";
                }
            }

            if (exceed_time) {
                break;
            }
        }
    }

    hsfile_->closeFile();
    laszip_close_writer(writer);
    laszip_destroy(writer);
}

void HsrFile::set_body(const Vector3d &r, const Vector3d &t) {
    R_body = Pos::ypr_to_rotation(r(0), r(1), r(2));
    t_body = t;
}

void HsrFile::set_body(const std::string &config) {
    nlohmann::json j = read_json_file(config);
    if (!j.count("LiDAR")) {
        return;
    }
    j = j["LiDAR"];

    Vector3d r;
    Vector3d t;
    t(0) = j["X"];
    t(1) = j["Y"];
    t(2) = j["Z"];

    r(0) = j["Yaw"].get<double>() * DEG2RAD;
    r(1) = j["Pitch"].get<double>() * DEG2RAD;
    r(2) = j["Roll"].get<double>() * DEG2RAD;

    set_body(r, t);
}

void HsrFile::set_config(const std::string &path) {
    auto config = HsrFile::load_config(path);
    this->set_config(config);
}

laszip_POINTER HsrFile::create_lasfile_header(const string &path, const Vector3d &offset,
                                              const BoundingBox3d &bounds) const {
    tm local_tm;
    {
        using namespace std::chrono;
        auto now = std::chrono::system_clock::now();
        time_t tt = system_clock::to_time_t(now);
        local_tm = *localtime(&tt);
    }

    laszip_POINTER writer;
    laszip_create(&writer);

    laszip_BOOL compress = (strstr(path.c_str(), ".laz") != 0);

    laszip_header *header;
    laszip_get_header_pointer(writer, &header);

    header->file_source_ID = 4711;
    header->global_encoding = (1 << 0) | (1 << 4); // see LAS specification for details
    header->version_major = 1;
    header->version_minor = 4;
    strncpy(header->system_identifier, "H2O MMS", 32);
    strncpy(header->generating_software, "H2O MMSConvert", 32);

    header->file_creation_day = local_tm.tm_yday;
    header->file_creation_year = local_tm.tm_year + 1900;
    header->header_size = 375;
    header->offset_to_point_data = 375;
    header->point_data_format = 3;
    header->point_data_record_length = 34;
    header->number_of_point_records = 0;
    for (int i = 0; i < 5; ++i) {
        header->number_of_points_by_return[i] = 0;
    }
    header->extended_number_of_point_records = 0;
    header->extended_number_of_points_by_return[0] = 0;

    header->x_scale_factor = 0.001;
    header->y_scale_factor = 0.001;
    header->z_scale_factor = 0.001;

    header->x_offset = offset.x();
    header->y_offset = offset.y();
    header->z_offset = offset.z();

    header->min_x = bounds.min().x();
    header->min_y = bounds.min().y();
    header->min_z = bounds.min().z();
    header->max_x = bounds.max().x();
    header->max_y = bounds.max().y();
    header->max_z = bounds.max().z();

    laszip_auto_offset(writer);

    if (!config_.wkt.empty()) {
        CHECK(config_.wkt.size() < 2048);
        uint8_t *data = new uint8_t[2048];
        memcpy(data, config_.wkt.c_str(), config_.wkt.size());
        data[config_.wkt.size() + 1] = '\0';
        if (laszip_add_vlr(writer, "LASF_Projection", 2112, config_.wkt.size() + 1, "OGC Transformation Record",
                           data)) {
            char *err;
            laszip_get_error(writer, &err);
            CHECK(false) << err;
        }
    }

    if (laszip_open_writer(writer, string_utf8_to_fstream(path).c_str(), compress)) {
        char *err;
        laszip_get_error(writer, &err);
        CHECK(false) << err;
    };

    return writer;
}

Vector2d HsrFile::get_intensity_range() const {
    CHECK(hsfile_->openFile(string_utf8_to_fstream(path_)));

    std::vector<double> values;

    std::vector<std::vector<HSPoint>> hspoints;
    constexpr int num_sufficient = 1e6; // 100 万点

    while (!hsfile_->getRingData(hspoints)) {
        for (int i = 0; i < hspoints.size(); ++i) {
            for (int j = 0; j < hspoints[i].size(); ++j) {
                values.push_back(hspoints[i][j].nReflection);
            }
        }
        if (values.size() > num_sufficient) {
            break;
        }
    }

    double clip_min, clip_max;
    clip_min = percentile(values, 5);
    clip_max = percentile(values, 95);

    clip_min = std::max(0.0, clip_min);
    clip_max = std::min(clip_max, 65535.0);

    hsfile_->closeFile();
    return Vector2d(clip_min, clip_max);
}

bool HsrFile::is_point_good(double range, double gpstime, double speed) const {
    if (config_.rtime.size() == 2 && (gpstime < config_.rtime[0] || gpstime > config_.rtime[1])) {
        return false;
    }

    if (config_.rdist.size() == 2 && (range < config_.rdist[0] || range > config_.rdist[1])) {
        return false;
    }

    if (speed < config_.speed) {
        return false;
    }

    return true;
}

std::tuple<Quaterniond, Vector3d> HsrFile::read_lidar_body(const std::string &path) {
    nlohmann::json j = read_json_file(path);
    if (!j.count("LiDAR")) {
        return std::make_tuple(Quaterniond::Identity(), Vector3d::Zero());
    }
    j = j["LiDAR"];

    Vector3d r;
    Vector3d t;
    t(0) = j["X"];
    t(1) = j["Y"];
    t(2) = j["Z"];

    r(0) = j["Yaw"].get<double>() * DEG2RAD;
    r(1) = j["Pitch"].get<double>() * DEG2RAD;
    r(2) = j["Roll"].get<double>() * DEG2RAD;

    Matrix3d R = Pos::ypr_to_rotation(r(0), r(1), r(2));
    Quaterniond q(R);

    return std::make_tuple(q, t);
}

} // namespace h2o
