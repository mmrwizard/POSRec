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
    cpptm.tm_hour = hstime.Hour - 1;
    cpptm.tm_min = hstime.Minute - 1;
    cpptm.tm_sec = hstime.Second - 1;

    return cpptm;
}

void HsrFile::load() {
    CHECK(hsfile_->openFile(string_utf8_to_fstream(path_)));

    std::vector<std::vector<HSPoint>> hspoints;

    while (!hsfile_->getRingData(hspoints)) {
        for (int i = 0; i < hspoints.size(); ++i) {
            for (int j = 0; j < hspoints[i].size(); ++j) {
                points_.push_back(Vector3f(hspoints[i][j].pt.x, hspoints[i][j].pt.y, hspoints[i][j].pt.z));
                ranges_.push_back(hspoints[i][j].lfRange);
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

void HsrFile::load_pos_filtered(const Pos &pos, double t_speed, const std::vector<double> &rtime,
                                const std::vector<double> &rdist) {
    CHECK(hsfile_->openFile(string_utf8_to_fstream(path_)));

    std::vector<std::vector<HSPoint>> hspoints;

    while (!hsfile_->getRingData(hspoints)) {
        for (int i = 0; i < hspoints.size(); ++i) {
            for (int j = 0; j < hspoints[i].size(); ++j) {
                double gpstime = hspoints[i][j].lfGpsSec;
                double range = hspoints[i][j].lfRange;

                if (rtime.size() == 2 && (gpstime < rtime[0] || gpstime > rtime[1])) {
                    continue;
                }

                if (rdist.size() == 2 && (range < rdist[0] || range > rdist[1])) {
                    continue;
                }

                double speed = pos.get_speed(gpstime);
                // 最低速度 1m/s
                if (speed < t_speed) {
                    continue;
                }

                gpssecs_.push_back(hspoints[i][j].lfGpsSec);
                points_.push_back(Vector3f(hspoints[i][j].pt.x, hspoints[i][j].pt.y, hspoints[i][j].pt.z));
                ranges_.push_back(hspoints[i][j].lfRange);
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

void HsrFile::export_las(const std::string &path, const Pos &pos, const LadybugStreamPtr&ladybug) {
    tm local_tm;
    {
        using namespace std::chrono;
        auto now = std::chrono::system_clock::now();
        time_t tt = system_clock::to_time_t(now);
        local_tm = *localtime(&tt);
    }

    BoundingBox3d bounds = pos.get_bounds();
    Vector3d offset = pos.get_position_offset();

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
    header->point_data_format = 3;
    header->point_data_record_length = 34;
    header->number_of_point_records = 0;
    for (int i = 0; i < 5; ++i) {
        header->number_of_points_by_return[i] = 0;
    }
    header->extended_number_of_point_records = points_.size();
    header->extended_number_of_points_by_return[0] = points_.size();
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

    if (!wkt_.empty()) {
        CHECK(wkt_.size() < 2048);
        uint8_t *data = new uint8_t[2048];
        memcpy(data, wkt_.c_str(), wkt_.size());
        data[wkt_.size() + 1] = '\0';
        if (laszip_add_vlr(writer, "LASF_Projection", 2112, wkt_.size() + 1, "OGC Transformation Record", data)) {
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

    laszip_point *laspoint;
    if (laszip_get_point_pointer(writer, &laspoint)) {
        char *err;
        laszip_get_error(writer, &err);
        CHECK(false) << err;
    }

    constexpr int NSEG = 1e7;
    for (int i = 0; i < points_.size(); i += NSEG) {

        int nsegments = std::min(NSEG, (int)points_.size() - i);

        std::vector<Vector3d> points(nsegments);
        std::vector<uint16_t> intensities(nsegments);
        std::vector<double> gpss(nsegments);
        std::vector<Vector3b> colors(ladybug == nullptr ? 0 : nsegments);

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

            if (ladybug != nullptr) {
                colors[p] = ladybug->colorize(point);
            }

            bounds.extend(points[p]);
        }

        for (int p = 0; p < nsegments; ++p) {
            laszip_set_coordinates(writer, points[p].data());
            laspoint->classification = 0;
            laspoint->extended_return_number = 1;
            laspoint->extended_number_of_returns = 1;
            laspoint->gps_time = gpss[p];
            laspoint->intensity = intensities[p];

            if (ladybug != nullptr) {
                laspoint->rgb[0] = colors[p].x();
                laspoint->rgb[1] = colors[p].y();
                laspoint->rgb[2] = colors[p].z();
                laspoint->rgb[3] = 255;
            }

            if (laszip_write_point(writer)) {
                CHECK(false) << "Write point failed!";
            }
        }
    }

    header->min_x = bounds.min().x();
    header->min_y = bounds.min().y();
    header->min_z = bounds.min().z();
    header->max_x = bounds.max().x();
    header->max_y = bounds.max().y();
    header->max_z = bounds.max().z();

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