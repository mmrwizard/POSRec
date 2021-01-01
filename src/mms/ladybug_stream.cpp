/*
 * Created Date: Tuesday, December 29th 2020, 2:55:29 pm
 * Author: Han
 * Ladybug 影像
 * Copyright (c) 2020 Vrlab
 */
#include "ladybug_stream.h"

#include <base/base.h>
#include <base/json.h>
#include <opencv2/imgcodecs.hpp>

#include <QDir>
#include <QDirIterator>

namespace h2o {
LadybugStream::LadybugStream(const std::string &dir, const std::string &name, const std::string &timestamp,
                             const Pos &pos)
    : pos_(pos) {

    mat_caches_ = std::make_shared<CacheImages>();

    std::ifstream file(string_utf8_to_fstream(timestamp));
    std::string line;

    QFileInfoList infos = QDir(QString::fromStdString(dir))
                              .entryInfoList(QStringList() << "*.jpg"
                                                           << "*.png"
                                                           << "*.jpeg");

    while (std::getline(file, line) && file.good()) {
        if (line.empty()) {
            continue;
        }
        int id, year, month, day, hour, minute, sec, millsec;
        sscanf(line.data(), "#C%05d,%d,%d,%d,%d,%d,%d,%d", &id, &year, &month, &day, &hour, &minute, &sec, &millsec);
        id -= 1;

        double sec_of_day;
        {
            std::tm tm_local;
            tm_local.tm_year = year - 1900;
            tm_local.tm_mon = month - 1;
            tm_local.tm_mday = day;
            tm_local.tm_hour = hour;
            tm_local.tm_min = minute;
            tm_local.tm_sec = sec;

            std::tm tm_utc = get_utc_time(tm_local);
            double sec_of_day_integer = get_seconds_of_day(tm_utc);
            sec_of_day = sec_of_day_integer + millsec / 1000.0;
        }

        // 从文件夹中读取这个影像
        std::string filename;
        for (const auto &info : infos) {
            QString qname = info.absoluteFilePath();
            QString pattern = QString::asprintf("%s_Panoramic_%06d", name.c_str(), id);

            if (qname.count(pattern)) {
                filename = qname.toStdString();
                break;
            }
        }
        if (filename.empty()) {
            continue;
        }

        auto [t_pos, R_pos] = pos.getd(sec_of_day);

        LadybugImage image;
        ImageHeader image_header;
        gdal_image_header(filename, image_header);
        image.height = image_header.height;
        image.width = image_header.width;
        image.path = filename;
        image.time = sec_of_day;
        image.type = 1;
        image.t_world = t_pos + pos.get_position_offset();
        image.q_world = R_pos;

        images_.push_back(image);
    }

    //
    // debug_trajectory();

    MatrixXd camera_positions(3, images_.size());
    for (int i = 0; i < images_.size(); ++i) {
        camera_positions(0, i) = images_[i].t_world.x();
        camera_positions(1, i) = images_[i].t_world.y();
        camera_positions(2, i) = images_[i].t_world.z();
    }
    trajectory_ = camera_positions;
    kdtree_trajectory_ = std::make_shared<KDTreeTrajectory>(3, trajectory_);
}

void LadybugStream::set_image_type(int type) {
    for (auto &image : images_) {
        image.type = type;
    }
}

void LadybugStream::set_image_type(const std::string &path) {
    nlohmann::json j = read_json_file(path);
    if (!j.count("Pano")) {
        return;
    }
    j = j["Pano"];
    int type = j["ImageType"];
    set_image_type(type);
}

void LadybugStream::set_body(const std::string &path) {
    nlohmann::json j = read_json_file(path);
    if (!j.count("Pano")) {
        return;
    }
    j = j["Pano"];

    Vector3d t, r;
    t(0) = j["X"];
    t(1) = j["Y"];
    t(2) = j["Z"];

    r(0) = j["Yaw"].get<double>() * DEG2RAD;
    r(1) = j["Pitch"].get<double>() * DEG2RAD;
    r(2) = j["Roll"].get<double>() * DEG2RAD;

    set_body(r, t);
}

void LadybugStream::set_body(const Vector3d &r, const Vector3d &t) {
    q_body_ = Pos::ypr_to_rotation(r(0), r(1), r(2));
    t_body_ = t;

    for (auto &image : images_) {
        image.q_body = q_body_;
        image.t_body = t_body_;
    }
}

Vector3b LadybugStream::colorize_on_images(const Vector3d &X, const std::vector<uint32_t> &images) const {
    for (Eigen::Index index : images) {
        if (!mat_caches_->contains(index)) {
            continue;
        }

        const auto &image = images_.at(index);
        Vector3d local = image.to_local(X);
        Vector2d angle = image.to_angle(local);
        bool valid = image.is_valid(angle);

        if (!valid) {
            continue;
        }

        Vector2d pixel = image.to_pixel(angle);

        cv::Mat mat = mat_caches_->get(index);

        cv::Vec3b color = image_interpolate<cv::Vec3b>(mat, pixel.cast<float>(), cv::BORDER_REPLICATE);
        return Vector3b(color[2], color[1], color[0]);
    }

    return Vector3b::Zero();
}

void LadybugStream::cache_images(const std::vector<uint32_t> &images) {
    for (uint32_t index : images) {
        if (!mat_caches_->contains(index)) {
            cv::Mat mat = cv::imread(string_utf8_to_fstream(images_.at(index).path), cv::IMREAD_COLOR);
            mat_caches_->insert(index, mat);
        }
    }
}

void LadybugStream::debug_trajectory() {
    std::ofstream ofile("C:\\Users\\huhan\\Documents\\Data\\MMS\\pano\\trajectory.txt");
    for (const auto &image : images_) {
        ofile << string_printf("%.3lf,", image.t_world.x());
        ofile << string_printf("%.3lf,", image.t_world.y());
        ofile << string_printf("%.3lf,", image.t_world.z());
        ofile << string_printf("%.3lf\n", image.time);
    }
}

Vector2d LadybugImage::to_pixel(const Vector2d &angle) const {
    constexpr double m_1_pi = 0.318309886183790671538; // 1 / pi
    Vector2d uv;

    if (type == 0) {
        uv.x() = 2.0 * height - angle.x() * height * m_1_pi;
        // 范围为 [-pi/2, pi/2]
        uv.y() = 0.5 * height - angle.y() * height * m_1_pi;
    } else if (type == 1) {
        uv.x() = angle.x() * height * m_1_pi + height;
        // 范围为 [-pi/4, pi/4]
        uv.y() = 0.5 * height - angle.y() * 0.5 * height;
    }

    // 计算值是从左上角的左上角即为 (0, 0)，但是应从左上角像素的中心
    uv -= Vector2d(.5, .5);
    return uv;
}

Vector2d LadybugImage::to_angle(const Vector2d &uv) const {
    constexpr double m_pi = 3.14159265358979323846;

    // 计算值是从左上角的左上角即为 (0, 0)，但是应从左上角像素的中心
    Vector2d uv_corner = uv + Vector2d(0.5, 0.5);

    Vector2d angle;
    angle.x() = (uv_corner.x() - height) / height * m_pi;
    if (type == 0) {
        angle.y() = (height - 2.0 * uv_corner.y()) / (2.0 * height) * m_pi;
    } else if (type == 1) {
        angle.y() = 1 - 2.0 * uv_corner.y() / height;
    }

    return angle;
}

Vector3d LadybugImage::to_local(const Vector3d &world) const {
    return q_body.conjugate() * (q_world.conjugate() * (world - t_world) - t_body);
}

Vector3d LadybugImage::to_world(const Vector3d &local) const { return q_world * (q_body * local + t_body) + t_world; }

Vector2d LadybugImage::to_angle(const Vector3d &local) const {
    Vector2d angle;
    double r = local.norm();
    angle.x() = atan2(local.y(), local.x());

    if (type == 0) {
        angle.x() += angle.x() < 0 ? 2.0 * M_PI : 0.0;
        angle.y() = asin(local.z() / r);
    } else if (type == 1) {
        double r2 = local.head(2).norm();
        angle.y() = local.z() / r2;
    }

    return angle;
}

bool LadybugImage::is_valid(const Vector2d &angle) const {
    if (type == 0) {
        return true;
    } else if (type == 1) {
        return std::abs(angle.y()) < 1.0;
    }

    return false;
}

} // namespace h2o
