/*
 * Created Date: Wednesday, December 23rd 2020, 3:48:16 pm
 * Author: Han
 * 检校 LiDAR 与 IMU 之间的参数
 * Copyright (c) 2020 Vrlab
 */

#include <cxxopts.hpp>

#include <QApplication>
#include <base/base.h>
#include <base/json.h>
#include <mms/hsr_file.h>
#include <mms/lidar_calibrate.h>
#include <mms/pos.h>

using namespace h2o;

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    initilize_glog(argv);

    cxxopts::Options options("MMSLiDARCalibrate");

    std::string path_gcp;
    std::string path_tps;
    std::string path_config;

    // clang-format off
    options.add_options("MMSLiDARCalibrate")
      ("g,gcp", "Input control files", cxxopts::value(path_gcp))
      ("t,tie", "Input tie points", cxxopts::value(path_tps))
      ("c,config", "Path to configuration file", cxxopts::value(path_config))
      ("h,help", "Print help message.");
    // clang-format on

    auto arguments = options.parse(argc, argv);

    if (arguments.count("help")) {
        std::cout << options.help({"MMSLiDARCalibrate"}) << std::endl;
        return 1;
    }

    path_gcp = QFileInfo(QString::fromLocal8Bit(path_gcp.c_str())).absoluteFilePath().toStdString();
    path_tps = QFileInfo(QString::fromLocal8Bit(path_tps.c_str())).absoluteFilePath().toStdString();
    path_config = QFileInfo(QString::fromLocal8Bit(path_config.c_str())).absoluteFilePath().toStdString();

    LiDARCalibrate calibrator;

    Quaterniond q_body;
    Vector3d t_body;
    std::tie(q_body, t_body) = HsrFile::read_lidar_body(path_config);
    calibrator.set_body(q_body, t_body);

    std::vector<LiDARCalibrate::GCP> gcps;
    std::vector<LiDARCalibrate::TPS> tps;
    if (!path_gcp.empty()) {
        gcps = LiDARCalibrate::GCP::load(path_gcp);
        calibrator.set_gcp(gcps);
    }

    if (!path_tps.empty()) {
        tps = LiDARCalibrate::TPS::load(path_tps);
        calibrator.set_tps(tps);
    }

    Quaterniond dq;
    Vector3d dt;
    std::tie(dq, dt) = calibrator.calibrate();

    q_body = dq * q_body;
    t_body = dt + t_body;

    // 保存新参数
    Vector3d ypr = Pos::rotation_to_ypr(q_body.toRotationMatrix());

    std::string name = get_filename_noext(path_config);
    name = join_paths(get_directory(path_config), name + "_cal.json");

    nlohmann::json config;
    nlohmann::json j;
    j["X"] = t_body.x();
    j["Y"] = t_body.y();
    j["Z"] = t_body.z();
    j["Yaw"] = ypr(0) * RAD2DEG;
    j["Pitch"] = ypr(1) * RAD2DEG;
    j["Roll"] = ypr(2) * RAD2DEG;

    config["LiDAR"] = j;

    save_json_file(config, name);

    return 0;
}