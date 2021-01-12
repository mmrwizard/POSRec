/*
 * Created Date: Tuesday, January 12th 2021, 2:25:19 pm
 * Author: Han
 * 采用 ICP 方法进行精检校，不需要同名点，但是需要较精确初值
 * Copyright (c) 2021 Vrlab
 */

#include <cxxopts.hpp>

#include <QApplication>
#include <base/base.h>
#include <base/json.h>
#include <mms/hsr_file.h>
#include <mms/lidar_calibrate_icp.h>
#include <mms/pos.h>

using namespace h2o;

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    initilize_glog(argv);

    cxxopts::Options options("MMSLiDARCalibrateIcp");

    std::vector<std::string> paths_las;
    std::vector<std::string> paths_pos;
    std::string path_config;

    // clang-format off
    options.add_options("MMSLiDARCalibrateIcp")
      ("l,las", "Input two las files path1,path2", cxxopts::value(paths_las))
      ("p,pos", "Input two pos files path1,path2", cxxopts::value(paths_pos))
      ("c,config", "Path to configuration file", cxxopts::value(path_config))
      ("h,help", "Print help message.");
    // clang-format on

    auto arguments = options.parse(argc, argv);

    if (arguments.count("help") || arguments.count("config") == 0) {
        std::cout << options.help({"MMSLiDARCalibrateIcp"}) << std::endl;
        return 1;
    }

    for (int i = 0; i < 2; ++i) {
        paths_las[i] = QFileInfo(QString::fromLocal8Bit(paths_las[i].c_str())).absoluteFilePath().toStdString();
        paths_pos[i] = QFileInfo(QString::fromLocal8Bit(paths_pos[i].c_str())).absoluteFilePath().toStdString();
    }
    path_config = QFileInfo(QString::fromLocal8Bit(path_config.c_str())).absoluteFilePath().toStdString();

    Quaterniond q_body;
    Vector3d t_body;
    std::tie(q_body, t_body) = HsrFile::read_lidar_body(path_config);

    Quaterniond dq;
    Vector3d dt;

    LiDARCalibrateIcpParam param = LiDARCalibrateIcpParam::load_config(path_config);

    LiDARCalibrateIcp calibrator(paths_las[0], paths_las[1], paths_pos[0], paths_pos[1], param);
    calibrator.set_body(q_body, t_body);

    std::tie(dq, dt) = calibrator.calibrate();

    q_body = dq * q_body;
    t_body = dt + t_body;

    // 保存新参数
    Vector3d ypr = Pos::rotation_to_ypr(q_body.toRotationMatrix());

    std::string name = get_filename_noext(path_config);
    name = join_paths(get_directory(path_config), name + "_cal.json");

    nlohmann::json config = read_json_file(path_config);
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
