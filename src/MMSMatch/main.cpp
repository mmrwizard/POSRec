/*
 * Created Date: Tuesday, January 5th 2021, 7:22:55 pm
 * Author: Han
 * 获取两站点云之间的同名点
 * Copyright (c) 2021 Vrlab
 */

#include <cxxopts.hpp>

#include <QApplication>
#include <base/base.h>
#include <mms/lidar_matcher.h>

using namespace h2o;

int main(int argc, char **argv) {
    QApplication app(argc, argv);

    initilize_glog(argv);

    cxxopts::Options options("MMSMatch");

    std::string path_pcdi, path_pcdj;
    std::string path_posi, path_posj;
    std::string path_config;

    // clang-format off
    options.add_options("MMSMatch")
      ("s,source", "Path to the first laz file", cxxopts::value(path_pcdi))
      ("t,target", "Path to the second laz file", cxxopts::value(path_pcdj))
      ("source-pos", "Path to the pos file of first las file", cxxopts::value(path_posi))
      ("target-pos", "Path to the pos file of second las file", cxxopts::value(path_posj))
      ("c,config", "Path to configuration file", cxxopts::value(path_config))
      ("h,help", "Print help message.");
    // clang-format on

    auto arguments = options.parse(argc, argv);

    if (arguments.count("help") || arguments.count("s") == 0 || arguments.count("t") == 0 ||
        arguments.count("source-pos") == 0 || arguments.count("target-pos") == 0) {
        std::cout << options.help({"MMSMatch"}) << std::endl;
        return 1;
    }

    path_pcdi = QFileInfo(QString::fromLocal8Bit(path_pcdi.c_str())).absoluteFilePath().toStdString();
    path_pcdj = QFileInfo(QString::fromLocal8Bit(path_pcdj.c_str())).absoluteFilePath().toStdString();
    path_posi = QFileInfo(QString::fromLocal8Bit(path_posi.c_str())).absoluteFilePath().toStdString();
    path_posj = QFileInfo(QString::fromLocal8Bit(path_posj.c_str())).absoluteFilePath().toStdString();
    path_config = QFileInfo(QString::fromLocal8Bit(path_config.c_str())).absoluteFilePath().toStdString();

    LiDARMatchParam param;
    if (!path_config.empty())
        param = lidar_load_config(path_config);

    LiDARMatches matches = lidar_match(path_pcdi, path_pcdj, path_posi, path_posj, param);

    return 0;
}
