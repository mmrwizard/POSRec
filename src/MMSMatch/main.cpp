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
#include <mms/rapidcsv.h>

using namespace h2o;

int main(int argc, char **argv) {
    QApplication app(argc, argv);

    initilize_glog(argv);

    cxxopts::Options options("MMSMatch");

    std::string path_pcdi, path_pcdj;
    std::string path_posi, path_posj;
    std::string path_config;
    std::string path_output;

    // clang-format off
    options.add_options("MMSMatch")
      ("s,source", "Path to the first laz file", cxxopts::value(path_pcdi))
      ("t,target", "Path to the second laz file", cxxopts::value(path_pcdj))
      ("source-pos", "Path to the pos file of first las file", cxxopts::value(path_posi))
      ("target-pos", "Path to the pos file of second las file", cxxopts::value(path_posj))
      ("o,output", "Path to output file", cxxopts::value(path_output))
      ("c,config", "Path to configuration file", cxxopts::value(path_config))
      ("h,help", "Print help message.");
    // clang-format on

    auto arguments = options.parse(argc, argv);

    if (arguments.count("help") || arguments.count("s") == 0 || arguments.count("t") == 0 ||
        arguments.count("source-pos") == 0 || arguments.count("target-pos") == 0 || arguments.count("output") == 0) {
        std::cout << options.help({"MMSMatch"}) << std::endl;
        return 1;
    }

    path_pcdi = QFileInfo(QString::fromLocal8Bit(path_pcdi.c_str())).absoluteFilePath().toStdString();
    path_pcdj = QFileInfo(QString::fromLocal8Bit(path_pcdj.c_str())).absoluteFilePath().toStdString();
    path_posi = QFileInfo(QString::fromLocal8Bit(path_posi.c_str())).absoluteFilePath().toStdString();
    path_posj = QFileInfo(QString::fromLocal8Bit(path_posj.c_str())).absoluteFilePath().toStdString();
    path_config = QFileInfo(QString::fromLocal8Bit(path_config.c_str())).absoluteFilePath().toStdString();
    path_output = QFileInfo(QString::fromLocal8Bit(path_output.c_str())).absoluteFilePath().toStdString();

    LiDARMatchParam param;
    if (!path_config.empty())
        param = lidar_match_config(path_config);

    LiDARMatches matches = lidar_match(path_pcdi, path_pcdj, path_posi, path_posj, param);

    rapidcsv::Document doc(std::string(""), rapidcsv::LabelParams(-1, -1));

    // t1 x1 y1 z1 t2 x2 y2 z2 tx1 ty1 tz1 qx1 qy1 qz1 qw1 tx2 ty2 tz2 qx2 qy2 qz2 qw2
    for (int i = 0; i < matches.size(); ++i) {
        doc.SetCell(0, i, string_printf("%.3lf", matches[i].t1));
        doc.SetCell(1, i, string_printf("%.3lf", matches[i].p_lidar1.x()));
        doc.SetCell(2, i, string_printf("%.3lf", matches[i].p_lidar1.y()));
        doc.SetCell(3, i, string_printf("%.3lf", matches[i].p_lidar1.z()));
        doc.SetCell(4, i, string_printf("%.3lf", matches[i].t2));
        doc.SetCell(5, i, string_printf("%.3lf", matches[i].p_lidar2.x()));
        doc.SetCell(6, i, string_printf("%.3lf", matches[i].p_lidar2.y()));
        doc.SetCell(7, i, string_printf("%.3lf", matches[i].p_lidar2.z()));
        doc.SetCell(8, i, string_printf("%.3lf", matches[i].t_pos1.x()));
        doc.SetCell(9, i, string_printf("%.3lf", matches[i].t_pos1.y()));
        doc.SetCell(10, i, string_printf("%.3lf", matches[i].t_pos1.z()));
        doc.SetCell(11, i, string_printf("%.15lf", matches[i].q_pos1.x()));
        doc.SetCell(12, i, string_printf("%.15lf", matches[i].q_pos1.y()));
        doc.SetCell(13, i, string_printf("%.15lf", matches[i].q_pos1.z()));
        doc.SetCell(14, i, string_printf("%.15lf", matches[i].q_pos1.w()));
        doc.SetCell(15, i, string_printf("%.3lf", matches[i].t_pos2.x()));
        doc.SetCell(16, i, string_printf("%.3lf", matches[i].t_pos2.y()));
        doc.SetCell(17, i, string_printf("%.3lf", matches[i].t_pos2.z()));
        doc.SetCell(18, i, string_printf("%.15lf", matches[i].q_pos2.x()));
        doc.SetCell(19, i, string_printf("%.15lf", matches[i].q_pos2.y()));
        doc.SetCell(20, i, string_printf("%.15lf", matches[i].q_pos2.z()));
        doc.SetCell(21, i, string_printf("%.15lf", matches[i].q_pos2.w()));
    }

    doc.Save(string_utf8_to_fstream(path_output));

    return 0;
}
