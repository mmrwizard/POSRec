/*
 * Created Date: Friday, January 15th 2021, 8:36:13 pm
 * Author: Han
 * 将点云渲染到影像
 * Copyright (c) 2021 Vrlab
 */
#include <cxxopts.hpp>

#include <QApplication>
#include <QSurfaceFormat>

#include <base/base.h>
#include <mms/ladybug_stream.h>
#include <mms/lidar_render.h>

using namespace h2o;

int main(int argc, char **argv) {
    {
        QSurfaceFormat format;
        format.setVersion(3, 3);
        format.setProfile(QSurfaceFormat::CoreProfile);
        format.setDepthBufferSize(24);
        format.setStencilBufferSize(8);
        format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
        QSurfaceFormat::setDefaultFormat(format);

        // 这句话必须要，否则在 窗口 popup 得时候，会因为 context 改变了导致很多问题
        QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    }

    QApplication app(argc, argv);

    initilize_glog(argv);

    cxxopts::Options options("MMSRender");

    std::string path_las;
    std::string path_pos;
    std::string path_pan;
    std::string path_config;

    // clang-format off
    options.add_options("MMSRender")
      ("las", "Input las file", cxxopts::value(path_las))
      ("pos", "Input pos file", cxxopts::value(path_pos))
      ("pan", "Input panorama directory", cxxopts::value(path_pan))
      ("c,config", "Path to configuration file", cxxopts::value(path_config))
      ("h,help", "Print help message.");
    // clang-format on

    auto arguments = options.parse(argc, argv);

    if (arguments.count("help") || arguments.count("las") == 0 || arguments.count("pos") == 0 ||
        arguments.count("pan") == 0) {
        std::cout << options.help({"MMSRender"}) << std::endl;
        return 1;
    }

    path_las = QFileInfo(QString::fromLocal8Bit(path_las.c_str())).absoluteFilePath().toStdString();
    path_pos = QFileInfo(QString::fromLocal8Bit(path_pos.c_str())).absoluteFilePath().toStdString();
    path_config = QFileInfo(QString::fromLocal8Bit(path_config.c_str())).absoluteFilePath().toStdString();
    path_pan = QFileInfo(QString::fromLocal8Bit(path_pan.c_str())).absoluteFilePath().toStdString();

    Pos pos;
    pos.load_pos(path_pos);

    std::string pano_id = get_filename(path_pan);
    std::string dir = path_pan;
    std::string timestamp = join_paths(dir, pano_id + ".syn");

    LadybugStreamPtr stream = std::make_shared<LadybugStream>(dir, pano_id, timestamp, pos);
    stream->set_body(path_config);
    stream->set_image_type(path_config);

    LiDARRender render;
    render.load_las(path_las);

    auto images = stream->get_image_infos();
    int i = 0;
    for (auto image : images) {
        if (++i < 50) {
            continue;
        }
        render.draw_color(image);
    }

    return 0;
}