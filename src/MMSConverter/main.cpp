/*
 * @Author: Han Hu
 * @Date: 2020-12-18 22:42:03
 * @Description: MMS 数据格式转换入口
 */
#include <cxxopts.hpp>

#include <QApplication>
#include <base/base.h>
#include <cpl_conv.h>
#include <gdal.h>
#include <mms/hsr_file.h>
#include <mms/ladybug_stream.h>
#include <mms/pos.h>

using namespace h2o;

void set_gdal_environment();

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    initilize_glog(argv);

    cxxopts::Options options("MMSConverter");

    std::string path_input;
    std::string mode;
    std::string path_pos;
    std::string path_config;
    std::string path_pano;

    // clang-format off
    options.add_options("MMSConverter")
      ("i,input", "Input file path", cxxopts::value(path_input))
      ("pano", "Directory of the panorama", cxxopts::value(path_pano))
      ("p,pos", "Path to pos file", cxxopts::value(path_pos))
      ("c,config", "Path to configuration file", cxxopts::value(path_config))
      ("h,help", "Print help message.");
    // clang-format on

    auto arguments = options.parse(argc, argv);

    if (arguments.count("help") || arguments.count("i") == 0 || arguments.count("pos") == 0) {
        std::cout << options.help({"MMSConverter"}) << std::endl;
        return 1;
    }

    path_input = QFileInfo(QString::fromLocal8Bit(path_input.c_str())).absoluteFilePath().toStdString();
    path_pos = QFileInfo(QString::fromLocal8Bit(path_pos.c_str())).absoluteFilePath().toStdString();
    path_config = QFileInfo(QString::fromLocal8Bit(path_config.c_str())).absoluteFilePath().toStdString();
    path_pano = QFileInfo(QString::fromLocal8Bit(path_pano.c_str())).absoluteFilePath().toStdString();

    Pos pos;
    pos.load_pos(path_pos);

    if (path_pano.empty()) {
        HsrFile file(path_input);
        file.set_config(path_config);
        file.set_body(path_config);

        std::string name = get_filename_noext(path_input);
        std::string dir = get_directory(path_input);
        name = join_paths(dir, name + ".laz");
        file.export_las_ooc(name, pos);
    } else {
        std::string pano_id = get_filename(path_pano);
        std::string dir = path_pano;
        std::string timestamp = join_paths(dir, pano_id + ".syn");

        LadybugStreamPtr stream = std::make_shared<LadybugStream>(dir, pano_id, timestamp, pos);
        stream->set_body(path_config);
        stream->set_image_type(path_config);

        HsrFile file(path_input);
        file.set_body(path_config);
        file.set_config(path_config);

        std::string name = get_filename_noext(path_input);
        dir = get_directory(path_input);
        name = join_paths(dir, name + ".laz");
        file.export_colored_las_ooc(name, pos, stream);
    }

    return 0;
}

void set_gdal_environment() {
    GDALAllRegister();

    QString appfile = QApplication::applicationDirPath();
    std::string qmdir = h2o::join_paths(appfile.toStdString(), "resources/gdal_data/");

    CPLSetConfigOption("GDAL_DATA", qmdir.c_str());
    CPLSetConfigOption("DXF_WRITE_HATCH", "FALSE"); // 必须要在这创建 DXF 导出的选项
    CPLSetConfigOption("TIFF_USE_OVR", "TRUE");     // 必须要在这创建 DXF 导出的选项

    std::string proj = h2o::join_paths(appfile.toStdString(), "resources/proj/");
    set_proj_directory(proj);
    proj = "PROJ_LIB=" + proj;
    putenv((char *)proj.c_str());
}