/*
 * @Author: Han Hu
 * @Date: 2020-12-18 22:42:03
 * @Description: MMS 数据格式转换入口
 */
#include <cxxopts.hpp>

#include <QApplication>
#include <base/base.h>
#include <mms/hsr_file.h>
#include <mms/ladybug_stream.h>
#include <mms/pos.h>

using namespace h2o;

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    initilize_glog(argv);

    cxxopts::Options options("MMSConverter");

    std::string path_input;
    std::string mode;
    std::string path_pos;
    std::string path_config;
    std::string path_pano;

    std::vector<double> time_range;

    // clang-format off
    options.add_options("MMSConverter")
      ("i,input", "Input file path", cxxopts::value(path_input))
      ("pano", "Directory of the panorama", cxxopts::value(path_pano))
      ("p,pos", "Path to pos file", cxxopts::value(path_pos))
      ("c,config", "Path to configuration file", cxxopts::value(path_config))
      ("t,time", "Range of UTC time, as --time=1.0,10.0", cxxopts::value(time_range))
      ("h,help", "Print help message.");
    // clang-format on

    auto arguments = options.parse(argc, argv);

    if (arguments.count("help") || arguments.count("i") == 0) {
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
        file.load_with_pos(pos);
        file.set_body(path_config);

        std::string name = get_filename_noext(path_input);
        std::string dir = get_directory(path_input);
        name = join_paths(dir, name + ".laz");
        file.export_las(name, pos);
    } else {
        std::string pano_id = get_filename(path_pano);
        std::string dir = path_pano;
        std::string timestamp = join_paths(dir, pano_id + ".syn");

        LadybugStreamPtr stream = std::make_shared<LadybugStream>(dir, pano_id, timestamp, pos);
        stream->set_body(path_config);

        HsrFile file(path_input);
        file.load_with_pos(pos);
        file.set_body(path_config);

        std::string name = get_filename_noext(path_input);
        dir = get_directory(path_input);
        name = join_paths(dir, name + ".laz");
    }

    return 0;
}
