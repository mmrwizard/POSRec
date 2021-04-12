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
    std::string path_cpt;

    // clang-format off
    options.add_options("MMSConverter")
      ("i,input", "Input file path", cxxopts::value(path_input))
      ("p,pos", "Path to pos file", cxxopts::value(path_pos))
      ("c,config", "Path to configuration file", cxxopts::value(path_config))
      ("control","control points file",cxxopts::value(path_cpt))
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
    path_cpt = QFileInfo(QString::fromLocal8Bit(path_cpt.c_str())).absoluteFilePath().toStdString();

    //计算新POS文件的位姿
    Pos pos;
    pos.read_controls(path_cpt);

    Vector3d translation;
    Matrix3d rotation;
    pos.load_pos(path_pos);
    pos.calculate_rotation(path_cpt);

    VectorXd gpstimes;
    //gpstimes = pos.get_gpstime();

    //保存为新POS文件

    HsrFile file(path_input);
    file.set_config(path_config);
    file.set_body(path_config);

    std::string name = get_filename_noext(path_input);
    std::string dir = get_directory(path_input);
    name = join_paths(dir, name + ".laz");
    file.export_las_ooc(name, pos);
   
    return 0;
}
