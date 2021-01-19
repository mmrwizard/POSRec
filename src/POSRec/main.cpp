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

void load_cpts();

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
      ("pano", "Directory of the panorama", cxxopts::value(path_pano))
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
    path_pano = QFileInfo(QString::fromLocal8Bit(path_pano.c_str())).absoluteFilePath().toStdString();
    path_cpt = QFileInfo(QString::fromLocal8Bit(path_cpt.c_str())).absoluteFilePath().toStdString();

    Pos pos;
    pos.load_pos(path_pos);

    std::vector<Eigen::Vector3d> cpt_r;
    std::vector<Eigen::Vector3d> cpt_o;

    std::ifstream cpt_file(string_utf8_to_fstream(path_cpt));
    std::string line;
    while (std::getline(cpt_file,line) && cpt_file.good()){
        string_left_trim(&line);
        string_right_trim(&line);

        auto new_end =
            std::unique(begin(line), end(line), [](char lhs, char rhs) { return (lhs == rhs) && (lhs == ' '); });
        line.erase(new_end, end(line));
        std::vector<std::string> chunks = string_split(line, " ");
        if (chunks.empty())
        {
            continue;
        }
        // 判断第一个点是不是数字
        try {
            double first_value = std::stod(chunks.front());
        } catch (const std::exception &) {
            continue;
        }

        Eigen::Vector3d temp_o(std::stod(chunks[0]), std::stod(chunks[1]), std::stod(chunks[2]));
        Eigen::Vector3d temp_r(std::stod(chunks[3]), std::stod(chunks[4]), std::stod(chunks[5]));
        cpt_o.push_back(temp_o);
        cpt_r.push_back(temp_r);
    }
    
    double x_sum = 0, y_sum = 0, z_sum = 0;
    for (int i=0; i<cpt_r.size();i++)
    {
        x_sum += cpt_r[i](0);
        y_sum += cpt_r[i](1);
        z_sum += cpt_r[i](2);
    }
    Eigen::Vector3d t(x_sum / cpt_r.size(), y_sum / cpt_r.size(), z_sum / cpt_r.size());


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
