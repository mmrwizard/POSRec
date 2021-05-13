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

int main() {
    

    std::vector<double> points_x;
    std::vector<double> points_y;
    std::vector<double> points_z;
    std::vector<double> points_t;
    std::vector<double> points_i;

    std::vector<double> targets_x;
    std::vector<double> targets_y;
    std::vector<double> targets_z;

    std::vector<std::vector<double>> control_pairs;
    std::vector<double> control_pair;
    
    for (int i = 1; i < 19; i++)
    {
        std::string path_target, path_cloud;

        path_target = std::to_string(i) + "-t.txt";
        path_cloud = std::to_string(i) + ".txt";


        std::ifstream target_file(string_utf8_to_fstream(path_target));
        std::ifstream cloud_file(string_utf8_to_fstream(path_cloud));
        std::string line;
        //读取实测标靶
        {
            while (getline(target_file, line) && target_file.good()) {
                string_left_trim(&line);
                string_right_trim(&line);

                auto new_end = std::unique(begin(line), end(line),
                                           [](char lhs, char rhs) { return (lhs == rhs) && (lhs == ' '); });
                line.erase(new_end, end(line));
                std::vector<std::string> chunks = string_split(line, ",");
                if (chunks.empty()) {
                    continue;
                }

                // 判断第一个点是不是数字
                try {
                    double first_value = std::stod(chunks.front());
                } catch (const std::exception &) {
                    continue;
                }

                targets_x.push_back(std::stod(chunks[0]));
                targets_y.push_back(std::stod(chunks[1]));
                targets_z.push_back(std::stod(chunks[2]));
            }
        }
        //读取点云
        {
            while (getline(cloud_file, line) && cloud_file.good()) {
                string_left_trim(&line);
                string_right_trim(&line);

                auto new_end = std::unique(begin(line), end(line),
                                           [](char lhs, char rhs) { return (lhs == rhs) && (lhs == ' '); });
                line.erase(new_end, end(line));
                std::vector<std::string> chunks = string_split(line, " ");
                if (chunks.empty()) {
                    continue;
                }

                // 判断第一个点是不是数字
                try {
                    double first_value = std::stod(chunks.front());
                    double vec_size = (chunks.size() > 3);
                } catch (const std::exception &) {
                    continue;
                }
                if (chunks.size()>3)
                {
                    points_x.push_back(std::stod(chunks[0]));
                    points_y.push_back(std::stod(chunks[1]));
                    points_z.push_back(std::stod(chunks[2]));
                    points_t.push_back(std::stod(chunks[6]));
                    points_i.push_back(std::stod(chunks[7]));
                }
            }
        }
        std::cout << i;
    }
    
    
    for (int j = 0; j < targets_x.size(); j++)
    {
        for (int k = 0; k < points_x.size(); k++)
        {
            if (std::abs(points_x[k] - targets_x[j]) < 0.001 && std::abs(points_y[k] - targets_y[j]) < 0.001 &&
                std::abs(points_z[k] - targets_z[j]) < 0.001 )
            {
                control_pair.resize(7);
                control_pair[0] = points_t[k];
                control_pair[1] = points_x[k];
                control_pair[2] = points_y[k];
                control_pair[3] = points_z[k];
                control_pair[4] = targets_x[j];
                control_pair[5] = targets_y[j];
                control_pair[6] = targets_z[j];
                control_pairs.push_back(control_pair);
                control_pair.clear();
                continue;
            }
        }        
    }

    

    std::ofstream controls_file("controls.txt");

    for (int i =0;i<control_pairs.size();i++)
    {
        controls_file << std::to_string(control_pairs[i][0]) << " ";
        controls_file << std::to_string(control_pairs[i][1]) << " ";
        controls_file << std::to_string(control_pairs[i][2]) << " ";
        controls_file << std::to_string(control_pairs[i][3]) << " ";
        controls_file << std::to_string(control_pairs[i][4]) << " ";
        controls_file << std::to_string(control_pairs[i][5]) << " ";
        controls_file << std::to_string(control_pairs[i][6]) << " ";
        controls_file << "\n ";
    }
    controls_file.close();



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