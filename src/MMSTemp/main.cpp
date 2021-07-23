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

    std::vector<std::vector<VectorXd>> control_points_sets;
    std::vector<Matrix3d> rotations;
    std::vector<Vector3d> translations;
    std::vector<Quaterniond> rotation_quaternions;

    /*读取控制点*/
    std::vector<double> gps_time;
    std::vector<double> x_pointcloud;
    std::vector<double> y_pointcloud;
    std::vector<double> z_pointcloud;
    std::vector<double> x_measure;
    std::vector<double> y_measure;
    std::vector<double> z_measure;
    std::string path_pairs = "F:\\hdsy\\controls-half.txt";

    std::ifstream cpt_file(string_utf8_to_fstream(path_pairs));
    std::string line;
    while (std::getline(cpt_file, line) && cpt_file.good()) {
        string_left_trim(&line);
        string_right_trim(&line);

        auto new_end =
            std::unique(begin(line), end(line), [](char lhs, char rhs) { return (lhs == rhs) && (lhs == ' '); });
        line.erase(new_end, end(line));
        std::vector<std::string> chunks = string_split(line, " ");
        if (chunks.empty()) {
            continue;
        }
        // 判断第一个点是不是数字
        try {
            double first_value = std::stod(chunks.front());
        } catch (const std::exception &) {
            continue;
        }
        gps_time.push_back(std::stod(chunks[0]));
        x_pointcloud.push_back(std::stod(chunks[1]));
        y_pointcloud.push_back(std::stod(chunks[2]));
        z_pointcloud.push_back(std::stod(chunks[3]));
        x_measure.push_back(std::stod(chunks[4]));
        y_measure.push_back(std::stod(chunks[5]));
        z_measure.push_back(std::stod(chunks[6]));
    }
    std::vector<VectorXd> controls;
    VectorXd control;
    control.resize(7);
    int nframes = gps_time.size();
    for (int i = 0; i < nframes; i++) {

        control[0] = gps_time[i];
        control[1] = x_pointcloud[i];
        control[2] = y_pointcloud[i];
        control[3] = z_pointcloud[i];
        control[4] = x_measure[i];
        control[5] = y_measure[i];
        control[6] = z_measure[i];
        controls.push_back(control);
    }

    /*更新POS*/
    int nsets = control_points_sets.size();
    rotations.resize(nsets + 2);
    translations.resize(nsets + 2);
    rotation_quaternions.resize(nsets + 2);
    Vector2d tunnel_period = Vector2d(DBL_MAX, -DBL_MAX);

    //计算旋转平移参数

    std::vector<Vector3d> control_pointclouds;
    std::vector<Vector3d> control_surveys;
    Vector3d p1, p2, controlpoint, surveypoint;
    p1 = p2 = Vector3d::Zero();
    for (int j = 0; j < nframes; j++) {
        controlpoint.x() = controls[j][1];
        controlpoint.y() = controls[j][2];
        controlpoint.z() = controls[j][3];
        surveypoint.x() = controls[j][4];
        surveypoint.y() = controls[j][5];
        surveypoint.z() = controls[j][6];
        control_pointclouds.push_back(controlpoint);
        control_surveys.push_back(surveypoint);
    }
    for (int j = 0; j < nframes; j++) {
        p1 += control_pointclouds[j];
        p2 += control_surveys[j];
    }
    p1 = Eigen::Vector3d(Eigen::Vector3d(p1) / nframes);
    p2 = Eigen::Vector3d(Eigen::Vector3d(p2) / nframes);
    std::vector<Eigen::Vector3d> q1(nframes), q2(nframes);
    for (int j = 0; j < nframes; j++) {
        q1[j] = control_pointclouds[j] - p1;
        q2[j] = control_surveys[j] - p2;
    }
    //计算需要进行奇异值分解的 W = sum(qi * qi’转置) compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < nframes; i++)
        W += Eigen::Vector3d(q1[i].x(), q1[i].y(), q1[i].z()) *
             Eigen::Vector3d(q2[i].x(), q2[i].y(), q2[i].z()).transpose();
    //对  W 进行SVD 奇异值分解
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    // 计算旋转 和平移矩阵 R  和 t,  R= V *M* UT
    double det = (U * V.transpose()).determinant();
    Eigen::Matrix3d M;
    M << 1, 0, 0, 0, 1, 0, 0, 0, det;
    Matrix3d R_ = V * M * (U.transpose());
    // t =  p' - R * p
    Vector3d T_ = Eigen::Vector3d(p2.x(), p2.y(), p2.z()) - R_ * Eigen::Vector3d(p1.x(), p1.y(), p1.z());
    std::cout << R_ << std::endl;
    std::cout << T_ << std::endl;

    Vector3d p11(359046.057, 3435941.898, 906.9648), p22(358905.4414, 3436015.216, 908.296021);
    auto checkp = R_ * Vector3d(p11.x(),p11.y(),p11.z()) - Vector3d(p11.x(),p11.y(),p11.z());
    std::cout << checkp << std::endl;
    return 0;
}

// int main() {
//
//
//    std::vector<double> points_x;
//    std::vector<double> points_y;
//    std::vector<double> points_z;
//    std::vector<double> points_t;
//    std::vector<double> points_i;
//
//    std::vector<double> targets_x;
//    std::vector<double> targets_y;
//    std::vector<double> targets_z;
//
//    std::vector<std::vector<double>> control_pairs;
//    std::vector<double> control_pair;
//
//    for (int i = 1; i < 19; i++)
//    {
//        std::string path_target, path_cloud;
//
//        path_target = std::to_string(i) + "-t.txt";
//        path_cloud = std::to_string(i) + ".txt";
//
//
//        std::ifstream target_file(string_utf8_to_fstream(path_target));
//        std::ifstream cloud_file(string_utf8_to_fstream(path_cloud));
//        std::string line;
//        //读取实测标靶
//        {
//            while (getline(target_file, line) && target_file.good()) {
//                string_left_trim(&line);
//                string_right_trim(&line);
//
//                auto new_end = std::unique(begin(line), end(line),
//                                           [](char lhs, char rhs) { return (lhs == rhs) && (lhs == ' '); });
//                line.erase(new_end, end(line));
//                std::vector<std::string> chunks = string_split(line, ",");
//                if (chunks.empty()) {
//                    continue;
//                }
//
//                // 判断第一个点是不是数字
//                try {
//                    double first_value = std::stod(chunks.front());
//                } catch (const std::exception &) {
//                    continue;
//                }
//
//                targets_x.push_back(std::stod(chunks[0]));
//                targets_y.push_back(std::stod(chunks[1]));
//                targets_z.push_back(std::stod(chunks[2]));
//            }
//        }
//        //读取点云
//        {
//            while (getline(cloud_file, line) && cloud_file.good()) {
//                string_left_trim(&line);
//                string_right_trim(&line);
//
//                auto new_end = std::unique(begin(line), end(line),
//                                           [](char lhs, char rhs) { return (lhs == rhs) && (lhs == ' '); });
//                line.erase(new_end, end(line));
//                std::vector<std::string> chunks = string_split(line, " ");
//                if (chunks.empty()) {
//                    continue;
//                }
//
//                // 判断第一个点是不是数字
//                try {
//                    double first_value = std::stod(chunks.front());
//                    double vec_size = (chunks.size() > 3);
//                } catch (const std::exception &) {
//                    continue;
//                }
//                if (chunks.size()>3)
//                {
//                    points_x.push_back(std::stod(chunks[0]));
//                    points_y.push_back(std::stod(chunks[1]));
//                    points_z.push_back(std::stod(chunks[2]));
//                    points_t.push_back(std::stod(chunks[6]));
//                    points_i.push_back(std::stod(chunks[7]));
//                }
//            }
//        }
//        std::cout << i;
//    }
//
//    for (int j = 0; j < targets_x.size(); j++)
//    {
//        for (int k = 0; k < points_x.size(); k++)
//        {
//            if (std::abs(points_x[k] - targets_x[j]) < 0.001 && std::abs(points_y[k] - targets_y[j]) < 0.001 &&
//                std::abs(points_z[k] - targets_z[j]) < 0.001 )
//            {
//                control_pair.resize(7);
//                control_pair[0] = points_t[k];
//                control_pair[1] = points_x[k];
//                control_pair[2] = points_y[k];
//                control_pair[3] = points_z[k];
//                control_pair[4] = targets_x[j];
//                control_pair[5] = targets_y[j];
//                control_pair[6] = targets_z[j];
//                control_pairs.push_back(control_pair);
//                control_pair.clear();
//                continue;
//            }
//        }
//    }
//    std::ofstream controls_file("controls.txt");
//
//    for (int i =0;i<control_pairs.size();i++)
//    {
//        controls_file << std::to_string(control_pairs[i][0]) << " ";
//        controls_file << std::to_string(control_pairs[i][1]) << " ";
//        controls_file << std::to_string(control_pairs[i][2]) << " ";
//        controls_file << std::to_string(control_pairs[i][3]) << " ";
//        controls_file << std::to_string(control_pairs[i][4]) << " ";
//        controls_file << std::to_string(control_pairs[i][5]) << " ";
//        controls_file << std::to_string(control_pairs[i][6]) << " ";
//        controls_file << "\n ";
//    }
//    controls_file.close();
//
//
//
//    return 0;
//}

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