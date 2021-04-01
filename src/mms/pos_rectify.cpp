#include "pos_rectify.h"
#include <base/base.h>
#include <iostream>
#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <ceres/autodiff_cost_function.h>

using namespace ceres;

namespace h2o {

    struct CostFunctor 
    {
    template <typename T> bool operator()(const T *const x, T *residual) const { residual[0] = 10.0 - x[0];
        return true;
    }
    };
    /*
    void Pos_rec::test_ceres() { 
        google::InitGoogleLogging("");

        double inital_x = 5.0;
        double x = inital_x;

        Problem problem;

        CostFunction *cost_function = new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
        problem.AddResidualBlock(cost_function, nullptr, &x);

        

    }*/
    std::tuple<Matrix3d, Vector3d> Pos_rec::calculate_rotation(const std::string &path_control_pts) {
        std::vector<double> gps_time;
        std::vector<double> x_pointcloud;
        std::vector<double> y_pointcloud;
        std::vector<double> z_pointcloud;
        std::vector<double> x_measure;
        std::vector<double> y_measure;
        std::vector<double> z_measure;

    std::ifstream cpt_file(string_utf8_to_fstream(path_control_pts));
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
 
    std::vector<Vector3d> control_pointcloud;
    std::vector<Vector3d> control_measure;
    int ncontrol = gps_time.size();
    control_pointcloud.resize(ncontrol);
    control_measure.resize(ncontrol);

    for (int i=0;i<ncontrol;i++) {
        control_pointcloud[i] = Vector3d{x_pointcloud[i], y_pointcloud[i], z_pointcloud[i]};
        control_measure[i] = Vector3d{x_measure[i], y_measure[i], z_measure[i]};
    }

    Vector3d p1,p2;
    p1 = p2 = Vector3d::Zero();

    int N = control_pointcloud.size();
    for (int i = 0; i < N; i++) {
        p1 += control_pointcloud[i] ;
        p2 += control_measure[i] ;
    }
    

    p1 = Eigen::Vector3d(Eigen::Vector3d(p1) / N);
    p2 = Eigen::Vector3d(Eigen::Vector3d(p2) / N);
    //std::cout << p2 << std::endl;
    // 得到去中心坐标
    std::vector<Eigen::Vector3d> q1(N), q2(N);
    for (int i = 0; i < N; i++) {
        q1[i] = control_pointcloud[i] - p1;
        q2[i] = control_measure[i] - p2;
    }

    //计算需要进行奇异值分解的 W = sum(qi * qi’转置) compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++)
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

    //计算POS
    Eigen::Vector3d pos_o(358078.98600000, 3437848.65000000, 948.28400000);
    Eigen::Vector3d pos_r(358079.31700000, 3437848.26600000, 945.92100000);
    auto chk = pos_r - T_ - R_ * pos_o;
    std::cout << chk.x() << "  " << chk.y() << "  " << chk.z() << std::endl;

    double x_sum = 0, y_sum = 0, z_sum = 0;
    for (int i = 0; i < control_measure.size(); i++) {
        x_sum += control_measure[i](0);
        y_sum += control_measure[i](1);
        z_sum += control_measure[i](2);
    }
    Eigen::Vector3d t(x_sum / control_measure.size(), y_sum / control_measure.size(), z_sum / control_measure.size());

    return std::make_tuple(R_, T_);
    }

    Vector2d Pos_rec::get_tunnel_time() {

        return Vector2d::Zero();

    }
    }