#include "pos_rectify.h"
#include <base/base.h>
#include <iostream>

namespace h2o {
void Pos_rec::calculate_rotation(const std::string path_control_pts) {

    std::vector<Eigen::Vector3d> cpt_r;
    std::vector<Eigen::Vector3d> cpt_o;

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

        Eigen::Vector3d temp_o(std::stod(chunks[0]), std::stod(chunks[1]), std::stod(chunks[2]));
        Eigen::Vector3d temp_r(std::stod(chunks[3]), std::stod(chunks[4]), std::stod(chunks[5]));
        cpt_o.push_back(temp_o);
        cpt_r.push_back(temp_r);
    }

    //计算RT
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    // std::tuple(R,T)=calculateRT(cpt_o, cpt_r);

    //求中心点
    Eigen::Vector3d p1, p2;

    int N = cpt_o.size();
    for (int i = 0; i < N; i++) {
        p1 += cpt_o[i];
        p2 += cpt_r[i];
    }

    p1 = Eigen::Vector3d(Eigen::Vector3d(p1) / N);
    p2 = Eigen::Vector3d(Eigen::Vector3d(p2) / N);
    // 得到去中心坐标
    std::vector<Eigen::Vector3d> q1(N), q2(N);
    for (int i = 0; i < N; i++) {
        q1[i] = cpt_o[i] - p1;
        q2[i] = cpt_r[i] - p2;
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

    Eigen::Matrix3d R_ = V * M * (U.transpose());

    // t =  p' - R * p
    Eigen::Vector3d t_ = Eigen::Vector3d(p2.x(), p2.y(), p2.z()) - R_ * Eigen::Vector3d(p1.x(), p1.y(), p1.z());

    //计算POS
    Eigen::Vector3d pos_o(438061.025, 3402682.112, 429.882);
    Eigen::Vector3d pos_r(438062.16, 3402682.767, 432.125);
    auto chk = pos_r - t_ - R_ * pos_o;
    std::cout << chk.x() << "  " << chk.y() << "  " << chk.z() << std::endl;

    double x_sum = 0, y_sum = 0, z_sum = 0;
    for (int i = 0; i < cpt_r.size(); i++) {
        x_sum += cpt_r[i](0);
        y_sum += cpt_r[i](1);
        z_sum += cpt_r[i](2);
    }
    Eigen::Vector3d t(x_sum / cpt_r.size(), y_sum / cpt_r.size(), z_sum / cpt_r.size());
}
}