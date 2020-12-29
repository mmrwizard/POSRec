/*
 * @Author: Han Hu
 * @Date: 2020-12-18 21:27:51
 * @Description: 读取 HSR 文件，结合 POS 输出信息
 */

#pragma once

#include "pos.h"

#include <base/common.h>
#include <chrono>

class HsRawiLSP;

namespace h2o {
class HsrFile {
public:
    HsrFile(const std::string &path);
    ~HsrFile();

    // 获取文件获取的实践，点云数据存储的是 UTC 天内秒，
    std::tm get_file_date();
    void load();
    void load_pos_filtered(const Pos &pos, double t_speed = 1.0,
                           const std::vector<double> &rtime = std::vector<double>(),
                           const std::vector<double> &rdist = {1.5, 30.0});

    void export_las(const std::string &path, const Pos &pos);

    // to body: X_b = R * X_l + t
    void set_body(const Vector3d &r, const Vector3d &t);
    void set_body(const std::string &config);

public:
    static std::tuple<Quaterniond, Vector3d> read_lidar_body(const std::string &path);

protected:
    // to body
    Matrix3d R_body;
    Vector3d t_body;

    std::string path_;
    std::shared_ptr<HsRawiLSP> hsfile_;

    std::string wkt_;

    std::vector<Vector3f> points_;
    std::vector<float> ranges_;
    std::vector<uint16_t> intens_;
    std::vector<double> gpssecs_;

    std::vector<uint8_t> header;
};

} // namespace h2o