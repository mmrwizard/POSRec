/*
 * @Author: Han Hu
 * @Date: 2020-12-18 21:27:51
 * @Description: 读取 HSR 文件，结合 POS 输出信息
 */

#pragma once

#include "ladybug_stream.h"
#include "pos.h"

#include <base/common.h>
#include <chrono>

class HsRawiLSP;

using laszip_POINTER = void *;

namespace h2o {
class HsrFile {
public:
    struct HsrFileConfig {
        double speed = 1.0;
        std::vector<double> rtime;
        std::vector<double> rdist = {1.5, 30.0};

        std::string wkt;
    };
    static HsrFileConfig load_config(const std::string &path);

public:
    HsrFile(const std::string &path);
    ~HsrFile();

    // 获取文件获取的实践，点云数据存储的是 UTC 天内秒，
    std::tm get_file_date();

    // 把 HSR 文件全部读入内存
    void load();
    // 把 HSR 文件全部读入内存，同时输入 POS 信息可判断速度信息
    void load_with_pos(const Pos &pos);

    // 导出点云数据，需要把所有信息都先读入内存
    void export_las(const std::string &path, const Pos &pos) const;

    // 采用 out-of-core 方式，不需全部读入内存，直接导出
    void export_las_ooc(const std::string &path, const Pos &pos) const;

    // 导出带颜色的点云
    void export_colored_las_ooc(const std::string &path, const Pos &pos, const LadybugStreamPtr &ladybug) const;

    // to body: X_b = R * X_l + t
    void set_body(const Vector3d &r, const Vector3d &t);
    void set_body(const std::string &config);

    void set_config(const HsrFileConfig &config) { config_ = config; }
    void set_config(const std::string& path);
protected:
    laszip_POINTER create_lasfile_header(const std::string &path, const Vector3d &offset,
                                         const BoundingBox3d &bounds) const;
    Vector2d get_intensity_range() const;
    bool is_point_good(double range, double gpstime, double speed = DBL_MAX) const;

public:
    static std::tuple<Quaterniond, Vector3d> read_lidar_body(const std::string &path);

protected:
    // to body
    Matrix3d R_body;
    Vector3d t_body;

    std::string path_;
    std::shared_ptr<HsRawiLSP> hsfile_;

    std::vector<Vector3f> points_;
    std::vector<uint16_t> intens_;
    std::vector<double> gpssecs_;

    std::vector<uint8_t> header;

    HsrFileConfig config_;
};

} // namespace h2o
