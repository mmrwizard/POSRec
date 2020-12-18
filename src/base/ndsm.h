/*
 * @Author: Han
 * @Date: 2017-12-11 19:52:50
 * DSM 和 DEM 之差
 */
#pragma once

#include <base/common.h>
#include <base/geo_image.h>

namespace h2o {
class NDsm {
public:
    NDsm(const GeoImagePtr &dsm, const GeoImagePtr &dem, const cv::Mat &mask, const cv::Vec4b &background, bool absolute = false);
    ~NDsm();

    // 计算或者更新，如果 bb 是空的则是根据 DSM DEM 重新计算，如果非空则更新
    void compute(BoundingBox2i rect = BoundingBox2i());
    void set_fixed_range(bool fixed, double range);

    // 设置颜色信息
    void set_colors(const std::vector<Vector3i> &rgb, const Vector3i &zero);

    GeoImagePtr get_ndsm_color() const { return ndsm_color_; }
    GeoImagePtr get_ndsm() const { return ndsm_; }

    // 获取用于绘制 legend 的信息，最大最小值 和 color bar
    std::tuple<Vector2f, std::vector<Vector3f>> get_lengend() const;
    void set_alpha(int v) { alpha_ = v; };

protected:
    // 计算最大最小高程，是 step 的 整数倍
    Vector2f compute_min_max(float min_v, float max_v);
    // 根据高差值给 ndsm 着色
    cv::Mat colorize_ndsm(const cv::Mat &ndsm, const cv::Mat &mask);

protected:
    GeoImagePtr dsm_;
    GeoImagePtr dem_;
    GeoImagePtr ndsm_;
    GeoImagePtr ndsm_color_;
    const cv::Mat &nodata_mask_;
    cv::Vec4b background_;

    std::vector<Vector3f> colors_;
    Vector3f color_below_zero_;

    // 最大最小值 是否固定
    bool fixed_;

    // 最大最小DEM 差异，以 10 为间隔更新
    Vector2f minmax_;

    // 最大最小值 需要是 step 的倍数
    float step_;

    // alpha 值，控制透明度
    int alpha_;

    // 是否只用绝对值
    bool absolute_;
};
} // namespace h2o
