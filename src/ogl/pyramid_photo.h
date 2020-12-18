/*!
 * \file ortho-photo.h
 *
 * \author Han
 * \date 2017/02/20
 *
 *
 */
#pragma once

#include <QOpenGLFunctions_3_3_Core>
#include <base/common.h>
#include <base/geo_image.h>

namespace h2o {

struct CameraParameters;

struct PyramidPhotoGpuData {
    std::vector<GLuint> textures;
    std::vector<MatrixXf> V;
    std::vector<MatrixXu> F;
    std::vector<MatrixXf> S;

    size_t size() { return textures.size(); }
    bool empty() { return size() == 0; }
};

class PyramidPhoto : public QOpenGLFunctions_3_3_Core {
public:
    PyramidPhoto(const GeoImagePtr &image, CameraParameters *camera, bool smooth = true);
    ~PyramidPhoto();

    int get_rows() const { return image_->get_image().rows; }
    int get_cols() const { return image_->get_image().cols; };
    void require_update(bool force_update = false) {
        need_update_ = true;
        force_update_ = force_update;
    }
    bool get_need_update() const { return need_update_; }

    void compute_gpu_textures();
    PyramidPhotoGpuData get_drawing_data() const;

protected:
    int compute_grid_size(int row, int col);
    Vector2i compute_grid_size_npot(int row, int col);
    void compute_gpu_vertices();

    void safe_delete(GLuint &texture);

protected:
    GeoImagePtr image_;
    const int GRID = 2048;
    const int MIN_GRID = 8;
    int image_rows_;
    int image_cols_;
    double nodata_;

    bool smooth_resample_; // 对非影像类型，只能采用最近邻采样
    bool need_update_;
    bool force_update_;

    Vector4f background_color_;
    ///\brief 对每个网格，存储当前网格的格网大小
    std::vector<int> levels_;
    PyramidPhotoGpuData gpu_data_;
    CameraParameters *camera_;
};

} // end h2o
