/*!
 * \file ortho-photo.cpp
 *
 * \author Han
 * \date 2017/02/20
 *
 * 在 CPU 里面生成金字塔影像供 GPU 绘制
 */

#include <base/common.h>
#include <base/gdal_wrapper.h>
#include <base/imgproc.h>
#include <ogl/camera.h>
#include <ogl/pyramid_photo.h>

#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace h2o {

PyramidPhoto::PyramidPhoto(const GeoImagePtr &image, CameraParameters *camera, bool smooth) {
    camera_ = camera;
    image_ = image;
    image_rows_ = image->get_image().rows;
    image_cols_ = image->get_image().cols;

    compute_gpu_vertices();
    smooth_resample_ = smooth;
    need_update_ = true;
    force_update_ = true;

    CHECK(initializeOpenGLFunctions());
}

PyramidPhoto::~PyramidPhoto() {
    for (int i = 0; i < gpu_data_.size(); ++i) {
        safe_delete(gpu_data_.textures[i]);
    }
}

void PyramidPhoto::compute_gpu_textures() {
    if (need_update_) {
        int rows = int(image_rows_ / GRID) + 1;
        int cols = int(image_cols_ / GRID) + 1;

        if ((image_rows_ % GRID) == 0) {
            rows--;
        }
        if ((image_cols_ % GRID) == 0) {
            cols--;
        }

        auto generate_texture = [&](const cv::Mat &image, GLint internal_format, GLenum format, GLenum type) {
            GLuint texID = 0;
            glGenTextures(1, &texID);
            glBindTexture(GL_TEXTURE_2D, texID);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexImage2D(GL_TEXTURE_2D, 0, internal_format, image.cols, image.rows, 0, format, type, image.data);
            glBindTexture(GL_TEXTURE_2D, 0);

            CHECK(GL_NO_ERROR == glGetError()) << "opengl error found";
            return texID;
        };
        if (gpu_data_.empty()) {
            gpu_data_.textures.resize(rows * cols, 0);
            levels_.resize(rows * cols, -1);
        }
        CHECK(rows * cols == levels_.size()) << "texture size is not the same";
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; c++) {
                int texture_size = compute_grid_size(r, c);
                int idx = r * cols + c;
                if (texture_size != levels_[idx] || force_update_) {
                    safe_delete(gpu_data_.textures[idx]);
                    // 重新生成
                    cv::Range row_range, col_range;
                    col_range.start = c * GRID;
                    col_range.end = (c + 1) * GRID;
                    row_range.start = r * GRID;
                    row_range.end = (r + 1) * GRID;
                    if (col_range.end > image_cols_) {
                        col_range.end = image_cols_;
                    }
                    if (row_range.end > image_rows_) {
                        row_range.end = image_rows_;
                    }
                    CHECK(row_range.size() > 0 && col_range.size() > 0) << "subset image is empty";

                    cv::Mat sub_image = image_->get_image()(row_range, col_range).clone();
                    if (smooth_resample_) {
                        sub_image = smooth_resize(sub_image, {texture_size, texture_size});
                    } else {
                        cv::resize(sub_image, sub_image, {texture_size, texture_size}, 0.0, 0.0, cv::INTER_NEAREST);
                    }

                    if (sub_image.type() == CV_8UC3) {
                        gpu_data_.textures[idx] = generate_texture(sub_image, GL_RGB, GL_RGB, GL_UNSIGNED_BYTE);
                    } else if (sub_image.type() == CV_8UC4) {
                        gpu_data_.textures[idx] = generate_texture(sub_image, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE);
                    }
                    levels_[idx] = texture_size;
                }
            }
        }
        need_update_ = false;
        force_update_ = false;
    }
}

PyramidPhotoGpuData PyramidPhoto::get_drawing_data() const { return gpu_data_; }

void PyramidPhoto::safe_delete(GLuint &texture) {
    if (texture != 0 && glIsTexture(texture)) {
        glDeleteTextures(1, &texture);
        texture = 0;
        CHECK(glGetError() == GL_NO_ERROR) << "Opengl functions failed";
    }

    CHECK(glGetError() == GL_NO_ERROR) << "Opengl functions failed";
}

int PyramidPhoto::compute_grid_size(int row, int col) {
    /*
     * 0 - 1
     * |   |
     * 3 - 2
     */
    int minx = col * GRID;
    int maxx = (col + 1) * GRID;
    int miny = row * GRID;
    int maxy = (row + 1) * GRID;
    if (maxx > image_cols_) {
        maxx = image_cols_;
    }
    if (maxy > image_rows_) {
        maxy = image_rows_;
    }
    CHECK(maxx > minx && maxy > miny) << "this cell is empty";
    Vector2f corners[4];
    corners[0] = Vector2f(minx, miny);
    corners[1] = Vector2f(maxx, miny);
    corners[2] = Vector2f(maxx, maxy);
    corners[3] = Vector2f(minx, maxy);

    // 根据当前 raster 的行列号，计算实际坐标，然后投影到屏幕坐标，根据屏幕坐标的大小判断输出的大小
    // 如果落入外部，则用最小值
    Vector2f corners_projection[4];
    int idx = 0;

    BoundingBox2f screen_bbox;
    for (const auto &corner_pixel : corners) {
        Vector2f point_xy = image_->pixel_to_projection(corner_pixel.cast<double>()).cast<float>();
        screen_bbox.extend(
            camera_->convert_3d_to_2d(Vector3f{point_xy(0), point_xy(1), camera_->scene_bbox.center().z()}));
    };

    BoundingBox2f screen_size;
    screen_size.extend(Vector2f(0.0f, 0.0f));
    screen_size.extend(camera_->size.cast<float>());

    // 如果端点落于外部
    if (!screen_bbox.intersects(screen_size)) {
        return MIN_GRID;
    }

    // 计算screen的大小
    // 最小是8，最大是GRID
    int min_power = int(log2(MIN_GRID) + 0.5), max_power = int(log2(GRID) + 0.5);
    float width = screen_bbox.sizes().x();
    float height = screen_bbox.sizes().y();

    float max_length = std::max(width, height);

    int power = int(std::log2(max_length) + 0.5);
    if (power < min_power) {
        power = min_power;
    }
    if (power > max_power) {
        power = max_power;
    }
    return int(std::pow(2, power) + 0.5);
}

Vector2i PyramidPhoto::compute_grid_size_npot(int row, int col) {
    /*
    * 0 - 1
    * |   |
    * 3 - 2
    */
    int minx = col * GRID;
    int maxx = (col + 1) * GRID;
    int miny = row * GRID;
    int maxy = (row + 1) * GRID;
    if (maxx > image_cols_) {
        maxx = image_cols_;
    }
    if (maxy > image_rows_) {
        maxy = image_rows_;
    }
    CHECK(maxx > minx && maxy > miny) << "this cell is empty";
    Vector2f corners[4];
    corners[0] = Vector2f(minx, miny);
    corners[1] = Vector2f(maxx, miny);
    corners[2] = Vector2f(maxx, maxy);
    corners[3] = Vector2f(minx, maxy);

    // 根据当前 raster 的行列号，计算实际坐标，然后投影到屏幕坐标，根据屏幕坐标的大小判断输出的大小
    // 如果落入外部，则用最小值
    Vector2f corners_projection[4];
    int idx = 0;

    BoundingBox2f screen_bbox;
    for (const auto &corner_pixel : corners) {
        Vector2f point_xy = image_->pixel_to_projection(corner_pixel.cast<double>()).cast<float>();
        screen_bbox.extend(camera_->convert_3d_to_2d({point_xy(0), point_xy(1), camera_->scene_bbox.center().z()}));
    };

    BoundingBox2f screen_size;
    screen_size.extend(Vector2f(0.0f, 0.0f));
    screen_size.extend(camera_->size.cast<float>());

    // 如果端点落于外部
    if (!screen_bbox.intersects(screen_size)) {
        return {MIN_GRID, MIN_GRID};
    }

    // 计算screen的大小
    // 最小是8，最大是GRID
    int min_power = int(log2(MIN_GRID) + 0.5), max_power = int(log2(GRID) + 0.5);
    float width = screen_bbox.sizes().x();
    float height = screen_bbox.sizes().y();

    Vector2i size;
    float max_length = std::max(width, height);

    int power = int(std::log2(width) + 0.5);
    if (power < min_power) {
        power = min_power;
    }
    if (power > max_power) {
        power = max_power;
    }

    size(0) = int(std::pow(2, power) + 0.5);

    power = int(std::log2(height) + 0.5);
    if (power < min_power) {
        power = min_power;
    }
    if (power > max_power) {
        power = max_power;
    }
    size(1) = int(std::pow(2, power) + 0.5);

    return size;
}

void PyramidPhoto::compute_gpu_vertices() {
    PyramidPhotoGpuData &data = gpu_data_;

    int rows, cols;
    rows = int(image_rows_ / GRID) + 1;
    cols = int(image_cols_ / GRID) + 1;
    if (image_rows_ % GRID == 0) {
        rows--;
    }
    if (image_cols_ % GRID == 0) {
        cols--;
    }
    data.V.reserve(rows * cols);
    data.F.reserve(rows * cols);
    data.S.reserve(rows * cols);

    Matrix23d xform = image_->get_xform();
    // 生成的texture是在影像坐标系，也就是原点在左上角
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            cv::Range row_range;
            cv::Range col_range;
            row_range.start = r * GRID;
            row_range.end = (r + 1) * GRID;
            if (row_range.end > image_rows_) {
                row_range.end = image_rows_;
            }

            col_range.start = c * GRID;
            col_range.end = (c + 1) * GRID;
            if (col_range.end > image_cols_) {
                col_range.end = image_cols_;
            }

            MatrixXf V(3, 4);
            MatrixXu F(3, 2);
            MatrixXf S(2, 4);

            /*
            * (1,2,3) + (1, 3, 4)
            *  4 - 3
            *  | / |
            *  1 - 2
            */
            int minx = col_range.start;
            int maxx = col_range.end;
            int miny = row_range.start;
            int maxy = row_range.end;

            Vector2d corners[4];
            corners[0] = Vector2d(minx, maxy);
            corners[1] = Vector2d(maxx, maxy);
            corners[2] = Vector2d(maxx, miny);
            corners[3] = Vector2d(minx, miny);

            Vector2d corners_projection[4];
            int idx = 0;
            for (const auto &corner_pixel : corners) {
                corners_projection[idx++] = image_->pixel_to_projection(corner_pixel);
            }

            double center_z = camera_->scene_bbox.center().z();

            // opengl 是左下角的左下 为 0.0, 0.0，但是 geoimage 是左上角的中心为原点
            for (int i = 0; i < 4; ++i) {
                Vector2d pt = Vector2d(corners_projection[i].x(), corners_projection[i].y());
                pt(0) += xform(0, 0) * (-0.5);
                pt(1) += xform(1, 1) * (-0.5);
                V.col(i) = Vector3f(pt(0), pt(1), center_z);
            }

            F.col(0) << 0, 1, 2;
            F.col(1) << 2, 3, 0;

            S.col(0) << 0.0f, 1.0f;
            S.col(1) << 1.0f, 1.0f;
            S.col(2) << 1.0f, 0.0f;
            S.col(3) << 0.0f, 0.0f;

            data.V.push_back(V);
            data.F.push_back(F);
            data.S.push_back(S);
        }
    }
}
} // end h2o
