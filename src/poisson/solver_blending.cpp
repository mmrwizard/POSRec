/*
 * @Author: Han
 * @Date: 2018-02-05 20:50:41
 * 解求 poisson blending 问题
 */

#include <poisson/solver.h>

#include <chrono>

namespace h2o {

template <typename T> const T *mat_row_ptr(const cv::Mat &mat, int r) {
    if (r >= 0 && r < mat.rows) {
        return mat.ptr<T>(r);
    }
    return nullptr;
}

cv::Mat poisson_blending_nomatrix(const cv::Mat &src, const cv::Mat &guide, const cv::Mat &mask,
                                  PoissonSolverOptions op) {
    CHECK(src.rows == guide.rows && src.rows == mask.rows && src.cols == guide.cols && src.cols == mask.cols)
        << "size should be the same";
    CHECK(src.type() == CV_32F && guide.type() == CV_32F && mask.type() == CV_8U) << "image type is not correct";

    cv::Mat blended;
    int rows = src.rows;
    int cols = src.cols;

    cv::Mat indices = cv::Mat(rows, cols, CV_32S, -1);
    // 初始化每个像素的在未知数中的index，只有Fixed和boundary 优化
    int index = 0;
    {
        const uint8_t *mask_ptr = mask.data;
        int *index_ptr = (int *)indices.data;
        for (int i = 0; i < rows * cols; ++i) {
            if (mask_ptr[i] == GUIDED) {
                index_ptr[i] = index++;
            }
        }
    }

    const int nnz = index;
    if (nnz == 0) {
        return src;
    }

    // Ax = b，其中x是每个非ignored像素
    std::vector<int> A_index(nnz * 5);

    VectorXf b = VectorXf::Zero(nnz);
    VectorXf x = VectorXf::Zero(nnz);

// 填充矩阵A b
#pragma omp parallel for
    for (int r = 0; r < rows; ++r) {
        const uint8_t *mask_r0 = mat_row_ptr<uint8_t>(mask, r - 1);
        const uint8_t *mask_r1 = mat_row_ptr<uint8_t>(mask, r);
        const uint8_t *mask_r2 = mat_row_ptr<uint8_t>(mask, r + 1);

        const int *index_r0 = mat_row_ptr<int>(indices, r - 1);
        const int *index_r1 = mat_row_ptr<int>(indices, r);
        const int *index_r2 = mat_row_ptr<int>(indices, r + 1);

        const float *guide_ptr = guide.ptr<float>(r);

        const float *src_r0 = mat_row_ptr<float>(src, r - 1);
        const float *src_r1 = mat_row_ptr<float>(src, r);
        const float *src_r2 = mat_row_ptr<float>(src, r + 1);

        for (int c = 0; c < cols; ++c) {
            if (mask_r1[c] != GUIDED) {
                continue;
            }

            CHECK(mask_r0 && mask_r1 && mask_r2);

            /*
             *     01
             *     |
             *  10-11-12
             *     |
             *     21
             */

            int i01 = index_r0[c];
            int i10 = index_r1[c - 1];
            int i11 = index_r1[c];
            int i12 = index_r1[c + 1];
            int i21 = index_r2[c];

            x[i11] = src_r1[c]; // 初始化未知数，初始化为当前DEM
            b[i11] = guide_ptr[c];

            // 对 Np 在 边界上的情况，需要加上边界值
            CHECK(mask_r0[c] != 0 && mask_r1[c - 1] != 0 && mask_r1[c + 1] != 0 && mask_r2[c] != 0);

            A_index[i11 * 5 + 0] = i11;
            A_index[i11 * 5 + 1] = i01;
            A_index[i11 * 5 + 2] = i10;
            A_index[i11 * 5 + 3] = i12;
            A_index[i11 * 5 + 4] = i21;

            if (mask_r0[c] == BOUNDARY) {
                b[i11] += src_r0[c];
            }
            if (mask_r1[c - 1] == BOUNDARY) {
                b[i11] += src_r1[c - 1];
            }
            if (mask_r1[c + 1] == BOUNDARY) {
                b[i11] += src_r1[c + 1];
            }
            if (mask_r2[c] == BOUNDARY) {
                b[i11] += src_r2[c];
            }
        }
    }

    blended = src.clone();

    solve_poisson_viennacl_nomatrix(A_index.data(), nnz, b.data(), x.data(), op);

#pragma omp parallel for
    for (int r = 0; r < rows; ++r) {
        const int *indices_ptr = indices.ptr<int>(r);
        float *blended_ptr = blended.ptr<float>(r);
        for (int c = 0; c < cols; ++c) {
            int idx = indices_ptr[c];
            if (idx != -1) {
                blended_ptr[c] = x[idx];
            }
        }
    }
    return blended;
}
} // namespace h2o
