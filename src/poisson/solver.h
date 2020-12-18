/*
 * @Author: Han
 * @Date: 2018-02-05 14:26:26
 * 泊松方程求解
 */
#pragma once

#include <Eigen/SparseCore>
#include <base/common.h>

namespace h2o {
enum PoissonType { IGNORED = 0, GUIDED = 128, BOUNDARY = 255 };
enum SolverType { EigenCG, ViennaclCG };

typedef Eigen::SparseMatrix<float> SparseMatrixf;
typedef Eigen::Triplet<float, int> Tripletf;

struct PoissonSolverOptions {
    int max_iteration = 50;
    float threshold = 1e-4f;

    double w_value = 1000.0;
    double w_gradient = 1.0;
    double w_lap = 0.0;
    double w_bilap = 1.0;
};

// 采用 eigen 的 cholesky 分解求解x
VectorXf solve_poisson_eigen(const SparseMatrixf &A, const VectorXf &b);

// 采用 eigen 的 CG 求解 x
void solve_poisson_cg(const SparseMatrixf &A, const VectorXf &b, VectorXf &x,
                      PoissonSolverOptions options = PoissonSolverOptions());

// 采用 viennacl 的 cg 求解 x，需要给定初始值
void solve_poisson_viennacl(const SparseMatrixf &A, const VectorXf &b, VectorXf &x,
                            PoissonSolverOptions options = PoissonSolverOptions());

void solve_poisson_viennacl_nomatrix(const int *A_index, int nnz, const float *b, float *x,
                                     PoissonSolverOptions options = PoissonSolverOptions());

// 泊松融合
cv::Mat poisson_blending_nomatrix(const cv::Mat &src, const cv::Mat &guide, const cv::Mat &mask,
                                  PoissonSolverOptions options = PoissonSolverOptions());

// 采用 am 进行内插
cv::Mat poisson_interpolation_am(const cv::Mat &src, const cv::Mat &mask, PoissonSolverOptions options = PoissonSolverOptions());
// 用 value 和 gradients 一起进行内插，ignored 和 boundary 都用 value， boundary 和 guided 都用 gradients
cv::Mat poisson_interpolation_am(const cv::Mat &src, const cv::Mat &gradients, const cv::Mat &mask, PoissonSolverOptions options = PoissonSolverOptions());

} // namespace h2o
