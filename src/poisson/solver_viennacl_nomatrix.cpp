/*
 * @Author: Han
 * @Date: 2018-02-08 11:41:57
 * 实现 poisson 求解的 matrix free 方式
 */
#include <poisson/solver.h>

#define VIENNACL_WITH_OPENCL 1
#include <viennacl/compressed_matrix.hpp>
#include <viennacl/linalg/cg.hpp>
#include <viennacl/matrix.hpp>
#include <viennacl/vector.hpp>
namespace h2o {

class PoissonOperator {
public:
    PoissonOperator(const viennacl::vector<int> &index) : A_index(index) { nnz = A_index.size() / 5; }

    // Dispatcher for y = Ax
    void apply(viennacl::vector_base<float> const &x, viennacl::vector_base<float> &y) const;
    std::size_t size1() const { return nnz; }

private:
    const viennacl::vector<int> &A_index;
    std::size_t nnz;
};

void PoissonOperator::apply(viennacl::vector_base<float> const &x, viennacl::vector_base<float> &y) const {
    viennacl::ocl::context &ctx = const_cast<viennacl::ocl::context &>(viennacl::traits::opencl_handle(x).context());
    viennacl::ocl::kernel &my_kernel = ctx.get_kernel("apply_opencl", "apply_opencl");
    viennacl::ocl::enqueue(my_kernel(x, y, A_index, static_cast<cl_int>(nnz)));
}

void solve_poisson_viennacl_nomatrix_opencl(const int *A_index, int nnz, const float *b, float *x,
                                            PoissonSolverOptions o) {
    viennacl::vector<int> vcl_A_index(nnz * 5);
    viennacl::copy(A_index, A_index + nnz * 5, vcl_A_index.begin());

    viennacl::vector<float> vcl_b(nnz);
    viennacl::copy(b, b + nnz, vcl_b.begin());

    viennacl::vector<float> vcl_x(nnz);
    viennacl::copy(x, x + nnz, vcl_x.begin());

    PoissonOperator op(vcl_A_index);

    viennacl::linalg::cg_tag solver_tag(o.threshold, o.max_iteration);
    viennacl::linalg::cg_solver<viennacl::vector<float>> solver(solver_tag);
    solver.set_initial_guess(vcl_x);
    vcl_x = solver(op, vcl_b);
    viennacl::copy(vcl_x.begin(), vcl_x.end(), x);
}
} // namespace h2o

namespace h2o {
void solve_poisson_viennacl_nomatrix(const int *A_index, int nnz, const float *b, float *x, PoissonSolverOptions op) {
    solve_poisson_viennacl_nomatrix_opencl(A_index, nnz, b, x, op);
}
} // namespace h2o
