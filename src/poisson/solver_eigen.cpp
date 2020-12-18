/*
 * @Author: Han
 * @Date: 2018-02-05 14:26:34
 * 泊松方程求解
 */
#include <poisson/solver.h>

#include <Eigen/Core>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>

#include <omp.h>

namespace h2o {

VectorXf solve_poisson_eigen(const SparseMatrixf &A, const VectorXf &b) {
    typedef Eigen::SimplicialLDLT<SparseMatrixf> Solver;
    Solver solver;
    solver.compute(A);
    CHECK(solver.info() == Eigen::Success) << "Solve failed";

    VectorXf x = solver.solve(b);
    return x;
}

void solve_poisson_cg(const SparseMatrixf &A, const VectorXf &b, VectorXf &x, PoissonSolverOptions options) {
    typedef Eigen::ConjugateGradient<SparseMatrixf> Solver;
    Eigen::initParallel();
    Eigen::setNbThreads(omp_get_num_procs());
    Solver solver;
    solver.compute(A);
    CHECK(solver.info() == Eigen::Success) << "Solve failed";
    solver.setMaxIterations(options.max_iteration);
    solver.setTolerance(options.threshold);
    VectorXf x1 = solver.solveWithGuess(b, x);
    x = x1;
    Eigen::setNbThreads(0);
}

} // namespace h2o
