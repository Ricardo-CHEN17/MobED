#pragma once

#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

namespace robot_control {
namespace dynamics {

/**
 * @brief Lightweight Quadratic Programming (QP) solver for small-scale problems.
 *
 * Solves problems of the form:
 *     min  0.5 * x^T * H * x + f^T * x
 *     s.t. lb <= x <= ub               (bound constraints)
 *          A_eq * x = b_eq             (equality constraints, optional)
 *
 * Designed for the MobED GRF optimization problem where:
 *   - Decision variables: 4 vertical ground reaction forces f_{z,i}
 *   - H encodes force-distribution cost + regularization
 *   - Equality constraints enforce Newton-Euler dynamics
 *   - Bounds enforce f_z >= 0 (no pulling from ground) and f_z <= f_max
 *
 * Implementation uses a projected gradient descent with active-set tracking,
 * suitable for real-time execution at 1kHz with only 4 variables.
 */
struct QpProblem {
    Eigen::MatrixXd H;      // n x n positive semi-definite cost matrix
    Eigen::VectorXd f;      // n x 1 linear cost vector
    Eigen::VectorXd lb;     // n x 1 lower bounds
    Eigen::VectorXd ub;     // n x 1 upper bounds
    Eigen::MatrixXd A_eq;   // m x n equality constraint matrix (can be empty)
    Eigen::VectorXd b_eq;   // m x 1 equality constraint rhs (can be empty)
};

struct QpResult {
    Eigen::VectorXd x;      // optimal solution
    double cost = 0.0;      // optimal objective value
    int iterations = 0;     // number of iterations used
    bool converged = false;  // whether the solver converged
};

/**
 * @brief Solve a small-scale box-constrained QP with optional equality constraints.
 *
 * For the MobED use case (n=4, m=2), this is extremely efficient.
 * Uses the KKT conditions with null-space method for equality constraints,
 * then projects the unconstrained solution onto the feasible box.
 *
 * @param problem  QP problem definition
 * @param max_iter Maximum iterations for iterative refinement (default: 50)
 * @param tol      Convergence tolerance (default: 1e-6)
 * @return QpResult with optimal solution and convergence info
 */
inline QpResult solveQP(const QpProblem& problem, int max_iter = 50, double tol = 1e-6) {
    QpResult result;
    int n = problem.H.rows();

    // Validate input dimensions
    if (problem.f.size() != n || problem.lb.size() != n || problem.ub.size() != n) {
        result.converged = false;
        result.x = Eigen::VectorXd::Zero(n);
        return result;
    }

    bool has_eq = (problem.A_eq.rows() > 0 && problem.A_eq.cols() == n);
    int m_eq = has_eq ? problem.A_eq.rows() : 0;

    if (!has_eq) {
        // ================================================================
        // Case 1: Box-constrained QP only (no equality constraints)
        // Projected gradient descent with Barzilai-Borwein step size
        // ================================================================
        Eigen::VectorXd x = 0.5 * (problem.lb + problem.ub);  // initialize at center

        for (int iter = 0; iter < max_iter; ++iter) {
            // Gradient: g = H*x + f
            Eigen::VectorXd g = problem.H * x + problem.f;

            // Projected gradient step
            // Use a conservative step size based on H's diagonal
            double step = 1.0 / problem.H.diagonal().maxCoeff();

            Eigen::VectorXd x_new = x - step * g;

            // Project onto bounds
            for (int i = 0; i < n; ++i) {
                x_new(i) = std::clamp(x_new(i), problem.lb(i), problem.ub(i));
            }

            // Check convergence
            double dx = (x_new - x).norm();
            x = x_new;

            if (dx < tol) {
                result.converged = true;
                result.iterations = iter + 1;
                break;
            }
        }

        result.x = x;
        result.cost = 0.5 * x.dot(problem.H * x) + problem.f.dot(x);
        if (!result.converged) result.iterations = max_iter;
        return result;
    }

    // ================================================================
    // Case 2: QP with equality constraints + box constraints
    // Use KKT system with iterative projection (ADMM-like)
    // ================================================================

    // Build the KKT system:
    //   [ H    A_eq^T ] [ x     ]   [ -f    ]
    //   [ A_eq   0    ] [ lambda ] = [  b_eq ]
    int kkt_size = n + m_eq;
    Eigen::MatrixXd KKT = Eigen::MatrixXd::Zero(kkt_size, kkt_size);
    KKT.block(0, 0, n, n) = problem.H;
    KKT.block(0, n, n, m_eq) = problem.A_eq.transpose();
    KKT.block(n, 0, m_eq, n) = problem.A_eq;

    Eigen::VectorXd rhs(kkt_size);
    rhs.head(n) = -problem.f;
    rhs.tail(m_eq) = problem.b_eq;

    // Solve unconstrained KKT
    Eigen::VectorXd sol = KKT.ldlt().solve(rhs);
    Eigen::VectorXd x_unc = sol.head(n);

    // Check if unconstrained solution satisfies bounds
    bool all_feasible = true;
    for (int i = 0; i < n; ++i) {
        if (x_unc(i) < problem.lb(i) - tol || x_unc(i) > problem.ub(i) + tol) {
            all_feasible = false;
            break;
        }
    }

    if (all_feasible) {
        // Unconstrained optimum is already feasible — we're done
        result.x = x_unc;
        result.cost = 0.5 * x_unc.dot(problem.H * x_unc) + problem.f.dot(x_unc);
        result.converged = true;
        result.iterations = 1;
        return result;
    }

    // Iterative projection: alternate between solving KKT and projecting onto bounds
    // This is a simplified ADMM approach suitable for our small problem size
    double rho = problem.H.diagonal().mean();  // penalty parameter
    Eigen::VectorXd z = 0.5 * (problem.lb + problem.ub);  // auxiliary variable
    Eigen::VectorXd u = Eigen::VectorXd::Zero(n);          // dual variable

    // Augmented KKT for ADMM
    Eigen::MatrixXd KKT_aug = Eigen::MatrixXd::Zero(kkt_size, kkt_size);
    KKT_aug.block(0, 0, n, n) = problem.H + rho * Eigen::MatrixXd::Identity(n, n);
    KKT_aug.block(0, n, n, m_eq) = problem.A_eq.transpose();
    KKT_aug.block(n, 0, m_eq, n) = problem.A_eq;

    Eigen::LDLT<Eigen::MatrixXd> kkt_solver(KKT_aug);

    Eigen::VectorXd x = z;
    for (int iter = 0; iter < max_iter; ++iter) {
        // x-update: solve augmented KKT
        Eigen::VectorXd rhs_aug(kkt_size);
        rhs_aug.head(n) = -problem.f + rho * (z - u);
        rhs_aug.tail(m_eq) = problem.b_eq;

        Eigen::VectorXd sol_aug = kkt_solver.solve(rhs_aug);
        x = sol_aug.head(n);

        // z-update: project (x + u) onto bounds
        Eigen::VectorXd z_old = z;
        for (int i = 0; i < n; ++i) {
            z(i) = std::clamp(x(i) + u(i), problem.lb(i), problem.ub(i));
        }

        // u-update: dual ascent
        u += x - z;

        // Check convergence (primal and dual residuals)
        double primal_res = (x - z).norm();
        double dual_res = rho * (z - z_old).norm();

        if (primal_res < tol && dual_res < tol) {
            result.converged = true;
            result.iterations = iter + 1;
            break;
        }
    }

    result.x = z;  // z is the feasible solution
    result.cost = 0.5 * z.dot(problem.H * z) + problem.f.dot(z);
    if (!result.converged) result.iterations = max_iter;
    return result;
}

}  // namespace dynamics
}  // namespace robot_control
