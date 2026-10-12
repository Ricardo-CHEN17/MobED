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
 * @brief Solve QP using the Goldfarb-Idnani Dual Active-Set Algorithm (QuadProg++).
 *
 * Paper Ref [29]: D. Goldfarb and A. Idnani, "A numerically stable dual method
 * for solving strictly convex quadratic programs," Mathematical Programming, 1983.
 *
 * Solves:
 *     min  0.5 * x^T * H * x + f^T * x
 *     s.t. lb <= x <= ub               (box inequality constraints)
 *          A_eq * x = b_eq             (optional equality constraints)
 *
 * Computes the exact global KKT minimum in a finite number of active-set pivots.
 * For small problems (n <= 4), converges in 1-4 iterations with machine precision.
 *
 * @param problem  QP problem definition
 * @param max_iter Maximum active-set pivot iterations (default: 100)
 * @param tol      Constraint violation tolerance (default: 1e-8)
 * @return QpResult with optimal solution and convergence info
 */
inline QpResult solveQP(const QpProblem& problem, int max_iter = 100, double tol = 1e-8) {
    QpResult result;
    int n = static_cast<int>(problem.H.rows());

    // Validate input dimensions
    if (problem.f.size() != n || problem.lb.size() != n || problem.ub.size() != n) {
        result.converged = false;
        result.x = Eigen::VectorXd::Zero(n);
        return result;
    }

    bool has_eq = (problem.A_eq.rows() > 0 && problem.A_eq.cols() == n);
    int m_eq = has_eq ? static_cast<int>(problem.A_eq.rows()) : 0;

    // Inequality constraints: C_I^T * x >= b_I
    //   [  I ] x >= lb
    //   [ -I ] x >= -ub
    int m_ineq = 2 * n;
    Eigen::MatrixXd CI(n, m_ineq);
    CI.block(0, 0, n, n) = Eigen::MatrixXd::Identity(n, n);
    CI.block(0, n, n, n) = -Eigen::MatrixXd::Identity(n, n);

    Eigen::VectorXd bI(m_ineq);
    bI.head(n) = problem.lb;
    bI.tail(n) = -problem.ub;

    // Invert positive-definite Hessian H
    Eigen::LDLT<Eigen::MatrixXd> ldlt_H(problem.H);
    if (ldlt_H.info() != Eigen::Success) {
        result.converged = false;
        result.x = Eigen::VectorXd::Zero(n);
        return result;
    }
    Eigen::MatrixXd H_inv = ldlt_H.solve(Eigen::MatrixXd::Identity(n, n));

    Eigen::VectorXd x;
    std::vector<int> A;  // active inequality constraint indices
    Eigen::VectorXd u = Eigen::VectorXd::Zero(m_ineq);

    if (!has_eq) {
        x = -H_inv * problem.f;
    } else {
        // Solve unconstrained KKT with equality constraints alone
        int kkt_size = n + m_eq;
        Eigen::MatrixXd KKT = Eigen::MatrixXd::Zero(kkt_size, kkt_size);
        KKT.block(0, 0, n, n) = problem.H;
        KKT.block(0, n, n, m_eq) = problem.A_eq.transpose();
        KKT.block(n, 0, m_eq, n) = problem.A_eq;

        Eigen::VectorXd rhs(kkt_size);
        rhs.head(n) = -problem.f;
        rhs.tail(m_eq) = problem.b_eq;

        Eigen::VectorXd sol = KKT.ldlt().solve(rhs);
        x = sol.head(n);
    }

    for (int it = 0; it < max_iter; ++it) {
        // Evaluate violations: viol_j = b_I(j) - c_j^T * x
        Eigen::VectorXd viol = bI - CI.transpose() * x;
        for (int a_idx : A) {
            viol(a_idx) = -1e10;  // already active
        }

        int p = 0;
        double max_viol = viol.maxCoeff(&p);
        if (max_viol <= tol) {
            result.x = x;
            result.cost = 0.5 * x.dot(problem.H * x) + problem.f.dot(x);
            result.converged = true;
            result.iterations = it + 1;
            return result;
        }

        // Inner dual active-set pivoting loop for constraint p
        while (true) {
            Eigen::VectorXd c_p = CI.col(p);
            int m_active = m_eq + static_cast<int>(A.size());

            if (m_active == 0) {
                Eigen::VectorXd d = H_inv * c_p;
                double denom = c_p.dot(d);
                double t1 = (denom > 1e-14) ? ((bI(p) - c_p.dot(x)) / denom) : 1e20;
                x += t1 * d;
                u(p) = t1;
                A.push_back(p);
                break;
            } else {
                Eigen::MatrixXd N(n, m_active);
                if (has_eq) {
                    N.block(0, 0, n, m_eq) = problem.A_eq.transpose();
                }
                for (size_t i = 0; i < A.size(); ++i) {
                    N.col(m_eq + static_cast<int>(i)) = CI.col(A[i]);
                }

                Eigen::MatrixXd M = N.transpose() * H_inv * N;
                Eigen::VectorXd r = M.ldlt().solve(N.transpose() * H_inv * c_p);
                Eigen::VectorXd d = H_inv * (c_p - N * r);

                double denom = c_p.dot(d);
                double t1 = (denom > 1e-14) ? ((bI(p) - c_p.dot(x)) / denom) : 1e20;

                double t2 = 1e20;
                int k_drop_idx = -1;
                for (size_t i = 0; i < A.size(); ++i) {
                    int a_idx = A[i];
                    double r_val = r(m_eq + static_cast<int>(i));
                    if (r_val > 1e-12) {
                        double step_k = u(a_idx) / r_val;
                        if (step_k < t2) {
                            t2 = step_k;
                            k_drop_idx = static_cast<int>(i);
                        }
                    }
                }

                if (t1 >= 1e19 && t2 >= 1e19) {
                    // Infeasible or degenerate dual direction
                    result.x = x;
                    result.cost = 0.5 * x.dot(problem.H * x) + problem.f.dot(x);
                    result.converged = false;
                    result.iterations = it + 1;
                    return result;
                }

                double step = std::min(t1, t2);
                x += step * d;
                for (size_t i = 0; i < A.size(); ++i) {
                    u(A[i]) -= step * r(m_eq + static_cast<int>(i));
                }

                if (t1 <= t2) {
                    u(p) = step;
                    A.push_back(p);
                    break;
                } else {
                    int dropped_a = A[k_drop_idx];
                    A.erase(A.begin() + k_drop_idx);
                    u(dropped_a) = 0.0;
                }
            }
        }
    }

    result.x = x;
    result.cost = 0.5 * x.dot(problem.H * x) + problem.f.dot(x);
    result.converged = false;
    result.iterations = max_iter;
    return result;
}

}  // namespace dynamics
}  // namespace robot_control
