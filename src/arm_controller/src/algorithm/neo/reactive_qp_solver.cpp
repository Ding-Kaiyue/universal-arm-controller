#include "reactive_qp_solver.hpp"

#include <Eigen/SparseCore>

#include "reactive_qp_validator.hpp"

namespace arm_controller::algorithm::reactive_qp {

ReactiveQpSolver::ReactiveQpSolver(ReactiveQpSolverConfig config)
    : config_(config) {}

bool ReactiveQpSolver::configureSolver(const ReactiveQpProblem& problem) {
    const int nv = problem.numVariables();
    const int nc = problem.numConstraints();

    solver_.settings()->setWarmStart(config_.warm_start);
    solver_.settings()->setVerbosity(config_.verbose);
    solver_.settings()->setMaxIteration(config_.max_iterations);
    solver_.settings()->setAbsoluteTolerance(config_.absolute_tolerance);
    solver_.settings()->setRelativeTolerance(config_.relative_tolerance);

    solver_.clearSolver();
    solver_.data()->setNumberOfVariables(nv);
    solver_.data()->setNumberOfConstraints(nc);
    if (initialized_) {
        // OsqpEigen data object keeps matrix-set state across solves.
        // Clear explicitly before setting new QP matrices in control loops.
        solver_.data()->clearHessianMatrix();
        solver_.data()->clearLinearConstraintsMatrix();
    }

    const Eigen::SparseMatrix<double> h_sparse = problem.hessian.sparseView();
    const Eigen::SparseMatrix<double> a_sparse = problem.constraint_matrix.sparseView();
    Eigen::VectorXd gradient = problem.gradient;
    Eigen::VectorXd lower_bound = problem.lower_bound;
    Eigen::VectorXd upper_bound = problem.upper_bound;

    if (!solver_.data()->setHessianMatrix(h_sparse)) {
        return false;
    }
    if (!solver_.data()->setGradient(gradient)) {
        return false;
    }
    if (!solver_.data()->setLinearConstraintsMatrix(a_sparse)) {
        return false;
    }
    if (!solver_.data()->setLowerBound(lower_bound)) {
        return false;
    }
    if (!solver_.data()->setUpperBound(upper_bound)) {
        return false;
    }

    if (!solver_.initSolver()) {
        return false;
    }

    cached_num_variables_ = nv;
    cached_num_constraints_ = nc;
    initialized_ = true;
    return true;
}

bool ReactiveQpSolver::solve(
    const ReactiveQpProblem& problem,
    Eigen::VectorXd& out_solution,
    std::string* error) {
    if (!ReactiveQpValidator::validateProblem(problem, error)) {
        return false;
    }
    if (!configureSolver(problem)) {
        if (error != nullptr) {
            *error = "Failed to configure OSQP solver data.";
        }
        return false;
    }
    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        if (error != nullptr) {
            *error = "OSQP solve failed.";
        }
        return false;
    }
    const auto status = solver_.getStatus();
    if (status != OsqpEigen::Status::Solved &&
        status != OsqpEigen::Status::SolvedInaccurate) {
        if (error != nullptr) {
            *error = "OSQP returned non-solved status.";
        }
        return false;
    }
    out_solution = solver_.getSolution();
    if (out_solution.size() != problem.numVariables() || !out_solution.allFinite()) {
        if (error != nullptr) {
            *error = "OSQP returned non-finite solution.";
        }
        return false;
    }
    const double max_abs = out_solution.cwiseAbs().maxCoeff();
    if (!std::isfinite(max_abs) || max_abs > 1e6) {
        if (error != nullptr) {
            *error = "OSQP returned implausibly large solution.";
        }
        return false;
    }
    return true;
}

}  // namespace arm_controller::algorithm::reactive_qp
