#include "reactive_qp_solver.hpp"

#include <Eigen/SparseCore>

#include <algorithm>
#include <cmath>
#include <sstream>

#include "reactive_qp_validator.hpp"

namespace arm_controller::algorithm::reactive_qp {

ReactiveQpSolver::ReactiveQpSolver(ReactiveQpSolverConfig config)
    : config_(config) {}

ReactiveQpSolver::~ReactiveQpSolver() {
    clearWorkspace();
}

void ReactiveQpSolver::clearWorkspace() {
    if (workspace_ != nullptr) {
        osqp_cleanup(workspace_);
        workspace_ = nullptr;
    }
    hessian_csc_ = OwnedCscMatrix{};
    constraint_csc_ = OwnedCscMatrix{};
#if ARM_CONTROLLER_OSQP_LEGACY_API
    data_ = OSQPData{};
#endif
    initialized_ = false;
    cached_num_variables_ = -1;
    cached_num_constraints_ = -1;
}

bool ReactiveQpSolver::fillCscMatrix(
    const Eigen::SparseMatrix<double>& matrix,
    OwnedCscMatrix& storage) {
    storage = OwnedCscMatrix{};
    Eigen::SparseMatrix<double, Eigen::ColMajor> compressed = matrix;
    compressed.makeCompressed();

    const auto* outer = compressed.outerIndexPtr();
    const auto* inner = compressed.innerIndexPtr();
    const auto* values = compressed.valuePtr();
    const int outer_size = static_cast<int>(compressed.cols()) + 1;
    const int nonzeros = static_cast<int>(compressed.nonZeros());

    storage.outer_index.resize(static_cast<std::size_t>(outer_size));
    storage.inner_index.resize(static_cast<std::size_t>(nonzeros));
    storage.values.resize(static_cast<std::size_t>(nonzeros));
    for (int i = 0; i < outer_size; ++i) {
        storage.outer_index[static_cast<std::size_t>(i)] = static_cast<OsqpInt>(outer[i]);
    }
    for (int i = 0; i < nonzeros; ++i) {
        storage.inner_index[static_cast<std::size_t>(i)] = static_cast<OsqpInt>(inner[i]);
        storage.values[static_cast<std::size_t>(i)] = static_cast<OsqpFloat>(values[i]);
    }

    storage.view.m = static_cast<OsqpInt>(compressed.rows());
    storage.view.n = static_cast<OsqpInt>(compressed.cols());
    storage.view.p = storage.outer_index.data();
    storage.view.i = storage.inner_index.empty() ? nullptr : storage.inner_index.data();
    storage.view.x = storage.values.empty() ? nullptr : storage.values.data();
    storage.view.nzmax = static_cast<OsqpInt>(nonzeros);
    storage.view.nz = -1;
#if !ARM_CONTROLLER_OSQP_LEGACY_API
    storage.view.owned = 0;
#endif
    return true;
}

#if ARM_CONTROLLER_OSQP_LEGACY_API
bool ReactiveQpSolver::diagnoseOsqpData(const OSQPData& data) {
    if (data.P == nullptr) {
        return false;
    }
    if (data.A == nullptr) {
        return false;
    }
    if (data.q == nullptr) {
        return false;
    }
    if (data.l == nullptr || data.u == nullptr) {
        return false;
    }
    if (data.n <= 0 || data.m < 0) {
        return false;
    }
    if (data.P->m != data.n || data.P->n != data.n) {
        return false;
    }
    if (data.P->p == nullptr || (data.P->nzmax > 0 && data.P->i == nullptr)) {
        return false;
    }
    if (data.P->p[0] != 0 || data.P->p[data.n] != data.P->nzmax) {
        return false;
    }
    for (c_int j = 0; j < data.n; ++j) {
        if (data.P->p[j] > data.P->p[j + 1] || data.P->p[j] < 0 ||
            data.P->p[j + 1] > data.P->nzmax) {
            return false;
        }
        for (c_int ptr = data.P->p[j]; ptr < data.P->p[j + 1]; ++ptr) {
            if (data.P->i[ptr] < 0 || data.P->i[ptr] >= data.n) {
                return false;
            }
            if (data.P->i[ptr] > j) {
                return false;
            }
        }
    }
    if (data.A->m != data.m || data.A->n != data.n) {
        return false;
    }
    if (data.A->p == nullptr || (data.A->nzmax > 0 && data.A->i == nullptr)) {
        return false;
    }
    if (data.A->p[0] != 0 || data.A->p[data.n] != data.A->nzmax) {
        return false;
    }
    for (c_int j = 0; j < data.n; ++j) {
        if (data.A->p[j] > data.A->p[j + 1] || data.A->p[j] < 0 ||
            data.A->p[j + 1] > data.A->nzmax) {
            return false;
        }
        for (c_int ptr = data.A->p[j]; ptr < data.A->p[j + 1]; ++ptr) {
            if (data.A->i[ptr] < 0 || data.A->i[ptr] >= data.m) {
                return false;
            }
        }
    }
    for (c_int i = 0; i < data.m; ++i) {
        if (data.l[i] > data.u[i]) {
            return false;
        }
    }
    return true;
}
#else
bool ReactiveQpSolver::diagnoseOsqpData(
    const OwnedCscMatrix& hessian,
    const OwnedCscMatrix& constraints,
    const std::vector<OsqpFloat>& gradient,
    const std::vector<OsqpFloat>& lower_bound,
    const std::vector<OsqpFloat>& upper_bound,
    int num_variables,
    int num_constraints) {
    const OsqpCscMatrix& P = hessian.view;
    const OsqpCscMatrix& A = constraints.view;
    if (num_variables <= 0 || num_constraints < 0) {
        return false;
    }
    if (P.m != num_variables || P.n != num_variables) {
        return false;
    }
    if (A.m != num_constraints || A.n != num_variables) {
        return false;
    }
    if (gradient.size() != static_cast<std::size_t>(num_variables) ||
        lower_bound.size() != static_cast<std::size_t>(num_constraints) ||
        upper_bound.size() != static_cast<std::size_t>(num_constraints)) {
        return false;
    }
    const auto check_matrix = [](const char* name,
                                 const OsqpCscMatrix& matrix,
                                 OsqpInt rows,
                                 bool upper_triangular) {
        if (matrix.p == nullptr || (matrix.nzmax > 0 && (matrix.i == nullptr || matrix.x == nullptr))) {
            (void)name;
            return false;
        }
        if (matrix.p[0] != 0 || matrix.p[matrix.n] != matrix.nzmax) {
            return false;
        }
        for (OsqpInt col = 0; col < matrix.n; ++col) {
            if (matrix.p[col] > matrix.p[col + 1] || matrix.p[col] < 0 ||
                matrix.p[col + 1] > matrix.nzmax) {
                return false;
            }
            for (OsqpInt ptr = matrix.p[col]; ptr < matrix.p[col + 1]; ++ptr) {
                if (matrix.i[ptr] < 0 || matrix.i[ptr] >= rows) {
                    return false;
                }
                if (upper_triangular && matrix.i[ptr] > col) {
                    return false;
                }
            }
        }
        return true;
    };
    if (!check_matrix("P", P, static_cast<OsqpInt>(num_variables), true) ||
        !check_matrix("A", A, static_cast<OsqpInt>(num_constraints), false)) {
        return false;
    }
    for (int i = 0; i < num_constraints; ++i) {
        if (lower_bound[static_cast<std::size_t>(i)] > upper_bound[static_cast<std::size_t>(i)]) {
            return false;
        }
    }
    return true;
}
#endif

bool ReactiveQpSolver::configureSolver(const ReactiveQpProblem& problem) {
    clearWorkspace();

    const int nv = problem.numVariables();
    const int nc = problem.numConstraints();
    Eigen::MatrixXd h_upper =
        Eigen::MatrixXd::Zero(problem.hessian.rows(), problem.hessian.cols());
    h_upper.triangularView<Eigen::Upper>() =
        problem.hessian.triangularView<Eigen::Upper>();
    Eigen::SparseMatrix<double> h_sparse = h_upper.sparseView();
    h_sparse.makeCompressed();
    Eigen::SparseMatrix<double> a_sparse = problem.constraint_matrix.sparseView();
    a_sparse.makeCompressed();

    if (!fillCscMatrix(h_sparse, hessian_csc_)) {
        return false;
    }
    if (!fillCscMatrix(a_sparse, constraint_csc_)) {
        return false;
    }

    gradient_storage_.resize(static_cast<std::size_t>(problem.gradient.size()));
    lower_bound_storage_.resize(static_cast<std::size_t>(problem.lower_bound.size()));
    upper_bound_storage_.resize(static_cast<std::size_t>(problem.upper_bound.size()));
    for (int i = 0; i < problem.gradient.size(); ++i) {
        gradient_storage_[static_cast<std::size_t>(i)] =
            static_cast<OsqpFloat>(problem.gradient(i));
    }
    for (int i = 0; i < problem.lower_bound.size(); ++i) {
        lower_bound_storage_[static_cast<std::size_t>(i)] =
            static_cast<OsqpFloat>(problem.lower_bound(i));
        upper_bound_storage_[static_cast<std::size_t>(i)] =
            static_cast<OsqpFloat>(problem.upper_bound(i));
    }

#if ARM_CONTROLLER_OSQP_LEGACY_API
    data_.n = static_cast<c_int>(nv);
    data_.m = static_cast<c_int>(nc);
    data_.P = &hessian_csc_.view;
    data_.A = &constraint_csc_.view;
    data_.q = gradient_storage_.data();
    data_.l = lower_bound_storage_.data();
    data_.u = upper_bound_storage_.data();
    if (!diagnoseOsqpData(data_)) {
        return false;
    }
#else
    if (!diagnoseOsqpData(hessian_csc_,
                          constraint_csc_,
                          gradient_storage_,
                          lower_bound_storage_,
                          upper_bound_storage_,
                          nv,
                          nc)) {
        return false;
    }
#endif

    osqp_set_default_settings(&settings_);
#if ARM_CONTROLLER_OSQP_LEGACY_API
    settings_.warm_start = config_.warm_start ? 1 : 0;
    settings_.verbose = config_.verbose ? 1 : 0;
    settings_.max_iter = static_cast<c_int>(config_.max_iterations);
    settings_.eps_abs = static_cast<c_float>(config_.absolute_tolerance);
    settings_.eps_rel = static_cast<c_float>(config_.relative_tolerance);
    settings_.polish = 0;
#else
    settings_.warm_starting = config_.warm_start ? 1 : 0;
    settings_.verbose = config_.verbose ? 1 : 0;
    settings_.max_iter = static_cast<OSQPInt>(config_.max_iterations);
    settings_.eps_abs = static_cast<OSQPFloat>(config_.absolute_tolerance);
    settings_.eps_rel = static_cast<OSQPFloat>(config_.relative_tolerance);
    settings_.polishing = 0;
#endif
#if ARM_CONTROLLER_OSQP_LEGACY_API
    const c_int setup_status = osqp_setup(&workspace_, &data_, &settings_);
#else
    const OSQPInt setup_status = osqp_setup(&workspace_,
                                            &hessian_csc_.view,
                                            gradient_storage_.data(),
                                            &constraint_csc_.view,
                                            lower_bound_storage_.data(),
                                            upper_bound_storage_.data(),
                                            static_cast<OSQPInt>(nc),
                                            static_cast<OSQPInt>(nv),
                                            &settings_);
#endif
    if (setup_status != 0 || workspace_ == nullptr) {
        if (workspace_ != nullptr) {
            osqp_cleanup(workspace_);
            workspace_ = nullptr;
        }
#if ARM_CONTROLLER_OSQP_LEGACY_API
        data_ = OSQPData{};
#endif
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
            *error = "Failed to configure OSQP C solver data.";
        }
        return false;
    }
    const OsqpInt solve_status = osqp_solve(workspace_);
    if (solve_status != 0) {
        if (error != nullptr) {
            *error = "OSQP solve failed.";
        }
        return false;
    }

#if ARM_CONTROLLER_OSQP_LEGACY_API
    if (workspace_ == nullptr || workspace_->info == nullptr ||
        (workspace_->info->status_val != OSQP_SOLVED &&
         workspace_->info->status_val != OSQP_SOLVED_INACCURATE)) {
        if (error != nullptr) {
            if (workspace_ != nullptr && workspace_->info != nullptr) {
                std::ostringstream oss;
                oss << "OSQP returned non-solved status: status=\""
                    << workspace_->info->status << "\""
                    << " status_val=" << workspace_->info->status_val
                    << " iter=" << workspace_->info->iter;
                *error = oss.str();
            } else {
                *error = "OSQP returned non-solved status.";
            }
        }
        return false;
    }
    if (workspace_->solution == nullptr || workspace_->solution->x == nullptr) {
        if (error != nullptr) {
            *error = "OSQP returned empty solution.";
        }
        return false;
    }

    out_solution.resize(problem.numVariables());
    for (int i = 0; i < out_solution.size(); ++i) {
        out_solution(i) = static_cast<double>(workspace_->solution->x[i]);
    }
#else
    if (workspace_ == nullptr || workspace_->info == nullptr ||
        (workspace_->info->status_val != OSQP_SOLVED &&
         workspace_->info->status_val != OSQP_SOLVED_INACCURATE)) {
        if (error != nullptr) {
            if (workspace_ != nullptr && workspace_->info != nullptr) {
                std::ostringstream oss;
                oss << "OSQP returned non-solved status: status=\""
                    << workspace_->info->status << "\""
                    << " status_val=" << workspace_->info->status_val
                    << " iter=" << workspace_->info->iter;
                *error = oss.str();
            } else {
                *error = "OSQP returned non-solved status.";
            }
        }
        return false;
    }
    if (workspace_->solution == nullptr || workspace_->solution->x == nullptr) {
        if (error != nullptr) {
            *error = "OSQP returned empty solution.";
        }
        return false;
    }

    out_solution.resize(problem.numVariables());
    for (int i = 0; i < out_solution.size(); ++i) {
        out_solution(i) = static_cast<double>(workspace_->solution->x[i]);
    }
#endif
    if (!out_solution.allFinite()) {
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
