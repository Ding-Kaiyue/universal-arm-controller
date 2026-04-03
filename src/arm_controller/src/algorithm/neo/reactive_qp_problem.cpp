#include "reactive_qp_problem.hpp"

#include <cmath>
#include <sstream>

namespace arm_controller::algorithm::reactive_qp {

bool ReactiveQpProblem::isWellFormed(std::string* error) const {
	auto fail = [&](const std::string& msg) {
		if (error != nullptr) {
		*error = msg;
		}
		return false;
	};

	const int nv = numVariables();
	const int nc = numConstraints();
	if (nv <= 0) {
		return fail("QP has no variables.");
	}
	if (nc <= 0) {
		return fail("QP has no constraints.");
	}
	if (hessian.rows() != nv || hessian.cols() != nv) {
		std::ostringstream oss;
		oss << "Hessian size mismatch: expected " << nv << "x" << nv << ", got "
			<< hessian.rows() << "x" << hessian.cols() << ".";
		return fail(oss.str());
	}
	if (constraint_matrix.rows() != nc || constraint_matrix.cols() != nv) {
		std::ostringstream oss;
		oss << "Constraint matrix size mismatch: expected " << nc << "x" << nv
			<< ", got " << constraint_matrix.rows() << "x" << constraint_matrix.cols() << ".";
		return fail(oss.str());
	}
	if (upper_bound.size() != nc) {
		return fail("Upper bound size mismatch.");
	}

	if (!hessian.allFinite() || !gradient.allFinite() ||
		!constraint_matrix.allFinite() || !lower_bound.allFinite() ||
		!upper_bound.allFinite()) {
		return fail("QP contains non-finite values.");
	}

	if (!hessian.isApprox(hessian.transpose(), 1e-9)) {
		return fail("Hessian is not symmetric.");
	}

    for (int i = 0; i < nc; ++i) {
        if (lower_bound(i) > upper_bound(i)) {
          std::ostringstream oss;
          oss << "Invalid bounds at row " << i << ": lb > ub.";
          return fail(oss.str());
        }
    }
  return true;
}

}  // namespace arm_controller::algorithm::reactive_qp
