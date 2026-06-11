#include "controller/reactive_task/local_planner/whole_body_lbfgs_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <utility>

#include <Eigen/LU>

namespace arm_controller::controller::reactive_task {

namespace {

constexpr int kBaseDof = 3;
using PathPlanningInput =
    arm_controller::algorithm::cartesian_path_planner::PathPlanningInput;

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

Eigen::VectorXd clampState(
    Eigen::VectorXd q,
    const Eigen::VectorXd& q_min,
    const Eigen::VectorXd& q_max) {
  if (q.size() != q_min.size() || q.size() != q_max.size()) {
    return q;
  }
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    q[i] = std::clamp(q[i], q_min[i], q_max[i]);
  }
  if (q.size() >= kBaseDof) {
    q[2] = normalizeAngle(q[2]);
  }
  return q;
}

double virtualTimeToRealTime(const double vt) {
  if (vt > 0.0) {
    return (0.5 * vt + 1.0) * vt + 1.0;
  }
  return 1.0 / ((0.5 * vt - 1.0) * vt + 1.0);
}

double realTimeToVirtualTime(const double rt) {
  const double t = std::max(1.0e-6, rt);
  if (t > 1.0) {
    return std::sqrt(2.0 * t - 1.0) - 1.0;
  }
  return 1.0 - std::sqrt(2.0 / t - 1.0);
}

double virtualTimeToRealTimeDerivative(const double vt) {
  if (vt > 0.0) {
    return vt + 1.0;
  }
  const double denom = (0.5 * vt - 1.0) * vt + 1.0;
  return (1.0 - vt) / std::max(1.0e-12, denom * denom);
}

std::vector<double> uniformDurations(
    const std::size_t piece_count,
    const double dt) {
  return std::vector<double>(piece_count, std::max(1.0e-3, dt));
}

bool unpackVariables(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& q_start,
    const Eigen::VectorXd& q_goal,
    const Eigen::VectorXd& q_min,
    const Eigen::VectorXd& q_max,
    WholeBodyLbfgsOptimizer::StateList* waypoints,
    std::vector<double>* durations) {
  if (waypoints == nullptr || durations == nullptr ||
      q_start.size() <= 0 || q_start.size() != q_goal.size()) {
    return false;
  }
  const Eigen::Index dof = q_start.size();
  if (x.size() < dof + 1) {
    return false;
  }
  const Eigen::Index inner_count =
      (x.size() - 1) / (dof + 1);
  const Eigen::Index piece_count = inner_count + 1;
  if (x.size() != inner_count * dof + piece_count) {
    return false;
  }

  waypoints->clear();
  waypoints->reserve(static_cast<std::size_t>(piece_count + 1));
  waypoints->push_back(clampState(q_start, q_min, q_max));
  Eigen::Index offset = 0;
  for (Eigen::Index i = 0; i < inner_count; ++i) {
    Eigen::VectorXd q = x.segment(offset, dof);
    q = clampState(q, q_min, q_max);
    waypoints->push_back(q);
    offset += dof;
  }
  waypoints->push_back(clampState(q_goal, q_min, q_max));

  durations->clear();
  durations->reserve(static_cast<std::size_t>(piece_count));
  for (Eigen::Index i = 0; i < piece_count; ++i) {
    const double duration = virtualTimeToRealTime(x[offset + i]);
    if (!(duration > 0.0) || !std::isfinite(duration)) {
      return false;
    }
    durations->push_back(duration);
  }
  return true;
}

bool packVariables(
    const WholeBodyLbfgsOptimizer::StateList& waypoints,
    const std::vector<double>& durations,
    Eigen::VectorXd* x) {
  if (x == nullptr || waypoints.size() < 2u ||
      durations.size() + 1u != waypoints.size()) {
    return false;
  }
  const Eigen::Index dof = waypoints.front().size();
  if (dof <= 0) {
    return false;
  }
  const auto inner_count = static_cast<Eigen::Index>(waypoints.size() - 2u);
  const auto piece_count = static_cast<Eigen::Index>(durations.size());
  x->resize(inner_count * dof + piece_count);
  Eigen::Index offset = 0;
  for (Eigen::Index i = 0; i < inner_count; ++i) {
    x->segment(offset, dof) = waypoints[static_cast<std::size_t>(i + 1)];
    offset += dof;
  }
  for (Eigen::Index i = 0; i < piece_count; ++i) {
    (*x)[offset + i] =
        realTimeToVirtualTime(durations[static_cast<std::size_t>(i)]);
  }
  return true;
}

bool validateState(
    const WholeBodyLbfgsOptimizer::JointStateValidatorFn& validator,
    const Eigen::VectorXd& q,
    const double safe_distance,
    PathPlanningInput::WholeBodyPoseDiagnostic* diag = nullptr) {
  if (!validator) {
    return true;
  }
  PathPlanningInput::WholeBodyPoseDiagnostic local_diag;
  const bool ok = validator(q, safe_distance, &local_diag);
  if (diag != nullptr) {
    *diag = local_diag;
  }
  return ok;
}

bool validateSegment(
    const WholeBodyLbfgsOptimizer::JointSegmentValidatorFn& validator,
    const Eigen::VectorXd& a,
    const Eigen::VectorXd& b,
    const double safe_distance,
    PathPlanningInput::WholeBodyPoseDiagnostic* diag = nullptr) {
  if (!validator) {
    return true;
  }
  PathPlanningInput::WholeBodyPoseDiagnostic local_diag;
  const bool ok = validator(a, b, safe_distance, &local_diag);
  if (diag != nullptr) {
    *diag = local_diag;
  }
  return ok;
}

bool pathValid(
    const WholeBodyLbfgsOptimizer::Input& input,
    const WholeBodyLbfgsOptimizer::StateList& states) {
  if (states.size() < 2u) {
    return false;
  }
  for (std::size_t i = 0; i < states.size(); ++i) {
    if (!validateState(input.state_validator, states[i], input.safe_distance)) {
      return false;
    }
    if (i > 0u &&
        !validateSegment(
            input.segment_validator,
            states[i - 1u],
            states[i],
            input.safe_distance)) {
      return false;
    }
  }
  return true;
}

bool pathValid(
    const WholeBodyLbfgsOptimizer::Input& input,
    const WholeBodyPolynomialTrajectory& trajectory) {
  const WholeBodyLbfgsOptimizer::StateList states =
      trajectory.sampleUniform(std::max<std::size_t>(2u, trajectory.size() * 3u));
  return pathValid(input, states);
}

struct CostGradient {
  double cost{std::numeric_limits<double>::infinity()};
  Eigen::VectorXd gradient;
};

double velocityLimitForIndex(
    Eigen::Index index,
    const WholeBodyLbfgsOptimizer::Config& cfg);

void addDurationGradient(
    std::size_t piece_index,
    double g_t,
    Eigen::Index inner_count,
    Eigen::Index dof,
    const Eigen::VectorXd& x,
    Eigen::VectorXd* gradient);

Eigen::Matrix<double, 8, 1> polynomialBasis(
    const double t,
    const int derivative) {
  Eigen::Matrix<double, 8, 1> basis;
  basis.setZero();
  for (int power = derivative; power < 8; ++power) {
    double factor = 1.0;
    for (int k = 0; k < derivative; ++k) {
      factor *= static_cast<double>(power - k);
    }
    basis[power] = factor * std::pow(t, power - derivative);
  }
  return basis;
}

struct MincoWorkspace {
  Eigen::MatrixXd A;
  Eigen::MatrixXd coeffs;
  Eigen::VectorXd T1;
  Eigen::VectorXd T2;
  Eigen::VectorXd T3;
  Eigen::VectorXd T4;
  Eigen::VectorXd T5;
  Eigen::VectorXd T6;
  Eigen::VectorXd T7;
};

bool buildMincoWorkspace(
    const WholeBodyLbfgsOptimizer::StateList& waypoints,
    const std::vector<double>& durations,
    MincoWorkspace* workspace) {
  if (workspace == nullptr || waypoints.size() < 2u ||
      durations.size() + 1u != waypoints.size()) {
    return false;
  }
  const Eigen::Index dof = waypoints.front().size();
  const int pieces = static_cast<int>(durations.size());
  workspace->T1.resize(pieces);
  workspace->T2.resize(pieces);
  workspace->T3.resize(pieces);
  workspace->T4.resize(pieces);
  workspace->T5.resize(pieces);
  workspace->T6.resize(pieces);
  workspace->T7.resize(pieces);
  for (int i = 0; i < pieces; ++i) {
    workspace->T1[i] = std::max(1.0e-9, durations[static_cast<std::size_t>(i)]);
    workspace->T2[i] = workspace->T1[i] * workspace->T1[i];
    workspace->T3[i] = workspace->T2[i] * workspace->T1[i];
    workspace->T4[i] = workspace->T2[i] * workspace->T2[i];
    workspace->T5[i] = workspace->T4[i] * workspace->T1[i];
    workspace->T6[i] = workspace->T4[i] * workspace->T2[i];
    workspace->T7[i] = workspace->T4[i] * workspace->T3[i];
  }

  Eigen::MatrixXd& A = workspace->A;
  A = Eigen::MatrixXd::Zero(8 * pieces, 8 * pieces);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(8 * pieces, dof);
  A(0, 0) = 1.0;
  A(1, 1) = 1.0;
  A(2, 2) = 2.0;
  A(3, 3) = 6.0;
  B.row(0) = waypoints.front().transpose();

  for (int i = 0; i < pieces - 1; ++i) {
    const double T1 = workspace->T1[i];
    const double T2 = workspace->T2[i];
    const double T3 = workspace->T3[i];
    const double T4 = workspace->T4[i];
    const double T5 = workspace->T5[i];
    const double T6 = workspace->T6[i];
    const double T7 = workspace->T7[i];
    A(8 * i + 4, 8 * i + 4) = 24.0;
    A(8 * i + 4, 8 * i + 5) = 120.0 * T1;
    A(8 * i + 4, 8 * i + 6) = 360.0 * T2;
    A(8 * i + 4, 8 * i + 7) = 840.0 * T3;
    A(8 * i + 4, 8 * i + 12) = -24.0;
    A(8 * i + 5, 8 * i + 5) = 120.0;
    A(8 * i + 5, 8 * i + 6) = 720.0 * T1;
    A(8 * i + 5, 8 * i + 7) = 2520.0 * T2;
    A(8 * i + 5, 8 * i + 13) = -120.0;
    A(8 * i + 6, 8 * i + 6) = 720.0;
    A(8 * i + 6, 8 * i + 7) = 5040.0 * T1;
    A(8 * i + 6, 8 * i + 14) = -720.0;
    A(8 * i + 7, 8 * i) = 1.0;
    A(8 * i + 7, 8 * i + 1) = T1;
    A(8 * i + 7, 8 * i + 2) = T2;
    A(8 * i + 7, 8 * i + 3) = T3;
    A(8 * i + 7, 8 * i + 4) = T4;
    A(8 * i + 7, 8 * i + 5) = T5;
    A(8 * i + 7, 8 * i + 6) = T6;
    A(8 * i + 7, 8 * i + 7) = T7;
    A(8 * i + 8, 8 * i) = 1.0;
    A(8 * i + 8, 8 * i + 1) = T1;
    A(8 * i + 8, 8 * i + 2) = T2;
    A(8 * i + 8, 8 * i + 3) = T3;
    A(8 * i + 8, 8 * i + 4) = T4;
    A(8 * i + 8, 8 * i + 5) = T5;
    A(8 * i + 8, 8 * i + 6) = T6;
    A(8 * i + 8, 8 * i + 7) = T7;
    A(8 * i + 8, 8 * i + 8) = -1.0;
    A(8 * i + 9, 8 * i + 1) = 1.0;
    A(8 * i + 9, 8 * i + 2) = 2.0 * T1;
    A(8 * i + 9, 8 * i + 3) = 3.0 * T2;
    A(8 * i + 9, 8 * i + 4) = 4.0 * T3;
    A(8 * i + 9, 8 * i + 5) = 5.0 * T4;
    A(8 * i + 9, 8 * i + 6) = 6.0 * T5;
    A(8 * i + 9, 8 * i + 7) = 7.0 * T6;
    A(8 * i + 9, 8 * i + 9) = -1.0;
    A(8 * i + 10, 8 * i + 2) = 2.0;
    A(8 * i + 10, 8 * i + 3) = 6.0 * T1;
    A(8 * i + 10, 8 * i + 4) = 12.0 * T2;
    A(8 * i + 10, 8 * i + 5) = 20.0 * T3;
    A(8 * i + 10, 8 * i + 6) = 30.0 * T4;
    A(8 * i + 10, 8 * i + 7) = 42.0 * T5;
    A(8 * i + 10, 8 * i + 10) = -2.0;
    A(8 * i + 11, 8 * i + 3) = 6.0;
    A(8 * i + 11, 8 * i + 4) = 24.0 * T1;
    A(8 * i + 11, 8 * i + 5) = 60.0 * T2;
    A(8 * i + 11, 8 * i + 6) = 120.0 * T3;
    A(8 * i + 11, 8 * i + 7) = 210.0 * T4;
    A(8 * i + 11, 8 * i + 11) = -6.0;
    Eigen::VectorXd inner = waypoints[static_cast<std::size_t>(i + 1)];
    inner[2] = waypoints[i][2] +
               normalizeAngle(waypoints[i + 1u][2] - waypoints[i][2]);
    B.row(8 * i + 7) = inner.transpose();
  }

  const int i = pieces - 1;
  A(8 * pieces - 4, 8 * pieces - 8) = 1.0;
  A(8 * pieces - 4, 8 * pieces - 7) = workspace->T1[i];
  A(8 * pieces - 4, 8 * pieces - 6) = workspace->T2[i];
  A(8 * pieces - 4, 8 * pieces - 5) = workspace->T3[i];
  A(8 * pieces - 4, 8 * pieces - 4) = workspace->T4[i];
  A(8 * pieces - 4, 8 * pieces - 3) = workspace->T5[i];
  A(8 * pieces - 4, 8 * pieces - 2) = workspace->T6[i];
  A(8 * pieces - 4, 8 * pieces - 1) = workspace->T7[i];
  A(8 * pieces - 3, 8 * pieces - 7) = 1.0;
  A(8 * pieces - 3, 8 * pieces - 6) = 2.0 * workspace->T1[i];
  A(8 * pieces - 3, 8 * pieces - 5) = 3.0 * workspace->T2[i];
  A(8 * pieces - 3, 8 * pieces - 4) = 4.0 * workspace->T3[i];
  A(8 * pieces - 3, 8 * pieces - 3) = 5.0 * workspace->T4[i];
  A(8 * pieces - 3, 8 * pieces - 2) = 6.0 * workspace->T5[i];
  A(8 * pieces - 3, 8 * pieces - 1) = 7.0 * workspace->T6[i];
  A(8 * pieces - 2, 8 * pieces - 6) = 2.0;
  A(8 * pieces - 2, 8 * pieces - 5) = 6.0 * workspace->T1[i];
  A(8 * pieces - 2, 8 * pieces - 4) = 12.0 * workspace->T2[i];
  A(8 * pieces - 2, 8 * pieces - 3) = 20.0 * workspace->T3[i];
  A(8 * pieces - 2, 8 * pieces - 2) = 30.0 * workspace->T4[i];
  A(8 * pieces - 2, 8 * pieces - 1) = 42.0 * workspace->T5[i];
  A(8 * pieces - 1, 8 * pieces - 5) = 6.0;
  A(8 * pieces - 1, 8 * pieces - 4) = 24.0 * workspace->T1[i];
  A(8 * pieces - 1, 8 * pieces - 3) = 60.0 * workspace->T2[i];
  A(8 * pieces - 1, 8 * pieces - 2) = 120.0 * workspace->T3[i];
  A(8 * pieces - 1, 8 * pieces - 1) = 210.0 * workspace->T4[i];
  B.row(8 * pieces - 4) = waypoints.back().transpose();

  workspace->coeffs = A.fullPivLu().solve(B);
  return workspace->coeffs.rows() == 8 * pieces &&
         workspace->coeffs.cols() == dof &&
         workspace->coeffs.allFinite();
}

double addMincoSnapGradCost(
    const WholeBodyLbfgsOptimizer::Config& cfg,
    const MincoWorkspace& minco,
    Eigen::MatrixXd* gdC,
    Eigen::VectorXd* gdT) {
  if (gdC == nullptr || gdT == nullptr) {
    return 0.0;
  }
  const int pieces = static_cast<int>(minco.T1.size());
  double cost = 0.0;
  for (int i = 0; i < pieces; ++i) {
    const Eigen::RowVectorXd c4 = minco.coeffs.row(8 * i + 4);
    const Eigen::RowVectorXd c5 = minco.coeffs.row(8 * i + 5);
    const Eigen::RowVectorXd c6 = minco.coeffs.row(8 * i + 6);
    const Eigen::RowVectorXd c7 = minco.coeffs.row(8 * i + 7);
    const double T1 = minco.T1[i];
    const double T2 = minco.T2[i];
    const double T3 = minco.T3[i];
    const double T4 = minco.T4[i];
    const double T5 = minco.T5[i];
    const double T6 = minco.T6[i];
    const double T7 = minco.T7[i];
    cost += cfg.smoothness_weight *
            (576.0 * c4.squaredNorm() * T1 +
             2880.0 * c4.dot(c5) * T2 +
             4800.0 * c5.squaredNorm() * T3 +
             5760.0 * c4.dot(c6) * T3 +
             21600.0 * c5.dot(c6) * T4 +
             10080.0 * c4.dot(c7) * T4 +
             25920.0 * c6.squaredNorm() * T5 +
             40320.0 * c5.dot(c7) * T5 +
             100800.0 * c6.dot(c7) * T6 +
             100800.0 * c7.squaredNorm() * T7);

    (*gdT)[i] += cfg.smoothness_weight *
                 (576.0 * c4.squaredNorm() +
                  5760.0 * c4.dot(c5) * T1 +
                  14400.0 * c5.squaredNorm() * T2 +
                  17280.0 * c4.dot(c6) * T2 +
                  86400.0 * c5.dot(c6) * T3 +
                  40320.0 * c4.dot(c7) * T3 +
                  129600.0 * c6.squaredNorm() * T4 +
                  201600.0 * c5.dot(c7) * T4 +
                  604800.0 * c6.dot(c7) * T5 +
                  705600.0 * c7.squaredNorm() * T6);
    gdC->row(8 * i + 7) += cfg.smoothness_weight *
        (10080.0 * c4 * T4 + 40320.0 * c5 * T5 +
         100800.0 * c6 * T6 + 201600.0 * c7 * T7);
    gdC->row(8 * i + 6) += cfg.smoothness_weight *
        (5760.0 * c4 * T3 + 21600.0 * c5 * T4 +
         51840.0 * c6 * T5 + 100800.0 * c7 * T6);
    gdC->row(8 * i + 5) += cfg.smoothness_weight *
        (2880.0 * c4 * T2 + 9600.0 * c5 * T3 +
         21600.0 * c6 * T4 + 40320.0 * c7 * T5);
    gdC->row(8 * i + 4) += cfg.smoothness_weight *
        (1152.0 * c4 * T1 + 2880.0 * c5 * T2 +
         5760.0 * c6 * T3 + 10080.0 * c7 * T4);
  }
  return cost;
}

void addMincoTimePropagation(
    const MincoWorkspace& minco,
    const Eigen::MatrixXd& adjGdC,
    Eigen::VectorXd* gdT) {
  if (gdT == nullptr) {
    return;
  }
  const int pieces = static_cast<int>(minco.T1.size());
  for (int i = 0; i < pieces - 1; ++i) {
    Eigen::MatrixXd B1 = Eigen::MatrixXd::Zero(8, minco.coeffs.cols());
    B1.row(3) = -(minco.coeffs.row(8 * i + 1) +
                  2.0 * minco.T1[i] * minco.coeffs.row(8 * i + 2) +
                  3.0 * minco.T2[i] * minco.coeffs.row(8 * i + 3) +
                  4.0 * minco.T3[i] * minco.coeffs.row(8 * i + 4) +
                  5.0 * minco.T4[i] * minco.coeffs.row(8 * i + 5) +
                  6.0 * minco.T5[i] * minco.coeffs.row(8 * i + 6) +
                  7.0 * minco.T6[i] * minco.coeffs.row(8 * i + 7));
    B1.row(4) = B1.row(3);
    B1.row(5) = -(2.0 * minco.coeffs.row(8 * i + 2) +
                  6.0 * minco.T1[i] * minco.coeffs.row(8 * i + 3) +
                  12.0 * minco.T2[i] * minco.coeffs.row(8 * i + 4) +
                  20.0 * minco.T3[i] * minco.coeffs.row(8 * i + 5) +
                  30.0 * minco.T4[i] * minco.coeffs.row(8 * i + 6) +
                  42.0 * minco.T5[i] * minco.coeffs.row(8 * i + 7));
    B1.row(6) = -(6.0 * minco.coeffs.row(8 * i + 3) +
                  24.0 * minco.T1[i] * minco.coeffs.row(8 * i + 4) +
                  60.0 * minco.T2[i] * minco.coeffs.row(8 * i + 5) +
                  120.0 * minco.T3[i] * minco.coeffs.row(8 * i + 6) +
                  210.0 * minco.T4[i] * minco.coeffs.row(8 * i + 7));
    B1.row(7) = -(24.0 * minco.coeffs.row(8 * i + 4) +
                  120.0 * minco.T1[i] * minco.coeffs.row(8 * i + 5) +
                  360.0 * minco.T2[i] * minco.coeffs.row(8 * i + 6) +
                  840.0 * minco.T3[i] * minco.coeffs.row(8 * i + 7));
    B1.row(0) = -(120.0 * minco.coeffs.row(8 * i + 5) +
                  720.0 * minco.T1[i] * minco.coeffs.row(8 * i + 6) +
                  2520.0 * minco.T2[i] * minco.coeffs.row(8 * i + 7));
    B1.row(1) = -(720.0 * minco.coeffs.row(8 * i + 6) +
                  5040.0 * minco.T1[i] * minco.coeffs.row(8 * i + 7));
    B1.row(2) = -5040.0 * minco.coeffs.row(8 * i + 7);
    (*gdT)[i] +=
        B1.cwiseProduct(adjGdC.block(8 * i + 4, 0, 8, minco.coeffs.cols()))
            .sum();
  }

  const int i = pieces - 1;
  Eigen::MatrixXd B2 = Eigen::MatrixXd::Zero(4, minco.coeffs.cols());
  B2.row(0) = -(minco.coeffs.row(8 * pieces - 7) +
                2.0 * minco.T1[i] * minco.coeffs.row(8 * pieces - 6) +
                3.0 * minco.T2[i] * minco.coeffs.row(8 * pieces - 5) +
                4.0 * minco.T3[i] * minco.coeffs.row(8 * pieces - 4) +
                5.0 * minco.T4[i] * minco.coeffs.row(8 * pieces - 3) +
                6.0 * minco.T5[i] * minco.coeffs.row(8 * pieces - 2) +
                7.0 * minco.T6[i] * minco.coeffs.row(8 * pieces - 1));
  B2.row(1) = -(2.0 * minco.coeffs.row(8 * pieces - 6) +
                6.0 * minco.T1[i] * minco.coeffs.row(8 * pieces - 5) +
                12.0 * minco.T2[i] * minco.coeffs.row(8 * pieces - 4) +
                20.0 * minco.T3[i] * minco.coeffs.row(8 * pieces - 3) +
                30.0 * minco.T4[i] * minco.coeffs.row(8 * pieces - 2) +
                42.0 * minco.T5[i] * minco.coeffs.row(8 * pieces - 1));
  B2.row(2) = -(6.0 * minco.coeffs.row(8 * pieces - 5) +
                24.0 * minco.T1[i] * minco.coeffs.row(8 * pieces - 4) +
                60.0 * minco.T2[i] * minco.coeffs.row(8 * pieces - 3) +
                120.0 * minco.T3[i] * minco.coeffs.row(8 * pieces - 2) +
                210.0 * minco.T4[i] * minco.coeffs.row(8 * pieces - 1));
  B2.row(3) = -(24.0 * minco.coeffs.row(8 * pieces - 4) +
                120.0 * minco.T1[i] * minco.coeffs.row(8 * pieces - 3) +
                360.0 * minco.T2[i] * minco.coeffs.row(8 * pieces - 2) +
                840.0 * minco.T3[i] * minco.coeffs.row(8 * pieces - 1));
  (*gdT)[i] +=
      B2.cwiseProduct(adjGdC.block(8 * pieces - 4, 0, 4, minco.coeffs.cols()))
          .sum();
}

Eigen::VectorXd evaluateCoeffBlock(
    const Eigen::MatrixXd& coeff,
    const double t,
    const int derivative) {
  return coeff.transpose() * polynomialBasis(t, derivative);
}

double addSampledMincoGradCost(
    const WholeBodyLbfgsOptimizer::Input& input,
    const WholeBodyLbfgsOptimizer::Config& cfg,
    const MincoWorkspace& minco,
    Eigen::MatrixXd* gdC,
    Eigen::VectorXd* gdT) {
  if (gdC == nullptr || gdT == nullptr) {
    return 0.0;
  }
  const int samples_per_piece = std::max(1, cfg.constrain_points_per_piece);
  const int pieces = static_cast<int>(minco.T1.size());
  const Eigen::Index dof = minco.coeffs.cols();
  double cost = 0.0;

  for (int i = 0; i < pieces; ++i) {
    const Eigen::MatrixXd coeff = minco.coeffs.block(8 * i, 0, 8, dof);
    const double step = minco.T1[i] / static_cast<double>(samples_per_piece);
    for (int j = 0; j <= samples_per_piece; ++j) {
      const double t = step * static_cast<double>(j);
      const double alpha =
          static_cast<double>(j) / static_cast<double>(samples_per_piece);
      const double weight = (j == 0 || j == samples_per_piece) ? 0.5 : 1.0;
      Eigen::VectorXd pos = evaluateCoeffBlock(coeff, t, 0);
      Eigen::VectorXd vel = evaluateCoeffBlock(coeff, t, 1);
      Eigen::VectorXd acc = evaluateCoeffBlock(coeff, t, 2);
      Eigen::VectorXd jerk = evaluateCoeffBlock(coeff, t, 3);
      const Eigen::VectorXd snap = evaluateCoeffBlock(coeff, t, 4);
      if (pos.size() >= kBaseDof) {
        pos[2] = normalizeAngle(pos[2]);
      }

      if (input.collision_cost_gradient_fn) {
        const double safe_distance =
            std::max(input.safe_distance, cfg.obstacle_safe_margin);
        const WholeBodyLbfgsOptimizer::CollisionCostGradient cg =
            input.collision_cost_gradient_fn(pos, safe_distance);
        if (cg.valid && std::isfinite(cg.cost) &&
            cg.gradient.size() == dof && cg.gradient.allFinite()) {
          const double sample_cost =
              cfg.collision_weight * std::max(0.0, cg.cost);
          const Eigen::VectorXd grad_pos = cfg.collision_weight * cg.gradient;
          cost += weight * step * sample_cost;
          gdC->block(8 * i, 0, 8, dof) +=
              weight * step * polynomialBasis(t, 0) * grad_pos.transpose();
          (*gdT)[i] +=
              weight *
              (sample_cost / static_cast<double>(samples_per_piece) +
               step * alpha * grad_pos.dot(vel));
        }
      }

      for (Eigen::Index dim = 0; dim < dof; ++dim) {
        const double limit = velocityLimitForIndex(dim, cfg);
        const double excess = std::abs(vel[dim]) - limit;
        if (excess > 0.0) {
          const double sample_cost = cfg.velocity_weight * excess * excess;
          Eigen::VectorXd grad_vel = Eigen::VectorXd::Zero(dof);
          grad_vel[dim] =
              cfg.velocity_weight * 2.0 * excess * (vel[dim] >= 0.0 ? 1.0 : -1.0);
          cost += weight * step * sample_cost;
          gdC->block(8 * i, 0, 8, dof) +=
              weight * step * polynomialBasis(t, 1) * grad_vel.transpose();
          (*gdT)[i] +=
              weight *
              (sample_cost / static_cast<double>(samples_per_piece) +
               step * alpha * grad_vel.dot(acc));
        }
      }

      for (Eigen::Index dim = 0; dim < dof; ++dim) {
        const double limit =
            dim < kBaseDof ? 2.0 * velocityLimitForIndex(dim, cfg)
                           : 2.0 * std::max(0.0, cfg.max_arm_qdot);
        const double excess = std::abs(acc[dim]) - limit;
        if (excess > 0.0) {
          const double sample_cost = 0.1 * cfg.velocity_weight * excess * excess;
          Eigen::VectorXd grad_acc = Eigen::VectorXd::Zero(dof);
          grad_acc[dim] =
              0.1 * cfg.velocity_weight * 2.0 * excess *
              (acc[dim] >= 0.0 ? 1.0 : -1.0);
          cost += weight * step * sample_cost;
          gdC->block(8 * i, 0, 8, dof) +=
              weight * step * polynomialBasis(t, 2) * grad_acc.transpose();
          (*gdT)[i] +=
              weight *
              (sample_cost / static_cast<double>(samples_per_piece) +
               step * alpha * grad_acc.dot(jerk));
        }
      }

      for (Eigen::Index dim = 0; dim < dof; ++dim) {
        const double limit =
            dim < kBaseDof ? 5.0 * velocityLimitForIndex(dim, cfg)
                           : 5.0 * std::max(0.0, cfg.max_arm_qdot);
        const double excess = std::abs(jerk[dim]) - limit;
        if (excess > 0.0) {
          const double sample_cost = 0.02 * cfg.velocity_weight * excess * excess;
          Eigen::VectorXd grad_jerk = Eigen::VectorXd::Zero(dof);
          grad_jerk[dim] =
              0.02 * cfg.velocity_weight * 2.0 * excess *
              (jerk[dim] >= 0.0 ? 1.0 : -1.0);
          cost += weight * step * sample_cost;
          gdC->block(8 * i, 0, 8, dof) +=
              weight * step * polynomialBasis(t, 3) * grad_jerk.transpose();
          (*gdT)[i] +=
              weight *
              (sample_cost / static_cast<double>(samples_per_piece) +
               step * alpha * grad_jerk.dot(snap));
        }
      }
    }
  }
  return cost;
}

void propagateMincoGradientsToVariables(
    const MincoWorkspace& minco,
    const Eigen::MatrixXd& gdC,
    const Eigen::VectorXd& gdT_from_cost,
    const Eigen::VectorXd& x,
    const Eigen::Index inner_count,
    const Eigen::Index dof,
    Eigen::VectorXd* gradient) {
  if (gradient == nullptr) {
    return;
  }
  Eigen::VectorXd gdT = gdT_from_cost;
  Eigen::MatrixXd adjGdC =
      minco.A.transpose().fullPivLu().solve(gdC);
  if (adjGdC.rows() != gdC.rows() || adjGdC.cols() != gdC.cols() ||
      !adjGdC.allFinite()) {
    return;
  }
  addMincoTimePropagation(minco, adjGdC, &gdT);
  for (Eigen::Index i = 0; i < inner_count; ++i) {
    const Eigen::Index offset = i * dof;
    gradient->segment(offset, dof) +=
        adjGdC.row(8 * static_cast<int>(i) + 7).transpose();
  }
  for (Eigen::Index i = 0; i < gdT.size(); ++i) {
    addDurationGradient(
        static_cast<std::size_t>(i),
        gdT[i],
        inner_count,
        dof,
        x,
        gradient);
  }
}

void addWaypointGradient(
    const std::size_t waypoint_index,
    const Eigen::VectorXd& g_q,
    const Eigen::Index inner_count,
    const Eigen::Index dof,
    Eigen::VectorXd* gradient) {
  if (gradient == nullptr || waypoint_index == 0u ||
      g_q.size() != dof) {
    return;
  }
  const Eigen::Index inner_index =
      static_cast<Eigen::Index>(waypoint_index) - 1;
  if (inner_index < 0 || inner_index >= inner_count) {
    return;
  }
  const Eigen::Index offset = inner_index * dof;
  if (offset < 0 || offset + dof > gradient->size()) {
    return;
  }
  gradient->segment(offset, dof) += g_q;
}

void addDurationGradient(
    const std::size_t piece_index,
    const double g_t,
    const Eigen::Index inner_count,
    const Eigen::Index dof,
    const Eigen::VectorXd& x,
    Eigen::VectorXd* gradient) {
  if (gradient == nullptr) {
    return;
  }
  const Eigen::Index offset =
      inner_count * dof + static_cast<Eigen::Index>(piece_index);
  if (offset < 0 || offset >= gradient->size()) {
    return;
  }
  (*gradient)[offset] += g_t * virtualTimeToRealTimeDerivative(x[offset]);
}

void addReferenceCostGradient(
    const WholeBodyLbfgsOptimizer::Input& input,
    const WholeBodyLbfgsOptimizer::Config& cfg,
    const WholeBodyLbfgsOptimizer::StateList& waypoints,
    const Eigen::Index dof,
    CostGradient* result) {
  if (result == nullptr) {
    return;
  }
  const Eigen::Index inner_count =
      static_cast<Eigen::Index>(waypoints.size()) - 2;
  const std::size_t count = std::min(waypoints.size(), input.references.size());
  for (std::size_t i = 0; i < count; ++i) {
    if (waypoints[i].size() != dof || input.references[i].size() != dof) {
      continue;
    }
    Eigen::VectorXd d = waypoints[i] - input.references[i];
    if (d.size() >= kBaseDof) {
      d[2] = normalizeAngle(waypoints[i][2] - input.references[i][2]);
    }
    Eigen::VectorXd weighted_grad = Eigen::VectorXd::Zero(dof);
    if (dof >= 1) {
      result->cost += cfg.base_reference_weight * d[0] * d[0];
      weighted_grad[0] = 2.0 * cfg.base_reference_weight * d[0];
    }
    if (dof >= 2) {
      result->cost += cfg.base_reference_weight * d[1] * d[1];
      weighted_grad[1] = 2.0 * cfg.base_reference_weight * d[1];
    }
    if (dof >= 3) {
      result->cost += cfg.yaw_reference_weight * d[2] * d[2];
      weighted_grad[2] = 2.0 * cfg.yaw_reference_weight * d[2];
    }
    if (dof > kBaseDof && cfg.reference_weight > 0.0) {
      result->cost +=
          cfg.reference_weight * d.tail(dof - kBaseDof).squaredNorm();
      weighted_grad.tail(dof - kBaseDof) =
          2.0 * cfg.reference_weight * d.tail(dof - kBaseDof);
    }
    addWaypointGradient(
        i,
        weighted_grad,
        inner_count,
        dof,
        &result->gradient);
  }
}

void addBaseProgressCostGradient(
    const WholeBodyLbfgsOptimizer::Input& input,
    const WholeBodyLbfgsOptimizer::Config& cfg,
    const WholeBodyLbfgsOptimizer::StateList& waypoints,
    const Eigen::Index dof,
    CostGradient* result) {
  if (result == nullptr || cfg.base_progress_weight <= 0.0 ||
      dof < kBaseDof || waypoints.size() < 2u ||
      input.references.size() != waypoints.size()) {
    return;
  }
  Eigen::Vector2d tangent = input.base_progress_direction;
  if (tangent.norm() < 1.0e-6) {
    tangent = input.references.back().head<2>() -
              input.references.front().head<2>();
  }
  const double tangent_norm = tangent.norm();
  if (tangent_norm < 1.0e-6) {
    return;
  }
  tangent /= tangent_norm;

  const Eigen::Index inner_count =
      static_cast<Eigen::Index>(waypoints.size()) - 2;
  const double tol = std::max(0.0, input.base_progress_tolerance);
  const Eigen::Vector2d p0 = waypoints.front().head<2>();
  const Eigen::Vector2d r0 = input.references.front().head<2>();

  auto addBaseGrad = [&](const std::size_t waypoint_index,
                         const Eigen::Vector2d& grad_xy) {
    Eigen::VectorXd g = Eigen::VectorXd::Zero(dof);
    g.head<2>() = grad_xy;
    addWaypointGradient(waypoint_index, g, inner_count, dof, &result->gradient);
  };

  for (std::size_t i = 1u; i < waypoints.size(); ++i) {
    if (waypoints[i].size() != dof || input.references[i].size() != dof) {
      continue;
    }
    const double actual =
        (waypoints[i].head<2>() - p0).dot(tangent);
    const double expected =
        (input.references[i].head<2>() - r0).dot(tangent);
    const double deficit = expected - actual - tol;
    if (deficit > 0.0) {
      result->cost += cfg.base_progress_weight * deficit * deficit;
      addBaseGrad(i, -2.0 * cfg.base_progress_weight * deficit * tangent);
    }
  }

  double prev_progress = 0.0;
  for (std::size_t i = 1u; i < waypoints.size(); ++i) {
    if (waypoints[i].size() != dof) {
      continue;
    }
    const double progress =
        (waypoints[i].head<2>() - p0).dot(tangent);
    const double deficit = prev_progress - progress - tol;
    if (deficit > 0.0) {
      result->cost += cfg.base_progress_weight * deficit * deficit;
      if (i > 1u) {
        addBaseGrad(i - 1u,
                    2.0 * cfg.base_progress_weight * deficit * tangent);
      }
      addBaseGrad(i, -2.0 * cfg.base_progress_weight * deficit * tangent);
    }
    prev_progress = std::max(prev_progress, progress);
  }
}

double velocityLimitForIndex(
    const Eigen::Index index,
    const WholeBodyLbfgsOptimizer::Config& cfg) {
  if (index == 0) {
    return std::max(0.0, cfg.max_base_vx);
  }
  if (index == 1) {
    return std::max(0.0, cfg.max_base_vy);
  }
  if (index == 2) {
    return std::max(0.0, cfg.max_base_wz);
  }
  return std::max(0.0, cfg.max_arm_qdot);
}

CostGradient evaluateCostGradient(
    const WholeBodyLbfgsOptimizer::Input& input,
    const WholeBodyLbfgsOptimizer::Config& cfg,
    const Eigen::VectorXd& q_start,
    const Eigen::VectorXd& q_goal,
    const Eigen::VectorXd& x) {
  CostGradient result;
  WholeBodyLbfgsOptimizer::StateList waypoints;
  std::vector<double> durations;
  if (!unpackVariables(
          x,
          q_start,
          q_goal,
          input.q_min,
          input.q_max,
          &waypoints,
          &durations)) {
    return result;
  }
  result.gradient = Eigen::VectorXd::Zero(x.size());
  result.cost = 0.0;

  WholeBodyPolynomialTrajectory trajectory;
  if (!trajectory.reset(waypoints, durations)) {
    return result;
  }
  MincoWorkspace minco;
  if (!buildMincoWorkspace(waypoints, durations, &minco)) {
    return result;
  }

  const Eigen::Index dof = q_start.size();
  addReferenceCostGradient(input, cfg, waypoints, dof, &result);
  addBaseProgressCostGradient(input, cfg, waypoints, dof, &result);
  Eigen::MatrixXd gdC = Eigen::MatrixXd::Zero(minco.coeffs.rows(), dof);
  Eigen::VectorXd gdT = Eigen::VectorXd::Zero(durations.size());
  result.cost += addMincoSnapGradCost(cfg, minco, &gdC, &gdT);
  result.cost += addSampledMincoGradCost(input, cfg, minco, &gdC, &gdT);

  const Eigen::Index inner_count =
      static_cast<Eigen::Index>(waypoints.size() - 2u);
  propagateMincoGradientsToVariables(
      minco,
      gdC,
      gdT,
      x,
      inner_count,
      dof,
      &result.gradient);
  if (cfg.time_weight > 0.0) {
    for (std::size_t i = 0; i < durations.size(); ++i) {
      result.cost += cfg.time_weight * durations[i];
      addDurationGradient(
          i, cfg.time_weight, inner_count, dof, x, &result.gradient);
    }
  }
  return result;
}

Eigen::VectorXd lbfgsDirection(
    const Eigen::VectorXd& gradient,
    const std::deque<Eigen::VectorXd>& s_history,
    const std::deque<Eigen::VectorXd>& y_history,
    const std::deque<double>& rho_history) {
  if (s_history.empty()) {
    return -gradient;
  }
  Eigen::VectorXd q = gradient;
  std::vector<double> alpha(s_history.size(), 0.0);
  for (int i = static_cast<int>(s_history.size()) - 1; i >= 0; --i) {
    alpha[static_cast<std::size_t>(i)] =
        rho_history[static_cast<std::size_t>(i)] *
        s_history[static_cast<std::size_t>(i)].dot(q);
    q -= alpha[static_cast<std::size_t>(i)] *
         y_history[static_cast<std::size_t>(i)];
  }

  const Eigen::VectorXd& last_s = s_history.back();
  const Eigen::VectorXd& last_y = y_history.back();
  const double yy = last_y.dot(last_y);
  const double gamma = yy > 1.0e-12 ? last_s.dot(last_y) / yy : 1.0;
  Eigen::VectorXd r = gamma * q;

  for (std::size_t i = 0; i < s_history.size(); ++i) {
    const double beta = rho_history[i] * y_history[i].dot(r);
    r += s_history[i] * (alpha[i] - beta);
  }
  return -r;
}

}  // namespace

WholeBodyLbfgsOptimizer::WholeBodyLbfgsOptimizer()
    : WholeBodyLbfgsOptimizer(Config{}) {}

WholeBodyLbfgsOptimizer::WholeBodyLbfgsOptimizer(Config config)
    : config_(config) {}

bool WholeBodyLbfgsOptimizer::optimize(
    const Input& input,
    Output* output) const {
  if (output == nullptr) {
    return false;
  }
  *output = Output{};
  if (input.initial_trajectory.empty()) {
    return false;
  }

  StateList initial_waypoints = input.initial_trajectory.controlPoints();
  if (initial_waypoints.size() < 2u) {
    return false;
  }
  for (Eigen::VectorXd& q : initial_waypoints) {
    q = clampState(q, input.q_min, input.q_max);
  }
  std::vector<double> initial_durations =
      input.initial_trajectory.pieceDurations();
  if (initial_durations.size() + 1u != initial_waypoints.size()) {
    initial_durations =
        uniformDurations(initial_waypoints.size() - 1u,
                         input.initial_trajectory.dt());
  }

  Eigen::VectorXd x;
  if (!packVariables(initial_waypoints, initial_durations, &x)) {
    return false;
  }
  if (x.size() <= 0) {
    output->trajectory = input.initial_trajectory;
    output->final_cost = output->initial_cost;
    output->ok = pathValid(input, input.initial_trajectory);
    return output->ok;
  }

  const Eigen::VectorXd q_start = initial_waypoints.front();
  const Eigen::VectorXd q_goal = initial_waypoints.back();
  auto buildTrajectory = [&](const Eigen::VectorXd& vars,
                             WholeBodyPolynomialTrajectory* trajectory) {
    StateList waypoints;
    std::vector<double> durations;
    if (!unpackVariables(
            vars,
            q_start,
            q_goal,
            input.q_min,
            input.q_max,
            &waypoints,
            &durations)) {
      return false;
    }
    return trajectory != nullptr && trajectory->reset(waypoints, durations);
  };

  CostGradient current = evaluateCostGradient(input, config_, q_start, q_goal, x);
  output->initial_cost = current.cost;
  if (!std::isfinite(current.cost) ||
      current.gradient.size() != x.size() ||
      !current.gradient.allFinite()) {
    return false;
  }

  WholeBodyPolynomialTrajectory best_trajectory;
  if (!buildTrajectory(x, &best_trajectory)) {
    return false;
  }
  double best_cost = current.cost;
  std::deque<Eigen::VectorXd> s_history;
  std::deque<Eigen::VectorXd> y_history;
  std::deque<double> rho_history;
  constexpr std::size_t kLbfgsMemory = 16u;
  constexpr double kArmijoC1 = 1.0e-4;

  for (int iter = 0; iter < std::max(0, config_.max_iterations); ++iter) {
    Eigen::VectorXd direction =
        lbfgsDirection(current.gradient, s_history, y_history, rho_history);
    double directional_derivative = current.gradient.dot(direction);
    if (!direction.allFinite() || directional_derivative >= 0.0) {
      direction = -current.gradient;
      directional_derivative = current.gradient.dot(direction);
    }
    if (!direction.allFinite() || directional_derivative >= 0.0 ||
        current.gradient.norm() < 1.0e-8) {
      break;
    }

    bool improved = false;
    double step = std::max(config_.min_step_size, config_.initial_step_size);
    Eigen::VectorXd accepted_x = x;
    CostGradient accepted;
    WholeBodyPolynomialTrajectory accepted_trajectory;
    while (step >= config_.min_step_size) {
      Eigen::VectorXd candidate = x + step * direction;
      WholeBodyPolynomialTrajectory candidate_trajectory;
      if (!buildTrajectory(candidate, &candidate_trajectory)) {
        step *= 0.5;
        continue;
      }
      accepted = evaluateCostGradient(input, config_, q_start, q_goal, candidate);
      if (std::isfinite(accepted.cost) &&
          accepted.cost <= current.cost + kArmijoC1 * step * directional_derivative) {
        if (!std::isfinite(accepted.cost) ||
            accepted.gradient.size() != x.size() ||
            !accepted.gradient.allFinite()) {
          step *= 0.5;
          continue;
        }
        accepted_x = std::move(candidate);
        accepted_trajectory = candidate_trajectory;
        improved = true;
        break;
      }
      step *= 0.5;
    }
    output->iterations = iter + 1;
    if (!improved) {
      break;
    }

    const Eigen::VectorXd s = accepted_x - x;
    const Eigen::VectorXd y = accepted.gradient - current.gradient;
    const double ys = y.dot(s);
    if (std::isfinite(ys) && ys > 1.0e-10) {
      if (s_history.size() >= kLbfgsMemory) {
        s_history.pop_front();
        y_history.pop_front();
        rho_history.pop_front();
      }
      s_history.push_back(s);
      y_history.push_back(y);
      rho_history.push_back(1.0 / ys);
    }

    x = std::move(accepted_x);
    current = std::move(accepted);
    best_trajectory = accepted_trajectory;
    best_cost = current.cost;
  }

  output->trajectory = best_trajectory;
  output->final_cost = best_cost;
  output->ok = pathValid(input, best_trajectory);
  return output->ok;
}

}  // namespace arm_controller::controller::reactive_task
