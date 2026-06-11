#include "controller/reactive_task/local_planner/whole_body_polynomial_trajectory.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <iterator>

#include <Eigen/LU>

namespace arm_controller::controller::reactive_task {

namespace {

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

Eigen::VectorXd interpolateState(
    const Eigen::VectorXd& a,
    const Eigen::VectorXd& b,
    const double ratio) {
  Eigen::VectorXd q = (1.0 - ratio) * a + ratio * b;
  if (q.size() >= 3) {
    q[2] = normalizeAngle(a[2] + ratio * normalizeAngle(b[2] - a[2]));
  }
  return q;
}

double factorialRatio(const int n, const int derivative) {
  double value = 1.0;
  for (int k = 0; k < derivative; ++k) {
    value *= static_cast<double>(n - k);
  }
  return value;
}

Eigen::VectorXd evaluatePolynomial(
    const WholeBodyPolynomialTrajectory::CoefficientMatrix& coeff,
    const double t,
    const int derivative) {
  if (coeff.rows() != 8 || coeff.cols() <= 0 ||
      derivative < 0 || derivative > 7) {
    return Eigen::VectorXd{};
  }
  Eigen::VectorXd q = Eigen::VectorXd::Zero(coeff.cols());
  for (int power = derivative; power < 8; ++power) {
    q += coeff.row(power).transpose() *
         factorialRatio(power, derivative) *
         std::pow(t, power - derivative);
  }
  if (q.size() >= 3) {
    q[2] = normalizeAngle(q[2]);
  }
  return q;
}

}  // namespace

bool WholeBodyPolynomialTrajectory::reset(
    const StateList& waypoints,
    const double dt_sec) {
  if (waypoints.size() < 2u || !(dt_sec > 0.0)) {
    return false;
  }
  const Eigen::Index dof = waypoints.front().size();
  if (dof <= 0) {
    return false;
  }
  for (const Eigen::VectorXd& q : waypoints) {
    if (q.size() != dof || !q.allFinite()) {
      return false;
    }
  }
  std::vector<double> durations(waypoints.size() - 1u, dt_sec);
  return reset(waypoints, durations);
}

bool WholeBodyPolynomialTrajectory::reset(
    const StateList& waypoints,
    const std::vector<double>& durations) {
  if (waypoints.size() < 2u || durations.size() + 1u != waypoints.size()) {
    return false;
  }
  const Eigen::Index dof = waypoints.front().size();
  if (dof <= 0) {
    return false;
  }
  for (const Eigen::VectorXd& q : waypoints) {
    if (q.size() != dof || !q.allFinite()) {
      return false;
    }
  }
  for (const double duration : durations) {
    if (!(duration > 0.0) || !std::isfinite(duration)) {
      return false;
    }
  }
  control_points_ = waypoints;
  piece_durations_ = durations;
  cumulative_times_.clear();
  cumulative_times_.reserve(piece_durations_.size() + 1u);
  cumulative_times_.push_back(0.0);
  for (const double duration : piece_durations_) {
    cumulative_times_.push_back(cumulative_times_.back() + duration);
  }
  dt_sec_ = cumulative_times_.back() /
            static_cast<double>(std::max<std::size_t>(1u, piece_durations_.size()));
  return rebuildPolynomialSegments();
}

double WholeBodyPolynomialTrajectory::duration() const {
  if (cumulative_times_.empty()) {
    return 0.0;
  }
  return cumulative_times_.back();
}

Eigen::VectorXd WholeBodyPolynomialTrajectory::sample(
    const double time_sec) const {
  return sampleDerivative(time_sec, 0);
}

Eigen::VectorXd WholeBodyPolynomialTrajectory::sampleDerivative(
    const double time_sec,
    const int derivative) const {
  if (control_points_.empty()) {
    return Eigen::VectorXd{};
  }
  if (derivative < 0 || derivative > 7) {
    return Eigen::VectorXd{};
  }
  if (control_points_.size() == 1u || time_sec <= 0.0) {
    if (derivative > 0) {
      return Eigen::VectorXd::Zero(control_points_.front().size());
    }
    return control_points_.front();
  }
  const double total = duration();
  if (time_sec >= total) {
    if (derivative > 0 && !segment_coefficients_.empty()) {
      return evaluatePolynomial(segment_coefficients_.back(),
                                piece_durations_.back(),
                                derivative);
    }
    return control_points_.back();
  }
  if (piece_durations_.empty() ||
      cumulative_times_.size() != control_points_.size() ||
      segment_coefficients_.size() != piece_durations_.size()) {
    if (derivative > 0) {
      return Eigen::VectorXd::Zero(control_points_.front().size());
    }
    const double u = time_sec / std::max(1.0e-9, dt_sec_);
    const auto i0 = static_cast<std::size_t>(
        std::clamp(
            static_cast<int>(std::floor(u)),
            0,
            static_cast<int>(control_points_.size() - 2u)));
    const double ratio = std::clamp(u - static_cast<double>(i0), 0.0, 1.0);
    return interpolateState(control_points_[i0], control_points_[i0 + 1u], ratio);
  }
  const auto upper = std::upper_bound(
      cumulative_times_.begin(), cumulative_times_.end(), time_sec);
  const auto i0 = static_cast<std::size_t>(
      std::clamp(
          static_cast<int>(std::distance(cumulative_times_.begin(), upper)) - 1,
          0,
          static_cast<int>(piece_durations_.size() - 1u)));
  const double t0 = cumulative_times_[i0];
  const double duration = std::max(1.0e-9, piece_durations_[i0]);
  const double local_t = std::clamp(time_sec - t0, 0.0, duration);
  return evaluatePolynomial(segment_coefficients_[i0], local_t, derivative);
}

WholeBodyPolynomialTrajectory::StateList
WholeBodyPolynomialTrajectory::sampleUniform(
    const std::size_t sample_count) const {
  StateList samples;
  if (sample_count == 0u || control_points_.empty()) {
    return samples;
  }
  samples.reserve(sample_count);
  if (sample_count == 1u) {
    samples.push_back(control_points_.front());
    return samples;
  }
  const double total = duration();
  for (std::size_t i = 0; i < sample_count; ++i) {
    const double ratio =
        static_cast<double>(i) / static_cast<double>(sample_count - 1u);
    samples.push_back(sample(total * ratio));
  }
  return samples;
}

bool WholeBodyPolynomialTrajectory::rebuildPolynomialSegments() {
  return rebuildMincoLikeSegments();
}

bool WholeBodyPolynomialTrajectory::rebuildMincoLikeSegments() {
  waypoint_velocities_.clear();
  waypoint_accelerations_.clear();
  waypoint_jerks_.clear();
  segment_coefficients_.clear();
  if (control_points_.empty()) {
    return false;
  }
  const Eigen::Index dof = control_points_.front().size();
  waypoint_velocities_.resize(
      control_points_.size(), Eigen::VectorXd::Zero(dof));
  waypoint_accelerations_.resize(
      control_points_.size(), Eigen::VectorXd::Zero(dof));
  waypoint_jerks_.resize(
      control_points_.size(), Eigen::VectorXd::Zero(dof));
  if (control_points_.size() < 2u ||
      piece_durations_.size() + 1u != control_points_.size()) {
    return false;
  }
  const int pieces = static_cast<int>(piece_durations_.size());
  Eigen::VectorXd T1(pieces), T2(pieces), T3(pieces), T4(pieces);
  Eigen::VectorXd T5(pieces), T6(pieces), T7(pieces);
  for (int i = 0; i < pieces; ++i) {
    T1[i] = std::max(1.0e-9, piece_durations_[static_cast<std::size_t>(i)]);
    T2[i] = T1[i] * T1[i];
    T3[i] = T2[i] * T1[i];
    T4[i] = T2[i] * T2[i];
    T5[i] = T4[i] * T1[i];
    T6[i] = T4[i] * T2[i];
    T7[i] = T4[i] * T3[i];
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(8 * pieces, 8 * pieces);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(8 * pieces, dof);

  A(0, 0) = 1.0;
  A(1, 1) = 1.0;
  A(2, 2) = 2.0;
  A(3, 3) = 6.0;
  B.row(0) = control_points_.front().transpose();

  for (int i = 0; i < pieces - 1; ++i) {
    A(8 * i + 4, 8 * i + 4) = 24.0;
    A(8 * i + 4, 8 * i + 5) = 120.0 * T1[i];
    A(8 * i + 4, 8 * i + 6) = 360.0 * T2[i];
    A(8 * i + 4, 8 * i + 7) = 840.0 * T3[i];
    A(8 * i + 4, 8 * i + 12) = -24.0;
    A(8 * i + 5, 8 * i + 5) = 120.0;
    A(8 * i + 5, 8 * i + 6) = 720.0 * T1[i];
    A(8 * i + 5, 8 * i + 7) = 2520.0 * T2[i];
    A(8 * i + 5, 8 * i + 13) = -120.0;
    A(8 * i + 6, 8 * i + 6) = 720.0;
    A(8 * i + 6, 8 * i + 7) = 5040.0 * T1[i];
    A(8 * i + 6, 8 * i + 14) = -720.0;
    A(8 * i + 7, 8 * i) = 1.0;
    A(8 * i + 7, 8 * i + 1) = T1[i];
    A(8 * i + 7, 8 * i + 2) = T2[i];
    A(8 * i + 7, 8 * i + 3) = T3[i];
    A(8 * i + 7, 8 * i + 4) = T4[i];
    A(8 * i + 7, 8 * i + 5) = T5[i];
    A(8 * i + 7, 8 * i + 6) = T6[i];
    A(8 * i + 7, 8 * i + 7) = T7[i];
    A(8 * i + 8, 8 * i) = 1.0;
    A(8 * i + 8, 8 * i + 1) = T1[i];
    A(8 * i + 8, 8 * i + 2) = T2[i];
    A(8 * i + 8, 8 * i + 3) = T3[i];
    A(8 * i + 8, 8 * i + 4) = T4[i];
    A(8 * i + 8, 8 * i + 5) = T5[i];
    A(8 * i + 8, 8 * i + 6) = T6[i];
    A(8 * i + 8, 8 * i + 7) = T7[i];
    A(8 * i + 8, 8 * i + 8) = -1.0;
    A(8 * i + 9, 8 * i + 1) = 1.0;
    A(8 * i + 9, 8 * i + 2) = 2.0 * T1[i];
    A(8 * i + 9, 8 * i + 3) = 3.0 * T2[i];
    A(8 * i + 9, 8 * i + 4) = 4.0 * T3[i];
    A(8 * i + 9, 8 * i + 5) = 5.0 * T4[i];
    A(8 * i + 9, 8 * i + 6) = 6.0 * T5[i];
    A(8 * i + 9, 8 * i + 7) = 7.0 * T6[i];
    A(8 * i + 9, 8 * i + 9) = -1.0;
    A(8 * i + 10, 8 * i + 2) = 2.0;
    A(8 * i + 10, 8 * i + 3) = 6.0 * T1[i];
    A(8 * i + 10, 8 * i + 4) = 12.0 * T2[i];
    A(8 * i + 10, 8 * i + 5) = 20.0 * T3[i];
    A(8 * i + 10, 8 * i + 6) = 30.0 * T4[i];
    A(8 * i + 10, 8 * i + 7) = 42.0 * T5[i];
    A(8 * i + 10, 8 * i + 10) = -2.0;
    A(8 * i + 11, 8 * i + 3) = 6.0;
    A(8 * i + 11, 8 * i + 4) = 24.0 * T1[i];
    A(8 * i + 11, 8 * i + 5) = 60.0 * T2[i];
    A(8 * i + 11, 8 * i + 6) = 120.0 * T3[i];
    A(8 * i + 11, 8 * i + 7) = 210.0 * T4[i];
    A(8 * i + 11, 8 * i + 11) = -6.0;

    Eigen::VectorXd inner = control_points_[static_cast<std::size_t>(i + 1)];
    inner[2] = control_points_[i][2] +
               normalizeAngle(control_points_[i + 1u][2] -
                              control_points_[i][2]);
    B.row(8 * i + 7) = inner.transpose();
  }

  const int i = pieces - 1;
  A(8 * pieces - 4, 8 * pieces - 8) = 1.0;
  A(8 * pieces - 4, 8 * pieces - 7) = T1[i];
  A(8 * pieces - 4, 8 * pieces - 6) = T2[i];
  A(8 * pieces - 4, 8 * pieces - 5) = T3[i];
  A(8 * pieces - 4, 8 * pieces - 4) = T4[i];
  A(8 * pieces - 4, 8 * pieces - 3) = T5[i];
  A(8 * pieces - 4, 8 * pieces - 2) = T6[i];
  A(8 * pieces - 4, 8 * pieces - 1) = T7[i];
  A(8 * pieces - 3, 8 * pieces - 7) = 1.0;
  A(8 * pieces - 3, 8 * pieces - 6) = 2.0 * T1[i];
  A(8 * pieces - 3, 8 * pieces - 5) = 3.0 * T2[i];
  A(8 * pieces - 3, 8 * pieces - 4) = 4.0 * T3[i];
  A(8 * pieces - 3, 8 * pieces - 3) = 5.0 * T4[i];
  A(8 * pieces - 3, 8 * pieces - 2) = 6.0 * T5[i];
  A(8 * pieces - 3, 8 * pieces - 1) = 7.0 * T6[i];
  A(8 * pieces - 2, 8 * pieces - 6) = 2.0;
  A(8 * pieces - 2, 8 * pieces - 5) = 6.0 * T1[i];
  A(8 * pieces - 2, 8 * pieces - 4) = 12.0 * T2[i];
  A(8 * pieces - 2, 8 * pieces - 3) = 20.0 * T3[i];
  A(8 * pieces - 2, 8 * pieces - 2) = 30.0 * T4[i];
  A(8 * pieces - 2, 8 * pieces - 1) = 42.0 * T5[i];
  A(8 * pieces - 1, 8 * pieces - 5) = 6.0;
  A(8 * pieces - 1, 8 * pieces - 4) = 24.0 * T1[i];
  A(8 * pieces - 1, 8 * pieces - 3) = 60.0 * T2[i];
  A(8 * pieces - 1, 8 * pieces - 2) = 120.0 * T3[i];
  A(8 * pieces - 1, 8 * pieces - 1) = 210.0 * T4[i];
  B.row(8 * pieces - 4) = control_points_.back().transpose();

  const Eigen::MatrixXd coeffs = A.fullPivLu().solve(B);
  if (coeffs.rows() != 8 * pieces || coeffs.cols() != dof ||
      !coeffs.allFinite()) {
    return false;
  }

  segment_coefficients_.reserve(piece_durations_.size());
  for (int piece = 0; piece < pieces; ++piece) {
    const CoefficientMatrix coeff =
        coeffs.block(8 * piece, 0, 8, dof);
    segment_coefficients_.push_back(coeff);
  }
  return true;
}

}  // namespace arm_controller::controller::reactive_task
