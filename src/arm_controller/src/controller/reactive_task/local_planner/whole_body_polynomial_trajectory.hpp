#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <vector>

namespace arm_controller::controller::reactive_task {

class WholeBodyPolynomialTrajectory {
public:
  using StateList =
      std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>>;
  using CoefficientMatrix = Eigen::Matrix<double, 8, Eigen::Dynamic>;
  using CoefficientMatrixList =
      std::vector<CoefficientMatrix, Eigen::aligned_allocator<CoefficientMatrix>>;

  bool reset(const StateList& waypoints, double dt_sec);
  bool reset(const StateList& waypoints, const std::vector<double>& durations);

  [[nodiscard]] bool empty() const { return control_points_.size() < 2u; }
  [[nodiscard]] std::size_t size() const { return control_points_.size(); }
  [[nodiscard]] double dt() const { return dt_sec_; }
  [[nodiscard]] double duration() const;

  [[nodiscard]] const StateList& controlPoints() const {
    return control_points_;
  }
  [[nodiscard]] StateList& mutableControlPoints() { return control_points_; }
  [[nodiscard]] const std::vector<double>& pieceDurations() const {
    return piece_durations_;
  }
  [[nodiscard]] const CoefficientMatrixList& coefficientMatrices() const {
    return segment_coefficients_;
  }

  [[nodiscard]] Eigen::VectorXd sample(double time_sec) const;
  [[nodiscard]] Eigen::VectorXd sampleDerivative(
      double time_sec,
      int derivative) const;
  [[nodiscard]] StateList sampleUniform(std::size_t sample_count) const;

private:
  bool rebuildPolynomialSegments();
  bool rebuildMincoLikeSegments();

  StateList control_points_;
  StateList waypoint_velocities_;
  StateList waypoint_accelerations_;
  StateList waypoint_jerks_;
  CoefficientMatrixList segment_coefficients_;
  std::vector<double> piece_durations_;
  std::vector<double> cumulative_times_;
  double dt_sec_{0.05};
};

}  // namespace arm_controller::controller::reactive_task
