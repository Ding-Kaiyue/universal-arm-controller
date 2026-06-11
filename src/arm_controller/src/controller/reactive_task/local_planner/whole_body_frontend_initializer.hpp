#pragma once

#include "controller/reactive_task/local_planner/whole_body_polynomial_trajectory.hpp"

namespace arm_controller::controller::reactive_task {

class WholeBodyFrontendInitializer {
public:
  using StateList = WholeBodyPolynomialTrajectory::StateList;

  bool initialize(
      const StateList& references,
      const StateList& frontend_path,
      double dt_sec,
      WholeBodyPolynomialTrajectory* trajectory) const;
};

}  // namespace arm_controller::controller::reactive_task
