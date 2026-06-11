#include "controller/reactive_task/local_planner/whole_body_frontend_initializer.hpp"

namespace arm_controller::controller::reactive_task {

bool WholeBodyFrontendInitializer::initialize(
    const StateList& references,
    const StateList& frontend_path,
    const double dt_sec,
    WholeBodyPolynomialTrajectory* trajectory) const {
  if (trajectory == nullptr || !(dt_sec > 0.0)) {
    return false;
  }
  if (frontend_path.size() >= 2u &&
      trajectory->reset(frontend_path, dt_sec)) {
    return true;
  }
  return references.size() >= 2u && trajectory->reset(references, dt_sec);
}

}  // namespace arm_controller::controller::reactive_task
