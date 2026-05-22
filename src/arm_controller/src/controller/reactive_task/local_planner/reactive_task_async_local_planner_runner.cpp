#include "controller/reactive_task/local_planner/reactive_task_async_local_planner_runner.hpp"

#include <utility>

namespace arm_controller::controller::reactive_task {

bool AsyncLocalPlannerRunner::dueForRequest(
    const LocalPlannerRuntime& runtime,
    const double now_sec,
    const double period_sec) const {
  return !pending_valid_ &&
         (!runtime.last_output_valid ||
          (now_sec - runtime.last_update_time_sec) >= period_sec - 1e-9);
}

void AsyncLocalPlannerRunner::start(const LocalReferencePlanner& planner,
                                    LocalReferencePlannerInput&& input,
                                    const int tick,
                                    const double request_time_sec) {
  const Eigen::VectorXd q_start = input.q_current;
  const Eigen::VectorXd q_goal = input.q_goal;
  ++generation_;
  pending_generation_ = generation_;
  pending_tick_ = tick;
  pending_request_time_sec_ = request_time_sec;
  pending_q_start_ = q_start;
  pending_q_goal_ = q_goal;
  pending_future_ = std::async(
      std::launch::async,
      [&planner, input = std::move(input)]() mutable {
        ArmLocalPlanner::Output output;
        if (!planner.compute(input, &output)) {
          output.ok = false;
        }
        return output;
      });
  pending_valid_ = true;
}

void AsyncLocalPlannerRunner::handleTerminalTracking(
    LocalPlannerRuntime* runtime,
    const bool entered_terminal_tracking,
    const std::string& mapping,
    const rclcpp::Logger logger) {
  if (entered_terminal_tracking) {
    if (runtime != nullptr) {
      runtime->clearTracking();
    }
    advanceGeneration();
  }
  dropReady(mapping, logger, "terminal goal tracking");
}

void AsyncLocalPlannerRunner::dropReady(const std::string& mapping,
                                        const rclcpp::Logger logger,
                                        const char* reason) {
  if (!pendingReady()) {
    return;
  }
  (void)pending_future_.get();
  pending_valid_ = false;
  RCLCPP_DEBUG(logger,
               "[%s] local_trajopt async result dropped: %s",
               mapping.c_str(),
               reason != nullptr ? reason : "unspecified");
}

AsyncLocalPlannerRunner::ApplyReadyResult AsyncLocalPlannerRunner::applyReady(
    LocalPlannerRuntime* runtime,
    ApplyReadyInput input) {
  ApplyReadyResult result;
  if (runtime == nullptr) {
    return result;
  }

  ArmLocalPlanner::Output output;
  if (!takeReadyOutput(&output)) {
    return result;
  }
  result.had_ready_output = true;

  if (!outputCurrent()) {
    result.stale_generation = true;
    RCLCPP_DEBUG(input.logger,
                 "[%s] local_trajopt async result dropped: stale generation",
                 input.mapping.c_str());
    return result;
  }
  if (!output.ok) {
    result.planner_failed = true;
    RCLCPP_WARN(input.logger,
                "[%s] local_trajopt failed at tick %d: %s",
                input.mapping.c_str(),
                pending_tick_,
                output.error.c_str());
    return result;
  }
  if (!output.used) {
    return result;
  }

  LocalPlannerRuntime::ApplyOutputInput apply_input;
  apply_input.mapping = input.mapping;
  apply_input.logger = input.logger;
  apply_input.clock = input.clock;
  apply_input.output = std::move(output);
  apply_input.planner_start_state = pending_q_start_;
  apply_input.q_goal = pending_q_goal_;
  apply_input.current_qdot_reference = input.current_qdot_reference;
  apply_input.current_qdot_reference_valid =
      input.current_qdot_reference_valid;
  apply_input.planner_tick = pending_tick_;
  apply_input.now_sec = input.now_sec;
  apply_input.request_time_sec = pending_request_time_sec_;
  apply_input.max_usable_age_sec = input.max_usable_age_sec;
  const LocalPlannerRuntime::ApplyOutputResult apply_result =
      runtime->applyOutput(std::move(apply_input));
  result.accepted = apply_result.accepted;
  result.nominal_twist = apply_result.nominal_twist;
  return result;
}

bool AsyncLocalPlannerRunner::pendingReady() const {
  return pending_valid_ && pending_future_.valid() &&
         pending_future_.wait_for(std::chrono::seconds(0)) ==
             std::future_status::ready;
}

bool AsyncLocalPlannerRunner::takeReadyOutput(ArmLocalPlanner::Output* output) {
  if (!pendingReady() || output == nullptr) {
    return false;
  }
  *output = pending_future_.get();
  pending_valid_ = false;
  return true;
}

}  // namespace arm_controller::controller::reactive_task
