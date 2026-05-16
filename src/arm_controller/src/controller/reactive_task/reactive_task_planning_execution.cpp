#include "reactive_task_controller.hpp"

void ReactiveTaskController::plan_and_execute(
    const std::string &mapping, const geometry_msgs::msg::Pose::SharedPtr msg) {
  PlanningSession session;
  if (!preparePlanningSession(mapping, msg, &session)) {
    return;
  }
  PlanningRuntime runtime;
  if (!initializePlanningRuntime(mapping, session, &runtime)) {
    last_execution_success_[mapping] = false;
    send_joint_velocities(
        mapping, std::vector<double>(session.ctx->joint_names.size(), 0.0));
    return;
  }

  runtime.reached_goal = runPlanningControlLoop(mapping, session, &runtime);

  if (!runtime.reached_goal && runtime.sample_ok) {
    RCLCPP_WARN(
        node_->get_logger(),
        "[%s] reactive_task stopped before reaching goal: ref_tick=%d "
        "effective_max_ticks=%d active_duration=%.3f s planner_dt=%.3f s",
        mapping.c_str(), runtime.exec_ctx.planner_tick,
        runtime.exec_ctx.effective_max_planner_ticks,
        runtime.active_duration_sec, runtime.planner_tick_sec);
  }

  send_joint_velocities(
      mapping, std::vector<double>(session.ctx->joint_names.size(), 0.0));
  last_execution_success_[mapping] = runtime.reached_goal;
}
