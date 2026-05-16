#include "reactive_task_execution_state_machine.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace arm_controller::controller::reactive_task {

ReactiveTaskExecutionStateMachine::ReactiveTaskExecutionStateMachine()
    : ReactiveTaskExecutionStateMachine(Config{}) {}

ReactiveTaskExecutionStateMachine::ReactiveTaskExecutionStateMachine(Config cfg)
    : cfg_(std::move(cfg)) {}

ExecutionStatusOutput ReactiveTaskExecutionStateMachine::describe(
    const ExecutionStatusInput& input) const {
    return describePhase(input.current_phase, input);
}

ExecutionStatusOutput ReactiveTaskExecutionStateMachine::evaluate(
    const ExecutionStatusInput& input) const {
    ExecutionStatusOutput output = describePhase(input.current_phase, input);
    output.phase = input.current_phase;

    if (input.qdot_limit_violation) {
        output.phase = ExecutionPhase::Abort;
        output.transition_reason = "qdot_limit_violation";
    } else if (input.current_phase == ExecutionPhase::Track &&
               shouldEnterHold(input)) {
        output.phase = ExecutionPhase::Hold;
        output.transition_reason = input.reference_finished
                                       ? "reference_finished"
                                       : "hard_collision_margin";
    }

    output.phase_changed = output.phase != input.current_phase;
    ExecutionStatusOutput phase_output = describePhase(output.phase, input);
    phase_output.transition_reason = output.transition_reason;
    phase_output.phase_changed = output.phase_changed;
    return phase_output;
}

bool ReactiveTaskExecutionStateMachine::shouldEnterHold(
    const ExecutionStatusInput& input) const {
    if (input.current_phase != ExecutionPhase::Track) {
        return false;
    }
    if (input.reference_finished &&
        input.pos_err_goal <= cfg_.goal_position_tolerance &&
        input.ori_err_goal <= cfg_.goal_orientation_tolerance_rad) {
        return true;
    }
    if (input.hard_collision_margin_cycles >= cfg_.hard_collision_enter_cycles &&
        std::isfinite(input.whole_body_min_margin) &&
        input.whole_body_min_margin <= cfg_.hard_collision_enter_margin) {
        return true;
    }
    return false;
}

ExecutionStatusOutput ReactiveTaskExecutionStateMachine::describePhase(
    const ExecutionPhase phase,
    const ExecutionStatusInput& input) const {
    (void)input;
    ExecutionStatusOutput output;
    output.phase = phase;

    switch (phase) {
        case ExecutionPhase::Track:
            output.freeze_reference_progress = false;
            output.allow_lookahead = true;
            output.use_anchor_pose_only = false;
            break;
        case ExecutionPhase::Hold:
            output.freeze_reference_progress = true;
            output.allow_lookahead = false;
            output.use_anchor_pose_only = true;
            break;
        case ExecutionPhase::Abort:
            output.freeze_reference_progress = true;
            output.allow_lookahead = false;
            output.use_anchor_pose_only = true;
            break;
    }

    return output;
}

}  // namespace arm_controller::controller::reactive_task
