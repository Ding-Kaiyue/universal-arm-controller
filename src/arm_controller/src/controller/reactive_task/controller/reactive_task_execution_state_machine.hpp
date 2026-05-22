#pragma once

#include "controller/reactive_task/controller/reactive_task_types.hpp"

namespace arm_controller::controller::reactive_task {

class ReactiveTaskExecutionStateMachine {
public:
    struct Config {
        double hard_collision_enter_margin{-0.030};
        int hard_collision_enter_cycles{3};
        double goal_position_tolerance{0.010};
        double goal_orientation_tolerance_rad{0.100};
    };

    ReactiveTaskExecutionStateMachine();
    explicit ReactiveTaskExecutionStateMachine(Config cfg);

    ExecutionStatusOutput describe(const ExecutionStatusInput& input) const;
    ExecutionStatusOutput evaluate(const ExecutionStatusInput& input) const;

    private:
        bool shouldEnterHold(const ExecutionStatusInput& input) const;
    ExecutionStatusOutput describePhase(
        ExecutionPhase phase,
        const ExecutionStatusInput& input) const;

    Config cfg_;
};

}  // namespace arm_controller::controller::reactive_task
