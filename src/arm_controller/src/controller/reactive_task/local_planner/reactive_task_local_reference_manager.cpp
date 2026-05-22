#include "controller/reactive_task/local_planner/reactive_task_local_reference_manager.hpp"

#include <algorithm>

namespace arm_controller::controller::reactive_task {

ReactiveTaskLocalReferenceManager::ReactiveTaskLocalReferenceManager()
    : ReactiveTaskLocalReferenceManager(Config{}) {}

ReactiveTaskLocalReferenceManager::ReactiveTaskLocalReferenceManager(Config cfg)
    : cfg_(std::move(cfg)) {}

bool ReactiveTaskLocalReferenceManager::compute(
    const LocalReferenceInput& input,
    LocalReferenceOutput* output) const {
    if (output == nullptr) {
        return false;
    }

    *output = LocalReferenceOutput{};
    output->tracked_reference_time_sec = input.tracked_reference_time_sec;
    output->progress_scale = computeProgressScale(input);

    if (input.global_trajectory == nullptr) {
        output->error = "global_trajectory is null";
        return false;
    }

    const double total_duration = input.global_trajectory->activeSegmentTotalDurationSec();
    output->continuous_sample_time_sec = std::min(
        input.tracked_reference_time_sec + input.planner_tick_accumulator,
        total_duration);

    if (!input.global_trajectory->sampleByElapsedTime(
            output->continuous_sample_time_sec,
            output->current_sample)) {
        output->error = "sampleByElapsedTime failed";
        return false;
    }

    output->time_hint_index =
        input.global_trajectory->pointIndexAtTime(output->continuous_sample_time_sec);
    output->anchor_index = output->time_hint_index;
    output->lookahead_index = output->time_hint_index;
    output->next_anchor_index_state = output->time_hint_index;
    output->reference_finished =
        output->continuous_sample_time_sec >= std::max(0.0, total_duration - 1e-6);
    output->path_progress =
        total_duration > 1e-6
            ? std::clamp(output->continuous_sample_time_sec / total_duration, 0.0, 1.0)
            : 1.0;

    const bool tracking_phase =
        input.phase == ExecutionPhase::Track ||
        input.phase == ExecutionPhase::Hold;
    if (tracking_phase) {
        const int search_hint =
            std::max(0, input.path_follow_joint_anchor_index_state);
        output->anchor_index = input.global_trajectory->nearestPointIndexByJointTarget(
            input.q_now,
            search_hint,
            8);
        output->anchor_index = std::max(output->anchor_index, search_hint);
        output->next_anchor_index_state =
            std::max(input.path_follow_joint_anchor_index_state, output->anchor_index);
        output->anchor_sample_valid =
            input.global_trajectory->sample(output->anchor_index, output->anchor_sample);
        if (output->anchor_sample_valid && input.phase == ExecutionPhase::Hold) {
            const double anchor_time =
                input.global_trajectory->timeAtPointIndex(output->anchor_index);
            const double max_time_lead =
                std::max(0.02, cfg_.max_constrained_reference_lead_sec);
            if (output->continuous_sample_time_sec > anchor_time + max_time_lead) {
                output->continuous_sample_time_sec = std::min(
                    total_duration,
                    anchor_time + max_time_lead);
                if (!input.global_trajectory->sampleByElapsedTime(
                        output->continuous_sample_time_sec,
                        output->current_sample)) {
                    output->error = "sampleByElapsedTime failed after anchor sync";
                    return false;
                }
                output->tracked_reference_time_sec = output->continuous_sample_time_sec;
                output->time_hint_index =
                    input.global_trajectory->pointIndexAtTime(output->continuous_sample_time_sec);
                output->reference_finished =
                    output->continuous_sample_time_sec >= std::max(0.0, total_duration - 1e-6);
                output->path_progress =
                    total_duration > 1e-6
                        ? std::clamp(output->continuous_sample_time_sec / total_duration, 0.0, 1.0)
                        : 1.0;
            }
        }
    }

    if (input.use_anchor_pose_only && output->anchor_sample_valid) {
        output->current_sample = output->anchor_sample;
        output->used_lookahead = false;
    } else if (input.allow_lookahead && !output->reference_finished) {
        const double local_guidance_lookahead_sec = cfg_.lookahead_max_sec;
        const double future_guidance_time_sec = std::min(
            output->continuous_sample_time_sec + local_guidance_lookahead_sec,
            total_duration);
        if (future_guidance_time_sec > output->continuous_sample_time_sec + 1e-4 &&
            input.global_trajectory->sampleByElapsedTime(
                future_guidance_time_sec,
                output->lookahead_sample)) {
            output->lookahead_sample_valid = true;
            output->lookahead_index =
                input.global_trajectory->pointIndexAtTime(future_guidance_time_sec);
            output->used_lookahead = true;
        }
    }

    output->ok = true;
    return true;
}

double ReactiveTaskLocalReferenceManager::computeProgressScale(
    const LocalReferenceInput& input) const {
    if (input.freeze_reference_progress) {
        return 0.0;
    }

    switch (input.phase) {
        case ExecutionPhase::Track:
            return 1.0;
        case ExecutionPhase::Hold:
        case ExecutionPhase::Abort:
            return 0.0;
    }

    return 0.0;
}

double ReactiveTaskLocalReferenceManager::smoothstep01(const double x) {
    const double clamped = std::clamp(x, 0.0, 1.0);
    return clamped * clamped * (3.0 - 2.0 * clamped);
}

}  // namespace arm_controller::controller::reactive_task
