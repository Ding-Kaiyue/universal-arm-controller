#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "algorithm/cartesian_path_planner/core/cartesian_path_planner.hpp"
#include "algorithm/cartesian_path_planner/sampling/reference_sampler.hpp"
#include "algorithm/cartesian_path_planner/types.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

struct ReplannerConfig {
    // Re-sample each planned segment by spatial step (meters).
    // Larger path length -> more sampled points.
    double segment_sample_step_m{0.03};
    // Trigger one global replan every N control ticks (e.g., NEO cycles).
    int replan_every_control_ticks{10};
    // Control loop period in seconds. Example: NEO 4ms -> 0.004.
    double control_cycle_sec{0.004};
    // Predict how many control ticks ahead from current execution point.
    int prediction_horizon_ticks{10};
    // Constant planning latency budget in seconds (REMANI-style).
    // Example conservative start: 0.06 (60ms).
    double planning_latency_sec{0.06};
    // Smooth only around new segment start (handoff point), while keeping
    // segment start as a hard anchor point.
    int handoff_blend_points{8};
};

class ReplannerManager {
public:
    explicit ReplannerManager(std::shared_ptr<CartesianPathPlanner> planner);

    void setConfig(const ReplannerConfig& cfg);

    // Build first active segment.
    bool start(
        const PathPlanningInput& first_input,
        std::string* error = nullptr);

    // Re-plan a fresh segment using provided start/goal.
    bool planSegment(
        const PathPlanningInput& input,
        std::string* error = nullptr);

    // Re-plan from real-time feedback pose (recommended for closed-loop handoff).
    bool planFromFeedbackPose(
        const PathPlanningInput& goal_input,
        const Eigen::Isometry3d& T_feedback,
        std::string* error = nullptr);

    // Re-plan from real-time joint feedback.
    // Caller typically gets q_current from HardwareManager and provides FK callback:
    //   bool fk(const std::vector<double>& q, Eigen::Isometry3d& T_out)
    bool planFromJointFeedback(
        const PathPlanningInput& goal_input,
        const std::vector<double>& q_current,
        const std::function<bool(const std::vector<double>&, Eigen::Isometry3d&)>& fk_callback,
        std::string* error = nullptr);

    // Re-plan from live joint feedback provider (recommended for lock-free reads).
    // Example provider:
    //   [&]() { return hardware_manager->get_current_joint_positions_lockfree(mapping); }
    bool planFromJointFeedbackProvider(
        const PathPlanningInput& goal_input,
        const std::function<std::vector<double>()>& q_provider,
        const std::function<bool(const std::vector<double>&, Eigen::Isometry3d&)>& fk_callback,
        std::string* error = nullptr);

    // Helper: true when control tick hits configured replan cadence.
    bool shouldReplanAtControlTick(int control_tick) const;

    // Re-plan directly from previous segment result:
    // new start is taken from the end pose of active trajectory.
    bool planFromPreviousResult(
        const PathPlanningInput& next_goal_input,
        std::string* error = nullptr);

    // Re-plan from a predicted future pose on current active trajectory:
    // t_pred = now + prediction_horizon_ticks * control_cycle_sec + planning_latency_sec.
    // In point-index form, this predicts from current executed point index.
    bool planFromPredictedActiveTrajectory(
        const PathPlanningInput& goal_input,
        int current_point_index,
        std::string* error = nullptr);

    // Build next segment into pending buffer from predicted active trajectory
    // without interrupting current active execution.
    bool preparePendingFromPredictedActiveTrajectory(
        const PathPlanningInput& goal_input,
        int current_point_index,
        std::string* error = nullptr);
    // Build next segment into pending buffer from an explicit active-trajectory
    // point index (no extra horizon prediction inside this method).
    bool preparePendingFromActiveTrajectoryPoint(
        const PathPlanningInput& goal_input,
        int start_point_index,
        std::string* error = nullptr);

    // Commit prepared pending segment to active segment.
    bool commitPendingSegment(std::string* error = nullptr);
    bool hasPendingTrajectory() const;

    // Sample predicted replan start pose used by planFromPredictedActiveTrajectory.
    bool samplePredictedReplanStart(
        int current_point_index,
        TimedCartesianSample& out) const;

    // Sample current active trajectory by point index.
    bool sample(int point_index, TimedCartesianSample& out) const;
    // Sample current active trajectory by elapsed execution time (seconds).
    bool sampleByElapsedTime(double elapsed_sec, TimedCartesianSample& out) const;
    // Convert elapsed execution time (seconds) to active segment point index.
    int pointIndexAtTime(double elapsed_sec) const;
    // Get total duration (seconds) of active segment.
    double activeSegmentTotalDurationSec() const;

    bool hasActiveTrajectory() const;
    int activeSegmentPointCount() const;

private:
    struct TimedSegment {
        TimedCartesianTrajectory traj;
    };

    PathPlanningInput makeNextInputFromHandoff(
        const PathPlanningInput& goal_input) const;

    TimedCartesianTrajectory toDistanceSampledSegment(
        const TimedCartesianTrajectory& in_traj) const;

    int predictedPointIndex(int current_point_index) const;
    int predictedPointIndex(
        const TimedCartesianTrajectory& traj,
        int current_point_index) const;

    bool sampleFromTrajectory(
        const TimedCartesianTrajectory& traj,
        int point_index,
        TimedCartesianSample& out) const;

    Eigen::Vector3d estimateTrajectoryVelocityAtPoint(
        const TimedCartesianTrajectory& traj,
        int point_index) const;

    void smoothSegmentStartForHandoff(
        TimedCartesianTrajectory& traj,
        const Eigen::Vector3d& v_start) const;

    ReplannerConfig cfg_{};
    std::shared_ptr<CartesianPathPlanner> planner_;
    ReferenceSampler sampler_;

    mutable std::mutex state_mutex_;
    bool started_{false};
    TimedSegment active_;
    TimedSegment pending_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
