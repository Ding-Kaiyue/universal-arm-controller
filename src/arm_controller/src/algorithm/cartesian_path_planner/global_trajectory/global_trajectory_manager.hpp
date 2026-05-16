#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "algorithm/cartesian_path_planner/types.hpp"
#include "algorithm/global_planner/global_planner_interface.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

struct GlobalTrajectoryConfig {
  // Control loop period in seconds. Example: NEO 4ms -> 0.004.
  double control_cycle_sec{0.004};
  // Constant planning latency budget in seconds (REMANI-style).
  // Example conservative start: 0.06 (60ms).
  double planning_latency_sec{0.06};
};

class GlobalTrajectoryManager {
public:
  explicit GlobalTrajectoryManager(
      std::shared_ptr<arm_controller::algorithm::global_planner::GlobalPlannerInterface> planner);

  void setConfig(const GlobalTrajectoryConfig &cfg);

  // Build first active global trajectory segment.
  bool start(const PathPlanningInput &first_input,
             std::string *error = nullptr);

  // Plan a fresh global trajectory segment using provided start/goal.
  bool planSegment(const PathPlanningInput &input,
                   std::string *error = nullptr);

  // Build next segment into pending buffer from an explicit start state
  // already filled in `goal_input` (for example, real-time joint feedback FK).
  bool preparePendingSegment(const PathPlanningInput &goal_input,
                             std::string *error = nullptr);

  // Commit prepared pending segment to active segment.
  bool commitPendingSegment(std::string *error = nullptr);

  // Sample current active trajectory by point index.
  bool sample(int point_index, TimedCartesianSample &out) const;
  // Sample current active joint reference by elapsed pseudo-time. The active
  // global path may contain only sparse RRT-Connect waypoints, so this returns
  // a continuous joint-space interpolation between the timed waypoints.
  bool sampleByElapsedTime(double elapsed_sec, TimedCartesianSample &out) const;
  // Find the active-trajectory waypoint whose joint target best matches
  // q_current.
  int nearestPointIndexByJointTarget(const Eigen::VectorXd &q_current,
                                     int hint_index = 0,
                                     int search_window = 12) const;
  // Get cumulative time at the given active-trajectory point index.
  double timeAtPointIndex(int point_index) const;
  // Convert elapsed execution time (seconds) to active segment point index.
  int pointIndexAtTime(double elapsed_sec) const;
  // Get total duration (seconds) of active segment.
  double activeSegmentTotalDurationSec() const;
  PlannedSegmentKind activeSegmentKind() const;

  bool hasActiveTrajectory() const;
  int activeSegmentPointCount() const;

private:
  struct TimedSegment {
    TimedJointTrajectory traj;
    PathPlanningInput::JointToPoseFn joint_to_pose_fn;
  };

  bool populateSamplePose(const PathPlanningInput::JointToPoseFn &joint_to_pose_fn,
                          TimedCartesianSample &sample) const;
  bool sampleFromSegment(const TimedSegment &segment, int point_index,
                         TimedCartesianSample &out) const;

  GlobalTrajectoryConfig cfg_{};
  std::shared_ptr<arm_controller::algorithm::global_planner::GlobalPlannerInterface> planner_;

  mutable std::mutex state_mutex_;
  bool started_{false};
  TimedSegment active_;
  TimedSegment pending_;
};

} // namespace arm_controller::algorithm::cartesian_path_planner
