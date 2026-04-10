#include "algorithm/cartesian_path_planner/core/cartesian_path_planner.hpp"

#include <cmath>
#include <iostream>
#include <sstream>

namespace arm_controller::algorithm::cartesian_path_planner {

namespace {

Eigen::Vector3d rotationToRpyDeg(const Eigen::Matrix3d& R) {
    return R.eulerAngles(0, 1, 2) * (180.0 / M_PI);
}

void appendForbiddenSphereIfFar(
    std::vector<PathPlanningInput::ForbiddenSphere>& forbidden_spheres,
    const Eigen::Vector3d& center,
    const double radius) {
    const double min_separation = std::max(1e-4, 0.35 * radius);
    for (const auto& sphere : forbidden_spheres) {
        if ((sphere.center - center).norm() < min_separation) {
            return;
        }
    }
    forbidden_spheres.push_back(PathPlanningInput::ForbiddenSphere{center, radius});
}

bool validateWholeBodyPath(
    const CartesianPath& path,
    const PathPlanningInput& input,
    std::size_t& failed_waypoint_idx,
    PathPlanningInput::WholeBodyPoseDiagnostic* failed_diag) {
    if (path.waypoints.empty()) {
        failed_waypoint_idx = 0;
        if (failed_diag != nullptr) {
            *failed_diag = {};
        }
        return false;
    }

    std::optional<Eigen::VectorXd> q_prev = input.q_start_seed;
    for (std::size_t i = 1; i < path.waypoints.size(); ++i) {
        if (input.whole_body_pose_validator) {
            Eigen::VectorXd q_sol;
            if (!input.whole_body_pose_validator(
                    path.waypoints[i].position,
                    path.waypoints[i].orientation,
                    input.safe_distance,
                    q_prev,
                    q_sol)) {
                failed_waypoint_idx = i;
                if (input.whole_body_pose_diagnostic) {
                    const auto diag = input.whole_body_pose_diagnostic(
                        path.waypoints[i].position,
                        path.waypoints[i].orientation,
                        input.safe_distance,
                        q_prev);
                    const Eigen::Vector3d rpy_deg = rotationToRpyDeg(path.waypoints[i].orientation);
                    std::ostringstream oss;
                    oss << "[planner] waypoint " << i
                        << " pose=("
                        << path.waypoints[i].position.x() << ", "
                        << path.waypoints[i].position.y() << ", "
                        << path.waypoints[i].position.z() << ")"
                        << " rpy_deg=("
                        << rpy_deg.x() << ", "
                        << rpy_deg.y() << ", "
                        << rpy_deg.z() << ")";
                    if (!diag.ik_ok) {
                        oss << " reason=ik_fail";
                    } else if (!diag.collision_free) {
                        oss << " reason=" << (diag.reason.empty() ? "collision_fail" : diag.reason)
                            << " min_margin=" << diag.min_margin;
                        if (!diag.worst_link_name.empty()) {
                            oss << " worst_link=" << diag.worst_link_name;
                        }
                    } else {
                        oss << " reason=unknown";
                    }
                    std::cout << oss.str() << std::endl;
                    if (failed_diag != nullptr) {
                        *failed_diag = diag;
                    }
                } else if (failed_diag != nullptr) {
                    *failed_diag = {};
                }
                return false;
            }
            q_prev = q_sol;
        }
    }

    failed_waypoint_idx = path.waypoints.empty() ? 0 : (path.waypoints.size() - 1);
    return true;
}

}  // namespace

CartesianPathPlanner::CartesianPathPlanner(
    const PlannerCommonConfig& common_cfg,
    const AStarConfig& astar_cfg,
    const SmoothingConfig& smoothing_cfg,
    std::shared_ptr<const DistanceFieldInterface> distance_field,
    const Eigen::Vector3d& map_min)
    : common_cfg_(common_cfg),
      distance_field_(distance_field),
      astar_(astar_cfg, distance_field, map_min),
      smoother_(smoothing_cfg, distance_field),
      parameterizer_(common_cfg, distance_field) {}

PathPlanningOutput CartesianPathPlanner::planPath(const PathPlanningInput& input) {
    PathPlanningInput attempt_input = input;
    const int max_attempts = std::max(1, input.whole_body_postcheck_max_attempts);
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        auto raw = astar_.plan(attempt_input);
        if (!raw.success) {
            return raw;
        }

        raw.path = smoother_.smooth(raw.path, attempt_input);

        if (attempt_input.whole_body_pose_validator) {
            std::size_t failed_waypoint_idx = 0;
            PathPlanningInput::WholeBodyPoseDiagnostic failed_diag;
            if (!validateWholeBodyPath(
                    raw.path, attempt_input, failed_waypoint_idx, &failed_diag)) {
                std::cout << "[planner] post-validate failed at waypoint " << failed_waypoint_idx
                          << " / "
                          << (raw.path.waypoints.empty() ? 0 : raw.path.waypoints.size() - 1)
                          << std::endl;
                if (!attempt_input.whole_body_postcheck_non_blocking) {
                    if (attempt < max_attempts &&
                        failed_waypoint_idx < raw.path.waypoints.size()) {
                        if (attempt_input.whole_body_postcheck_failure_callback) {
                            attempt_input.whole_body_postcheck_failure_callback(
                                PathPlanningInput::WholeBodyPostcheckFailureEvent{
                                    attempt,
                                    max_attempts,
                                    static_cast<int>(failed_waypoint_idx),
                                    raw.path.waypoints[failed_waypoint_idx].position,
                                    raw.path.waypoints[failed_waypoint_idx].orientation,
                                    failed_diag});
                        }
                        const double forbid_radius =
                            std::max(attempt_input.whole_body_retry_forbidden_radius,
                                     attempt_input.safe_distance + 1e-3);
                        const std::size_t begin_idx =
                            (failed_waypoint_idx > 0) ? (failed_waypoint_idx - 1) : failed_waypoint_idx;
                        const std::size_t end_idx =
                            std::min(failed_waypoint_idx + 1, raw.path.waypoints.size() - 1);
                        int added_count = 0;
                        for (std::size_t idx = begin_idx; idx <= end_idx; ++idx) {
                            if (idx == 0 || idx + 1 == raw.path.waypoints.size()) {
                                continue;
                            }
                            const std::size_t before = attempt_input.forbidden_spheres.size();
                            appendForbiddenSphereIfFar(
                                attempt_input.forbidden_spheres,
                                raw.path.waypoints[idx].position,
                                forbid_radius);
                            if (attempt_input.forbidden_spheres.size() > before) {
                                ++added_count;
                            }
                        }
                        const auto& failed_wp = raw.path.waypoints[failed_waypoint_idx];
                        std::cout << "[planner] retry geometric planning attempt "
                                  << (attempt + 1) << " / " << max_attempts
                                  << " with forbidden neighborhood centered at ("
                                  << failed_wp.position.x() << ", "
                                  << failed_wp.position.y() << ", "
                                  << failed_wp.position.z() << ") r="
                                  << forbid_radius
                                  << " count_added="
                                  << added_count << std::endl;
                        continue;
                    }
                    if (attempt_input.whole_body_postcheck_failure_callback &&
                        failed_waypoint_idx < raw.path.waypoints.size()) {
                        attempt_input.whole_body_postcheck_failure_callback(
                            PathPlanningInput::WholeBodyPostcheckFailureEvent{
                                attempt,
                                max_attempts,
                                static_cast<int>(failed_waypoint_idx),
                                raw.path.waypoints[failed_waypoint_idx].position,
                                raw.path.waypoints[failed_waypoint_idx].orientation,
                                failed_diag});
                    }
                    raw.success = false;
                } else {
                    std::cout << "[planner] post-validate is non-blocking; keep geometric path"
                              << std::endl;
                }
            }
        }
        return raw;
    }
    return {};
}

TimedCartesianTrajectory CartesianPathPlanner::planTrajectory(
    const PathPlanningInput& input) {
    const auto out = planPath(input);
    if (!out.success) {
        return {};
    }
    return parameterizer_.parameterize(
        out.path, input.R_start, input.R_goal, input.safe_distance);
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
