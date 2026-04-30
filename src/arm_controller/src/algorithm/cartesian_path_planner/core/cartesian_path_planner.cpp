#include "algorithm/cartesian_path_planner/core/cartesian_path_planner.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>

#include "algorithm/cartesian_path_planner/collision/cartesian_collision_checker.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

namespace {

Eigen::Vector3d rotationToRpyDeg(const Eigen::Matrix3d& R) {
    return R.eulerAngles(0, 1, 2) * (180.0 / M_PI);
}

bool hasUsableDirection(const Eigen::Vector3d& v) {
    return v.allFinite() && v.norm() > 1e-9;
}

Eigen::Vector3d computeRetryCenter(
    const PathPlanningInput::WholeBodyPoseDiagnostic& diag,
    const Eigen::Vector3d& fallback_position,
    const double pushout_distance) {
    Eigen::Vector3d center = diag.has_failed_pose ? diag.failed_pose_world : fallback_position;
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
    if (hasUsableDirection(diag.worst_gradient_world)) {
        direction = diag.worst_gradient_world.normalized();
    } else if (diag.has_failed_pose) {
        const Eigen::Vector3d fallback_direction = center - diag.worst_point_world;
        if (hasUsableDirection(fallback_direction)) {
            direction = fallback_direction.normalized();
        }
    }
    if (pushout_distance > 0.0 && hasUsableDirection(direction)) {
        center += pushout_distance * direction;
    }
    return center;
}

Eigen::Vector3d computeLinkContactRetryCenter(
    const PathPlanningInput::WholeBodyPoseDiagnostic& diag,
    const double forbid_radius,
    const double pushout_distance) {
    Eigen::Vector3d center = diag.worst_point_world;
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
    if (hasUsableDirection(diag.worst_gradient_world)) {
        direction = diag.worst_gradient_world.normalized();
    } else if (diag.has_failed_pose) {
        const Eigen::Vector3d fallback_direction = diag.failed_pose_world - diag.worst_point_world;
        if (hasUsableDirection(fallback_direction)) {
            direction = fallback_direction.normalized();
        }
    }
    const double offset = std::max(0.5 * forbid_radius, pushout_distance);
    if (offset > 0.0 && hasUsableDirection(direction)) {
        center += offset * direction;
    }
    return center;
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

bool isForbiddenSphereTooCloseToProtectedState(
    const Eigen::Vector3d& center,
    const double radius,
    const PathPlanningInput& input) {
    const double guard_margin = std::max(0.01, 0.25 * radius);
    const double min_clearance = radius + guard_margin;
    return ((center - input.p_start).norm() < min_clearance) ||
           ((center - input.p_goal).norm() < min_clearance);
}

bool appendForbiddenSphereIfAllowed(
    std::vector<PathPlanningInput::ForbiddenSphere>& forbidden_spheres,
    const Eigen::Vector3d& center,
    const double radius,
    const PathPlanningInput& input) {
    if (isForbiddenSphereTooCloseToProtectedState(center, radius, input)) {
        return false;
    }
    const std::size_t before = forbidden_spheres.size();
    appendForbiddenSphereIfFar(forbidden_spheres, center, radius);
    return forbidden_spheres.size() > before;
}

bool validateWholeBodyPath(
    const CartesianPath& path,
    const PathPlanningInput& input,
    std::size_t& failed_waypoint_idx,
    PathPlanningInput::WholeBodyPoseDiagnostic* failed_diag);

CartesianPath makeDetourPath(
    const CartesianWaypoint& start_wp,
    const CartesianWaypoint& detour_wp,
    const CartesianWaypoint& goal_wp) {
    CartesianPath path;
    path.waypoints = {start_wp, detour_wp, goal_wp};
    path.length = (detour_wp.position - start_wp.position).norm() +
                  (goal_wp.position - detour_wp.position).norm();
    return path;
}

Eigen::Vector3d chooseFallbackAxis(const Eigen::Vector3d& direction) {
    const Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();
    if (std::abs(direction.dot(z_axis)) < 0.9) {
        return z_axis;
    }
    return Eigen::Vector3d::UnitX();
}

bool tryRepairWithDetourCandidates(
    const CartesianPath& failed_path,
    const PathPlanningInput& input,
    const PathPlanningInput::WholeBodyPoseDiagnostic& failed_diag,
    const CartesianWaypoint& failed_wp,
    const std::shared_ptr<const DistanceFieldInterface>& distance_field,
    CartesianPath* repaired_path) {
    if (repaired_path == nullptr || failed_path.waypoints.size() < 2) {
        return false;
    }

    const CartesianWaypoint& start_wp = failed_path.waypoints.front();
    const CartesianWaypoint& goal_wp = failed_path.waypoints.back();
    const Eigen::Vector3d base_position =
        failed_diag.has_failed_pose ? failed_diag.failed_pose_world : failed_wp.position;

    Eigen::Vector3d normal = failed_diag.worst_gradient_world;
    if (!hasUsableDirection(normal) && failed_diag.has_failed_pose) {
        normal = base_position - failed_diag.worst_point_world;
    }
    if (!hasUsableDirection(normal)) {
        return false;
    }
    normal.normalize();

    Eigen::Vector3d segment_dir = goal_wp.position - start_wp.position;
    if (!hasUsableDirection(segment_dir)) {
        segment_dir = goal_wp.position - base_position;
    }
    if (!hasUsableDirection(segment_dir)) {
        segment_dir = chooseFallbackAxis(normal);
    }
    segment_dir.normalize();

    Eigen::Vector3d tangent = segment_dir.cross(normal);
    if (!hasUsableDirection(tangent)) {
        tangent = chooseFallbackAxis(normal).cross(normal);
    }
    if (!hasUsableDirection(tangent)) {
        return false;
    }
    tangent.normalize();

    const CartesianCollisionChecker checker(distance_field);
    const double edge_step = 0.01;
    const double pushout = std::max(input.safe_distance + 0.01, input.whole_body_retry_pushout_distance);
    const double lateral_base =
        std::max(input.whole_body_retry_forbidden_radius * 1.5, input.safe_distance + 0.02);

    struct DetourCandidate {
        Eigen::Vector3d position;
        const char* label;
    };

    std::vector<DetourCandidate> candidates;
    candidates.push_back({base_position + normal * pushout + tangent * lateral_base, "left_tangent"});
    candidates.push_back({base_position + normal * pushout - tangent * lateral_base, "right_tangent"});
    candidates.push_back({base_position + normal * (pushout + lateral_base), "forward_normal"});
    candidates.push_back({base_position + normal * pushout + tangent * (1.8 * lateral_base), "left_tangent_wide"});
    candidates.push_back({base_position + normal * pushout - tangent * (1.8 * lateral_base), "right_tangent_wide"});

    for (const auto& candidate : candidates) {
        if ((candidate.position - input.p_start).norm() < input.goal_tolerance ||
            (candidate.position - input.p_goal).norm() < input.goal_tolerance) {
            continue;
        }
        if (!checker.isStateValid(candidate.position, input.hard_clearance, input.forbidden_spheres)) {
            continue;
        }

        CartesianWaypoint detour_wp;
        detour_wp.position = candidate.position;
        detour_wp.orientation = failed_diag.has_failed_pose
                                    ? failed_diag.failed_pose_orientation
                                    : failed_wp.orientation;

        if (!checker.isSegmentValid(
                start_wp.position,
                detour_wp.position,
                input.hard_clearance,
                edge_step,
                input.forbidden_spheres)) {
            continue;
        }
        if (!checker.isSegmentValid(
                detour_wp.position,
                goal_wp.position,
                input.hard_clearance,
                edge_step,
                input.forbidden_spheres)) {
            continue;
        }

        CartesianPath path = makeDetourPath(start_wp, detour_wp, goal_wp);
        std::size_t detour_failed_idx = 0;
        PathPlanningInput::WholeBodyPoseDiagnostic detour_failed_diag;
        if (validateWholeBodyPath(path, input, detour_failed_idx, &detour_failed_diag)) {
            *repaired_path = std::move(path);
            std::cout << "[planner] detour repair accepted label=" << candidate.label
                      << " detour=(" << detour_wp.position.x() << ", "
                      << detour_wp.position.y() << ", "
                      << detour_wp.position.z() << ")" << std::endl;
            return true;
        }

        std::cout << "[planner] detour repair rejected label=" << candidate.label
                  << " failed_waypoint=" << detour_failed_idx
                  << " detour=(" << detour_wp.position.x() << ", "
                  << detour_wp.position.y() << ", "
                  << detour_wp.position.z() << ")" << std::endl;
    }

    return false;
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

    auto logFailedDiag = [&](const std::size_t waypoint_idx,
                             const PathPlanningInput::WholeBodyPoseDiagnostic& diag) {
        const Eigen::Vector3d pose_position =
            diag.has_failed_pose ? diag.failed_pose_world : path.waypoints[waypoint_idx].position;
        const Eigen::Matrix3d pose_orientation =
            diag.has_failed_pose ? diag.failed_pose_orientation : path.waypoints[waypoint_idx].orientation;
        const Eigen::Vector3d rpy_deg = rotationToRpyDeg(pose_orientation);
        std::ostringstream oss;
        oss << "[planner] waypoint " << waypoint_idx
            << " pose=("
            << pose_position.x() << ", "
            << pose_position.y() << ", "
            << pose_position.z() << ")"
            << " rpy_deg=("
            << rpy_deg.x() << ", "
            << rpy_deg.y() << ", "
            << rpy_deg.z() << ")";
        if (diag.has_failed_pose) {
            oss << " failed_sample_t=" << diag.failed_segment_t;
        }
        if (!diag.ik_ok) {
            oss << " reason=ik_fail";
        } else if (!diag.collision_free) {
            oss << " reason=" << (diag.reason.empty() ? "collision_fail" : diag.reason)
                << " min_margin=" << diag.min_margin;
            if (!diag.worst_link_name.empty()) {
                oss << " worst_link=" << diag.worst_link_name;
            }
            oss << " worst_point=("
                << diag.worst_point_world.x() << ", "
                << diag.worst_point_world.y() << ", "
                << diag.worst_point_world.z() << ")"
                << " worst_distance=" << diag.worst_distance
                << " r_eff=" << diag.worst_effective_radius
                << " safe_distance=" << diag.safe_distance_used
                << " required_clearance=" << diag.required_clearance
                << " worst_gradient_norm=" << diag.worst_gradient_norm;
        } else {
            oss << " reason=segment_fail";
        }
        std::cout << oss.str() << std::endl;
    };

    std::optional<Eigen::VectorXd> q_prev = input.q_start_seed;
    for (std::size_t i = 1; i < path.waypoints.size(); ++i) {
        if (input.whole_body_segment_validator) {
            Eigen::VectorXd q_end;
            PathPlanningInput::WholeBodyPoseDiagnostic diag;
            if (!input.whole_body_segment_validator(
                    path.waypoints[i - 1],
                    path.waypoints[i],
                    input.safe_distance,
                    q_prev,
                    q_end,
                    &diag)) {
                failed_waypoint_idx = i;
                if (!diag.has_failed_pose && input.whole_body_pose_diagnostic) {
                    diag = input.whole_body_pose_diagnostic(
                        path.waypoints[i].position,
                        path.waypoints[i].orientation,
                        input.safe_distance,
                        q_prev);
                }
                logFailedDiag(i, diag);
                if (failed_diag != nullptr) {
                    *failed_diag = diag;
                }
                return false;
            }
            q_prev = q_end;
            continue;
        }

        if (input.whole_body_pose_validator) {
            Eigen::VectorXd q_sol;
            if (!input.whole_body_pose_validator(
                    path.waypoints[i].position,
                    path.waypoints[i].orientation,
                    input.safe_distance,
                    q_prev,
                    q_sol)) {
                failed_waypoint_idx = i;
                PathPlanningInput::WholeBodyPoseDiagnostic diag;
                if (input.whole_body_pose_diagnostic) {
                    diag = input.whole_body_pose_diagnostic(
                        path.waypoints[i].position,
                        path.waypoints[i].orientation,
                        input.safe_distance,
                        q_prev);
                    logFailedDiag(i, diag);
                }
                if (failed_diag != nullptr) {
                    *failed_diag = diag;
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
                    const auto& failed_wp = raw.path.waypoints[failed_waypoint_idx];
                    const Eigen::Vector3d failure_position =
                        failed_diag.has_failed_pose ? failed_diag.failed_pose_world : failed_wp.position;
                    const Eigen::Matrix3d failure_orientation =
                        failed_diag.has_failed_pose ? failed_diag.failed_pose_orientation
                                                    : failed_wp.orientation;
                    if (attempt < max_attempts &&
                        failed_waypoint_idx < raw.path.waypoints.size()) {
                        if (attempt_input.whole_body_postcheck_failure_callback) {
                            attempt_input.whole_body_postcheck_failure_callback(
                                PathPlanningInput::WholeBodyPostcheckFailureEvent{
                                    attempt,
                                    max_attempts,
                                    static_cast<int>(failed_waypoint_idx),
                                    failure_position,
                                    failure_orientation,
                                    failed_diag});
                        }
                        CartesianPath repaired_path;
                        if (failed_diag.has_failed_pose &&
                            tryRepairWithDetourCandidates(
                                raw.path,
                                attempt_input,
                                failed_diag,
                                failed_wp,
                                distance_field_,
                                &repaired_path)) {
                            raw.path = std::move(repaired_path);
                            return raw;
                        }
                        const double forbid_radius =
                            std::max(attempt_input.whole_body_retry_forbidden_radius,
                                     attempt_input.safe_distance + 1e-3);
                        const double pushout_distance =
                            std::max(0.0, attempt_input.whole_body_retry_pushout_distance);
                        const std::size_t begin_idx =
                            (failed_waypoint_idx > 0) ? (failed_waypoint_idx - 1) : failed_waypoint_idx;
                        const std::size_t end_idx =
                            std::min(failed_waypoint_idx + 1, raw.path.waypoints.size() - 1);
                        int added_count = 0;
                        Eigen::Vector3d retry_center = failure_position;
                        std::string retry_center_source = failed_diag.has_failed_pose
                                                              ? "segment_sample"
                                                              : "failed_waypoint";
                        if (failed_diag.has_failed_pose) {
                            const Eigen::Vector3d pose_retry_center = computeRetryCenter(
                                failed_diag, failure_position, pushout_distance);
                            const Eigen::Vector3d link_retry_center = computeLinkContactRetryCenter(
                                failed_diag, forbid_radius, pushout_distance);

                            if (appendForbiddenSphereIfAllowed(
                                    attempt_input.forbidden_spheres,
                                    link_retry_center,
                                    forbid_radius,
                                    attempt_input)) {
                                retry_center = link_retry_center;
                                retry_center_source = "link_contact_side";
                                ++added_count;
                            } else if (appendForbiddenSphereIfAllowed(
                                           attempt_input.forbidden_spheres,
                                           pose_retry_center,
                                           forbid_radius,
                                           attempt_input)) {
                                retry_center = pose_retry_center;
                                retry_center_source = "segment_sample";
                                ++added_count;
                            } else {
                                retry_center = link_retry_center;
                                retry_center_source = "protected_skip";
                                std::cout << "[planner] skip forbidden spheres near protected state link_center=("
                                          << link_retry_center.x() << ", "
                                          << link_retry_center.y() << ", "
                                          << link_retry_center.z() << ") pose_center=("
                                          << pose_retry_center.x() << ", "
                                          << pose_retry_center.y() << ", "
                                          << pose_retry_center.z() << ") r="
                                          << forbid_radius << std::endl;
                            }
                        }
                        for (std::size_t idx = begin_idx; idx <= end_idx; ++idx) {
                            if (idx == 0 || idx + 1 == raw.path.waypoints.size()) {
                                continue;
                            }
                            if (appendForbiddenSphereIfAllowed(
                                    attempt_input.forbidden_spheres,
                                    raw.path.waypoints[idx].position,
                                    forbid_radius,
                                    attempt_input)) {
                                ++added_count;
                            }
                        }
                        std::cout << "[planner] retry geometric planning attempt "
                                  << (attempt + 1) << " / " << max_attempts
                                  << " with forbidden neighborhood centered at ("
                                  << retry_center.x() << ", "
                                  << retry_center.y() << ", "
                                  << retry_center.z() << ") r="
                                  << forbid_radius
                                  << " source=" << retry_center_source
                                  << " pushout=" << pushout_distance
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
                                failure_position,
                                failure_orientation,
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
    TimedCartesianTrajectory traj = parameterizer_.parameterize(
        out.path, input.R_start, input.R_goal, input.hard_clearance);
    if (traj.empty()) {
        return {};
    }

    if (input.whole_body_pose_validator) {
        traj.waypoint_joint_targets.clear();
        traj.waypoint_joint_targets.reserve(traj.waypoints.size());

        std::optional<Eigen::VectorXd> q_prev = input.q_start_seed;
        bool all_valid = true;
        for (const auto& wp : traj.waypoints) {
            Eigen::VectorXd q_sol;
            if (!input.whole_body_pose_validator(
                    wp.position,
                    wp.orientation,
                    input.safe_distance,
                    q_prev,
                    q_sol)) {
                all_valid = false;
                break;
            }
            traj.waypoint_joint_targets.push_back(q_sol);
            q_prev = q_sol;
        }

        if (!all_valid ||
            traj.waypoint_joint_targets.size() != traj.waypoints.size()) {
            if (!input.whole_body_postcheck_non_blocking) {
                std::cout << "[planner] timed trajectory IK cache build failed in strict whole-body mode; abort trajectory"
                          << std::endl;
                return {};
            }
            std::cout << "[planner] warning: failed to build full IK waypoint cache for timed trajectory"
                      << std::endl;
            traj.waypoint_joint_targets.clear();
        }
    }

    return traj;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
