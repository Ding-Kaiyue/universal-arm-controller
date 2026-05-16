#include "algorithm/cartesian_path_planner/sampling/reference_sampler.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace arm_controller::algorithm::cartesian_path_planner {

TimedCartesianSample ReferenceSampler::sample(
    const TimedCartesianTrajectory& traj,
    double t_query) const {
    TimedCartesianSample out;
    out.is_cartesian_tracking_target = false;
    if (traj.empty()) {
        return out;
    }

    const double t =
        std::clamp(t_query, 0.0, std::max(0.0, traj.total_duration));
    out.time_from_start = t;

    const bool at_trajectory_end =
        t >= std::max(0.0, traj.total_duration) - 1e-9;

    size_t seg = 0;
    while (seg + 1 < traj.cumulative_times.size() &&
           traj.cumulative_times[seg + 1] < t) {
        ++seg;
    }

    if (seg >= traj.segment_durations.size()) {
        seg = traj.segment_durations.size() - 1;
    }

    const double t0 = traj.cumulative_times[seg];
    const double dt = traj.segment_durations[seg];
    const double s = (dt > 1e-9) ? std::clamp((t - t0) / dt, 0.0, 1.0) : 0.0;

    const auto& wp0 = traj.waypoints[seg];
    const auto& wp1 = traj.waypoints[seg + 1];

    const Eigen::Vector3d p =
        (1.0 - s) * wp0.position + s * wp1.position;
    const Eigen::Vector3d v =
        (wp1.position - wp0.position) / std::max(dt, 1e-9);

    Eigen::Quaterniond q0(wp0.orientation);
    Eigen::Quaterniond q1(wp1.orientation);
    q0.normalize();
    q1.normalize();
    if (q0.dot(q1) < 0.0) {
        q1.coeffs() *= -1.0;
    }
    const Eigen::Quaterniond q = q0.slerp(s, q1);

    out.T_target.setIdentity();
    out.T_target.linear() = q.toRotationMatrix();
    out.T_target.translation() = p;

    out.target_twist.setZero();
    if (!at_trajectory_end) {
        out.target_twist.head<3>() = v;
    }
    // Angular feedforward from segment rotation change.
    // Use body-fixed rotation vector over this segment and map it to world frame
    // at the sampled orientation to align with geometric Jacobian convention.
    if (!at_trajectory_end && dt > 1e-9) {
        const Eigen::Matrix3d R_rel = wp0.orientation.transpose() * wp1.orientation;
        const Eigen::AngleAxisd aa_rel(R_rel);
        if (std::isfinite(aa_rel.angle()) && std::abs(aa_rel.angle()) > 1e-12) {
            const Eigen::Vector3d w_body = aa_rel.axis() * (aa_rel.angle() / dt);
            out.target_twist.tail<3>() = q.toRotationMatrix() * w_body;
        }
    }

    if (traj.waypoint_joint_targets.size() == traj.waypoints.size() &&
        seg + 1 < traj.waypoint_joint_targets.size()) {
        const Eigen::VectorXd& q0 = traj.waypoint_joint_targets[seg];
        const Eigen::VectorXd& q1 = traj.waypoint_joint_targets[seg + 1];
        const bool q0_ok = (q0.size() > 0) && q0.allFinite();
        const bool q1_ok = (q1.size() > 0) && q1.allFinite();
        if (q0_ok && q1_ok && q0.size() == q1.size()) {
            out.ik_joint_target = (1.0 - s) * q0 + s * q1;
            out.has_ik_joint_target = true;
        } else if (q0_ok) {
            out.ik_joint_target = q0;
            out.has_ik_joint_target = true;
        } else if (q1_ok) {
            out.ik_joint_target = q1;
            out.has_ik_joint_target = true;
        }
    }

    return out;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
