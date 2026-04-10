#include "algorithm/cartesian_path_planner/sampling/reference_sampler.hpp"

#include <Eigen/Geometry>
#include <algorithm>

namespace arm_controller::algorithm::cartesian_path_planner {

TimedCartesianSample ReferenceSampler::sample(
    const TimedCartesianTrajectory& traj,
    double t_query) const {
    TimedCartesianSample out;
    if (traj.empty()) {
        return out;
    }

    const double t =
        std::clamp(t_query, 0.0, std::max(0.0, traj.total_duration));
    out.time_from_start = t;

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
    const Eigen::Quaterniond q = q0.slerp(s, q1);

    out.T_target.setIdentity();
    out.T_target.linear() = q.toRotationMatrix();
    out.T_target.translation() = p;

    out.target_twist.setZero();
    out.target_twist.head<3>() = v;
    // 第一版先不给角速度解析项，保持 0；后面可补 SO(3) 对数映射差分
    out.target_twist.tail<3>().setZero();

    return out;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner