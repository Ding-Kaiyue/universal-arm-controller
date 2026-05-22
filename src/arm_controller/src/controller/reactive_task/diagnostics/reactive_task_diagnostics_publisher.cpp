#include "controller/reactive_task/diagnostics/reactive_task_diagnostics_publisher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <visualization_msgs/msg/marker.hpp>

namespace arm_controller::controller::reactive_task {

namespace {

geometry_msgs::msg::Point toPointMsg(const Eigen::Vector3d& p) {
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    return point;
}

std::string markerNamespace(const std::string& mapping, const char* name) {
    const std::string scope = mapping.empty() ? "default" : mapping;
    return "reactive_task/" + scope + "/" + name;
}

void setMarkerColor(
    visualization_msgs::msg::Marker& marker,
    const float r,
    const float g,
    const float b,
    const float a) {
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
}

}  // namespace

ReactiveTaskDiagnosticsPublisher::ReactiveTaskDiagnosticsPublisher(
    const rclcpp::Node::SharedPtr& node)
    : node_(node) {
    const auto marker_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    collision_ellipsoid_marker_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/reactive_task/collision_ellipsoid_markers",
            marker_qos);
    trajectory_marker_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/reactive_task/trajectory_markers",
            marker_qos);
}

void ReactiveTaskDiagnosticsPublisher::publishCollisionEllipsoids(
    const std::string& mapping,
    const Eigen::VectorXd& q_current,
    const arm_controller::kinematics::PinocchioForwardKinematics& fk_provider,
    const rq::LinkCollisionEllipsoidList& collision_ellipsoids) {
    if (!collision_ellipsoid_marker_pub_ || collision_ellipsoids.empty() || q_current.size() <= 0) {
        return;
    }

    std::vector<std::string> link_names;
    link_names.reserve(collision_ellipsoids.size());
    for (const auto& ellipsoid : collision_ellipsoids) {
        if (std::find(link_names.begin(), link_names.end(), ellipsoid.link_name) ==
            link_names.end()) {
            link_names.push_back(ellipsoid.link_name);
        }
    }
    arm_controller::kinematics::LinkPoseResultList link_poses;
    if (!fk_provider.computeLinkPoses(q_current, link_names, link_poses)) {
        return;
    }

    visualization_msgs::msg::MarkerArray array_msg;
    std::size_t marker_count = 0u;
    const std::string marker_ns = markerNamespace(mapping, "collision_ellipsoids");

    for (const auto& ellipsoid : collision_ellipsoids) {
        const auto it = std::find_if(
            link_poses.begin(),
            link_poses.end(),
            [&ellipsoid](const arm_controller::kinematics::LinkPoseResult& result) {
                return result.link_name == ellipsoid.link_name;
            });
        if (it == link_poses.end()) {
            continue;
        }

        const Eigen::Isometry3d& T_world_link = it->pose;
        const Eigen::Vector3d center_world = T_world_link * ellipsoid.center_in_link;
        const Eigen::Quaterniond q_world(T_world_link.linear());

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = node_->now();
        marker.ns = marker_ns;
        marker.id = static_cast<int>(marker_count);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = center_world.x();
        marker.pose.position.y = center_world.y();
        marker.pose.position.z = center_world.z();
        marker.pose.orientation.x = q_world.x();
        marker.pose.orientation.y = q_world.y();
        marker.pose.orientation.z = q_world.z();
        marker.pose.orientation.w = q_world.w();
        marker.scale.x = 2.0 * ellipsoid.radii.x();
        marker.scale.y = 2.0 * ellipsoid.radii.y();
        marker.scale.z = 2.0 * ellipsoid.radii.z();
        marker.color.r = 1.0f;
        marker.color.g = 0.1f;
        marker.color.b = 0.1f;
        marker.color.a = 0.65f;
        array_msg.markers.push_back(marker);
        ++marker_count;
    }

    {
        std::lock_guard<std::mutex> lock(collision_ellipsoid_marker_mutex_);
        const std::size_t previous_count = collision_ellipsoid_marker_counts_[mapping];
        for (std::size_t i = marker_count; i < previous_count; ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = node_->now();
            marker.ns = marker_ns;
            marker.id = static_cast<int>(i);
            marker.action = visualization_msgs::msg::Marker::DELETE;
            array_msg.markers.push_back(marker);
        }
        collision_ellipsoid_marker_counts_[mapping] = marker_count;
    }

    collision_ellipsoid_marker_pub_->publish(array_msg);
}

void ReactiveTaskDiagnosticsPublisher::clearCollisionEllipsoids(const std::string& mapping) {
    if (!collision_ellipsoid_marker_pub_) {
        return;
    }

    visualization_msgs::msg::MarkerArray array_msg;
    const std::string marker_ns = markerNamespace(mapping, "collision_ellipsoids");
    std::size_t previous_count = 0u;
    {
        std::lock_guard<std::mutex> lock(collision_ellipsoid_marker_mutex_);
        const auto it = collision_ellipsoid_marker_counts_.find(mapping);
        if (it != collision_ellipsoid_marker_counts_.end()) {
            previous_count = it->second;
            collision_ellipsoid_marker_counts_.erase(it);
        }
    }
    for (std::size_t i = 0; i < previous_count; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = node_->now();
        marker.ns = marker_ns;
        marker.id = static_cast<int>(i);
        marker.action = visualization_msgs::msg::Marker::DELETE;
        array_msg.markers.push_back(marker);
    }
    if (!array_msg.markers.empty()) {
        collision_ellipsoid_marker_pub_->publish(array_msg);
    }
}

void ReactiveTaskDiagnosticsPublisher::publishTrajectory(
    const std::string& mapping,
    const cp::GlobalTrajectoryManager& global_trajectory,
    const cp::TimedCartesianSample& sample,
    const Eigen::Vector3d& ee_position) {
    if (!trajectory_marker_pub_) {
        return;
    }
    if (!global_trajectory.hasActiveTrajectory()) {
        clearTrajectory(mapping);
        return;
    }

    visualization_msgs::msg::MarkerArray array_msg;
    const auto stamp = node_->now();

    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = "world";
    path_marker.header.stamp = stamp;
    path_marker.ns = markerNamespace(mapping, "active_path");
    path_marker.id = 0;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.012;
    setMarkerColor(path_marker, 0.15f, 0.45f, 1.0f, 0.9f);
    const int point_count = global_trajectory.activeSegmentPointCount();
    path_marker.points.reserve(static_cast<std::size_t>(std::max(0, point_count)));
    for (int i = 0; i < point_count; ++i) {
        cp::TimedCartesianSample path_sample;
        if (global_trajectory.sample(i, path_sample)) {
            path_marker.points.push_back(toPointMsg(path_sample.T_target.translation()));
        }
    }
    if (path_marker.points.size() < 2u) {
        path_marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    array_msg.markers.push_back(path_marker);

    {
        std::lock_guard<std::mutex> lock(execution_trace_mutex_);
        execution_traces_[mapping].clear();
    }
    visualization_msgs::msg::Marker executed_path_marker;
    executed_path_marker.header.frame_id = "world";
    executed_path_marker.header.stamp = stamp;
    executed_path_marker.ns = markerNamespace(mapping, "executed_path");
    executed_path_marker.id = 0;
    executed_path_marker.action = visualization_msgs::msg::Marker::DELETE;
    array_msg.markers.push_back(executed_path_marker);

    visualization_msgs::msg::Marker target_marker;
    target_marker.header.frame_id = "world";
    target_marker.header.stamp = stamp;
    target_marker.ns = markerNamespace(mapping, "target_point");
    target_marker.id = 0;
    target_marker.type = visualization_msgs::msg::Marker::SPHERE;
    target_marker.action = sample.is_cartesian_tracking_target
                               ? visualization_msgs::msg::Marker::ADD
                               : visualization_msgs::msg::Marker::DELETE;
    target_marker.pose.position = toPointMsg(sample.T_target.translation());
    target_marker.pose.orientation.w = 1.0;
    target_marker.scale.x = 0.035;
    target_marker.scale.y = 0.035;
    target_marker.scale.z = 0.035;
    target_marker.color.r = 1.0f;
    target_marker.color.g = 0.82f;
    target_marker.color.b = 0.10f;
    target_marker.color.a = 0.95f;
    array_msg.markers.push_back(target_marker);

    visualization_msgs::msg::Marker actual_marker;
    actual_marker.header.frame_id = "world";
    actual_marker.header.stamp = stamp;
    actual_marker.ns = markerNamespace(mapping, "actual_point");
    actual_marker.id = 0;
    actual_marker.type = visualization_msgs::msg::Marker::SPHERE;
    actual_marker.action = visualization_msgs::msg::Marker::ADD;
    actual_marker.pose.position = toPointMsg(ee_position);
    actual_marker.pose.orientation.w = 1.0;
    actual_marker.scale.x = 0.03;
    actual_marker.scale.y = 0.03;
    actual_marker.scale.z = 0.03;
    actual_marker.color.r = 0.10f;
    actual_marker.color.g = 1.0f;
    actual_marker.color.b = 0.25f;
    actual_marker.color.a = 0.95f;
    array_msg.markers.push_back(actual_marker);

    trajectory_marker_pub_->publish(array_msg);
}

void ReactiveTaskDiagnosticsPublisher::publishExecutionTrace(
    const std::string& mapping,
    const Eigen::Vector3d& ee_position) {
    if (!trajectory_marker_pub_ || !ee_position.allFinite()) {
        return;
    }

    std::vector<Eigen::Vector3d> trace;
    {
        std::lock_guard<std::mutex> lock(execution_trace_mutex_);
        std::vector<Eigen::Vector3d>& stored_trace = execution_traces_[mapping];
        if (stored_trace.empty() ||
            (stored_trace.back() - ee_position).norm() >= 0.003) {
            stored_trace.push_back(ee_position);
        }
        constexpr std::size_t kMaxTracePoints = 2000u;
        if (stored_trace.size() > kMaxTracePoints) {
            stored_trace.erase(
                stored_trace.begin(),
                stored_trace.begin() +
                    static_cast<std::ptrdiff_t>(stored_trace.size() - kMaxTracePoints));
        }
        trace = stored_trace;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = node_->now();
    marker.ns = markerNamespace(mapping, "executed_path");
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = trace.size() >= 2u
                        ? visualization_msgs::msg::Marker::ADD
                        : visualization_msgs::msg::Marker::DELETE;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.010;
    setMarkerColor(marker, 0.0f, 0.95f, 0.35f, 0.95f);
    marker.points.reserve(trace.size());
    for (const Eigen::Vector3d& p : trace) {
        marker.points.push_back(toPointMsg(p));
    }

    visualization_msgs::msg::MarkerArray array_msg;
    array_msg.markers.push_back(marker);
    trajectory_marker_pub_->publish(array_msg);
}

void ReactiveTaskDiagnosticsPublisher::clearTrajectory(const std::string& mapping) {
    if (!trajectory_marker_pub_) {
        return;
    }

    visualization_msgs::msg::MarkerArray array_msg;
    const std::string namespaces[] = {
        markerNamespace(mapping, "active_path"),
        markerNamespace(mapping, "executed_path"),
        markerNamespace(mapping, "target_point"),
        markerNamespace(mapping, "actual_point"),
    };
    {
        std::lock_guard<std::mutex> lock(execution_trace_mutex_);
        execution_traces_.erase(mapping);
    }
    for (int i = 0; i < 4; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = node_->now();
        marker.ns = namespaces[i];
        marker.id = 0;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        array_msg.markers.push_back(marker);
    }
    trajectory_marker_pub_->publish(array_msg);
}

void ReactiveTaskDiagnosticsPublisher::clearVisuals(const std::string& mapping) {
    clearCollisionEllipsoids(mapping);
    clearTrajectory(mapping);
}

void ReactiveTaskDiagnosticsPublisher::logPhaseTransition(
    const std::string& mapping,
    const ExecutionStatusOutput& phase_decision,
    const double path_progress,
    const double pos_err_goal,
    const double ori_err_goal) const {
    if (!phase_decision.phase_changed) {
        return;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "[%s] phase_transition: phase=%s reason=%s path_progress=%.3f pos_err=%.5f ori_err=%.5f",
        mapping.c_str(),
        toString(phase_decision.phase),
        phase_decision.transition_reason.c_str(),
        path_progress,
        pos_err_goal,
        ori_err_goal);
}

void ReactiveTaskDiagnosticsPublisher::publishRuntimeCycle(
    const RuntimeVisualizationInput& input) {
    RCLCPP_INFO(
        node_->get_logger(),
        "[%s] rt: ref_tick=%d neo_iter=%d phase=%s progress=%.3f "
        "goal(pos=%.4f ori=%.4f) neo(qdot=%.3f residual=%.4f) "
        "safety(gate=%.3f obs=%.4f active=%.4f wb=%.4f) "
        "trajopt(samples=%d dt=%.3f)",
        input.mapping.c_str(),
        input.planner_tick,
        input.neo_iter,
        toString(input.phase),
        input.path_progress,
        input.pos_err_goal,
        input.ori_err_goal,
        input.qdot_norm,
        input.task_residual_norm,
        input.obstacle_guidance_gate,
        input.obstacle_min_distance,
        input.active_safety_distance,
        input.whole_body_status.min_margin,
        input.local_plan_sampled_steps,
        input.local_plan_dt_sec);
    const bool log_detail =
        input.qdot_limit_violation ||
        !input.whole_body_status.collision_free ||
        input.obstacle_guidance_gate >= 0.45 ||
        (std::isfinite(input.whole_body_status.min_margin) &&
         input.whole_body_status.min_margin <= -0.005) ||
        input.task_residual_norm >= 0.08;
    if (log_detail && !input.whole_body_status.worst_link.empty()) {
        const std::string nearest_str =
            input.whole_body_status.nearest_obstacle_point_valid
                ? ("(" +
                   std::to_string(input.whole_body_status.nearest_obstacle_point_world.x()) +
                   ", " +
                   std::to_string(input.whole_body_status.nearest_obstacle_point_world.y()) +
                   ", " +
                   std::to_string(input.whole_body_status.nearest_obstacle_point_world.z()) +
                   ")")
                : "invalid";
        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] safety_detail: worst_link=%s dist=%.5f eff_r=%.5f req=%.5f "
            "safe=%.5f neo(qmax=%.4f dq=%.4f limit=%s) ee=%.4f "
            "joint=%.4f state=%s query=(%.3f, %.3f, %.3f) nearest=%s",
            input.mapping.c_str(),
            input.whole_body_status.worst_link.c_str(),
            input.whole_body_status.worst_distance,
            input.whole_body_status.worst_effective_radius,
            input.whole_body_status.required_clearance,
            input.whole_body_status.safe_distance_used,
            input.qdot_max_abs,
            input.qdot_delta_norm,
            input.qdot_limit_violation ? "true" : "false",
            input.ee_clearance,
            input.joint_limit_margin_min,
            input.whole_body_status.state.c_str(),
            input.whole_body_status.worst_point_world.x(),
            input.whole_body_status.worst_point_world.y(),
            input.whole_body_status.worst_point_world.z(),
            nearest_str.c_str());
    }
}

}  // namespace arm_controller::controller::reactive_task
