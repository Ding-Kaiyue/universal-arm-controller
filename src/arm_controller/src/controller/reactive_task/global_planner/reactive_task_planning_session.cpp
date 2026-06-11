#include "controller/reactive_task/reactive_task_controller.hpp"

#include "controller/reactive_task/global_planner/reactive_task_planning_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>

#include "algorithm/cartesian_path_planner/map/dummy_distance_field.hpp"

namespace cp = arm_controller::algorithm::cartesian_path_planner;

namespace {

Eigen::Isometry3d poseMsgToIso(const geometry_msgs::msg::Pose &pose) {
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                       pose.orientation.y, pose.orientation.z);
  if (q.norm() < 1e-8) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }

  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() =
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  t.linear() = q.toRotationMatrix();
  return t;
}

geometry_msgs::msg::Pose toPoseMsg(const Eigen::Vector3d &p,
                                   const Eigen::Matrix3d &R) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = p.x();
  pose.position.y = p.y();
  pose.position.z = p.z();
  const Eigen::Quaterniond q(R);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

std::shared_ptr<cp::DistanceFieldInterface>
makeDummyMap(const ReactiveTaskController::ControllerRuntimeConfig &cfg,
             const std::string &mapping, const Eigen::Vector3d &map_min,
             const Eigen::Vector3d &map_max) {
  auto dummy_map = std::make_shared<cp::DummyDistanceField>(map_min, map_max);
  if (cfg.enable_dummy_obstacle && cfg.dummy_obstacle_radius > 0.0) {
    cp::SphereObstacle obstacle;
    obstacle.center = (mapping == "right_arm")
                          ? cfg.dummy_obstacle_center_right_arm
                          : cfg.dummy_obstacle_center_left_arm;
    obstacle.radius = cfg.dummy_obstacle_radius;
    dummy_map->addSphere(obstacle);
  }
  return dummy_map;
}

std::string formatVector(const Eigen::VectorXd &v) {
  std::ostringstream oss;
  oss << v.transpose().format(Eigen::IOFormat(4, 0, ", ", ", ", "[", "]"));
  return oss.str();
}

} // namespace

bool ReactiveTaskController::preparePlanningSession(
    const std::string &mapping, const geometry_msgs::msg::Pose::SharedPtr &msg,
    PlanningSession *session) {
  if (session == nullptr || !msg) {
    return false;
  }

  std::string init_error;
  if (!initializeMappingContext(mapping, &init_error)) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] planning_setup init failed: %s",
                 mapping.c_str(), init_error.c_str());
    last_execution_success_[mapping] = false;
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mapping_contexts_mutex_);
    session->ctx = &mapping_contexts_[mapping];
  }
  if (session->ctx == nullptr || !session->ctx->initialized) {
    last_execution_success_[mapping] = false;
    return false;
  }

  session->q_current_vec =
      hardware_manager_->get_current_joint_positions_lockfree(mapping);
  if (session->q_current_vec.size() != session->ctx->joint_names.size()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[%s] planning_setup current joint size mismatch",
                 mapping.c_str());
    last_execution_success_[mapping] = false;
    return false;
  }

  session->q_start = Eigen::Map<const Eigen::VectorXd>(
      session->q_current_vec.data(),
      static_cast<Eigen::Index>(session->q_current_vec.size()));

  static const std::vector<std::string> kNoLinkPoseQueries;
  arm_controller::kinematics::LinkPoseResultList start_link_poses;
  if (!session->ctx->fk_provider->computeLinkPoses(
          session->q_start,
          kNoLinkPoseQueries,
          start_link_poses,
          &session->fk_start_position,
          &session->fk_start_rotation)) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] planning_setup FK failed at start",
                 mapping.c_str());
    last_execution_success_[mapping] = false;
    return false;
  }

  const Eigen::Isometry3d goal = poseMsgToIso(*msg);
  session->request.p_start = session->fk_start_position;
  session->request.R_start = session->fk_start_rotation;
  session->request.p_goal = goal.translation();
  session->request.R_goal = goal.linear();
  session->request.bypass_axis_hint = Eigen::Vector3d::UnitZ();
  session->request.q_start_seed = session->q_start;
  session->request.q_min = session->ctx->joint_limits.q_min;
  session->request.q_max = session->ctx->joint_limits.q_max;
  session->control_safe_distance = runtime_cfg_.request_safe_distance;
  session->request.safe_distance =
      std::max(runtime_cfg_.request_safe_distance,
               runtime_cfg_.request_planning_safe_distance);
  session->request.hard_clearance = runtime_cfg_.request_hard_clearance;
  session->request.goal_tolerance = runtime_cfg_.request_goal_tolerance;

  const Eigen::Vector3d min_corner =
      session->request.p_start.cwiseMin(session->request.p_goal);
  const Eigen::Vector3d max_corner =
      session->request.p_start.cwiseMax(session->request.p_goal);
  const Eigen::Vector3d map_margin = runtime_cfg_.map_margin_xyz;
  const Eigen::Vector3d map_min = min_corner - map_margin;
  const Eigen::Vector3d map_max = max_corner + map_margin;

  std::shared_ptr<cp::DistanceFieldInterface> distance_field_map;
  std::shared_ptr<cp::DistanceFieldInterface> collision_map;
  ensureCameraDriverDistanceFieldInitialized();
  {
    std::lock_guard<std::mutex> lock(live_distance_field_mutex_);
    if (runtime_cfg_.distance_field_source == "camera_driver_esdf") {
      distance_field_map = std::static_pointer_cast<cp::DistanceFieldInterface>(
          camera_driver_esdf_map_);
    } else if (runtime_cfg_.distance_field_source == "camera_driver_pointcloud") {
      distance_field_map = std::static_pointer_cast<cp::DistanceFieldInterface>(
          camera_driver_pointcloud_map_);
    }
    if (runtime_cfg_.distance_field_source == "camera_driver_esdf") {
      collision_map = std::static_pointer_cast<cp::DistanceFieldInterface>(
          camera_driver_esdf_map_);
    } else if (runtime_cfg_.collision_map_source == "camera_driver_esdf") {
      collision_map = std::static_pointer_cast<cp::DistanceFieldInterface>(
          camera_driver_esdf_map_);
    }
    if (runtime_cfg_.collision_map_source == "camera_driver_pointcloud") {
      collision_map = std::static_pointer_cast<cp::DistanceFieldInterface>(
          camera_driver_pointcloud_map_);
    }
  }
  if (!distance_field_map) {
    distance_field_map = makeDummyMap(runtime_cfg_, mapping, map_min, map_max);
  }
  if (!collision_map) {
    collision_map = makeDummyMap(runtime_cfg_, mapping, map_min, map_max);
  }
  session->map = distance_field_map;
  session->collision_map = collision_map;

  session->planner = buildPlanner();
  session->enable_obstacle_constraints =
      reactive_cfg_.qp_build.enable_obstacle_damper &&
      (runtime_cfg_.distance_field_source == "camera_driver_esdf" ||
       runtime_cfg_.distance_field_source == "camera_driver_pointcloud" ||
       (runtime_cfg_.distance_field_source == "dummy" &&
        runtime_cfg_.enable_dummy_obstacle));

  if (session->ctx->tracik_ready && session->ctx->tracik_adapter &&
      session->ctx->moveit_adapter &&
      !session->ctx->collision_ellipsoids.empty()) {
    cp::WholeBodyEllipsoidCollisionChecker::Config wb_cfg;
    wb_cfg.ik_max_iterations = 25;
    wb_cfg.ik_pos_tolerance_m = 0.01;
    wb_cfg.ik_rot_tolerance_rad = 0.15;
    wb_cfg.ik_damping = 0.05;
    wb_cfg.ik_step_scale = 0.6;
    wb_cfg.segment_substeps_min = 1;
    wb_cfg.segment_check_step_m = runtime_cfg_.whole_body_segment_check_step_m;
    // The validator margin is already measured relative to safe_distance:
    //   margin = distance - (safe_distance + effective_body_radius).
    // Treat margin < 0 as blocking so the planner does not hand the QP a
    // path that already violates the safety shell and then rely on CBF to
    // recover from it.
    wb_cfg.collision_blocking_margin_m = 0.0;
    wb_cfg.default_q_seed = session->q_start;
    wb_cfg.ik_solver_fn = [moveit = session->ctx->moveit_adapter,
                           tracik = session->ctx->tracik_adapter,
                           seed_default = session->q_current_vec](
                              const Eigen::Vector3d &p_target,
                              const Eigen::Matrix3d &R_target,
                              const std::optional<Eigen::VectorXd> &q_seed,
                              Eigen::VectorXd &q_solution) -> bool {
      if (!moveit || !tracik) {
        return false;
      }

      std::vector<double> seed = seed_default;
      if (q_seed.has_value() && q_seed->size() > 0) {
        seed.assign(q_seed->data(), q_seed->data() + q_seed->size());
      }
      if (seed.empty()) {
        return false;
      }

      const geometry_msgs::msg::Pose pose_world = toPoseMsg(p_target, R_target);
      const geometry_msgs::msg::Pose pose_base =
          moveit->worldPoseToBaseLinkPose(pose_world);

      std::vector<double> q_solution_vec;
      if (!tracik->computeIKClosest(pose_base, seed, q_solution_vec, 8,
                                    false) ||
          q_solution_vec.empty()) {
        return false;
      }
      q_solution = Eigen::Map<const Eigen::VectorXd>(
          q_solution_vec.data(),
          static_cast<Eigen::Index>(q_solution_vec.size()));
      return true;
    };

    session->whole_body_validator =
        std::make_shared<cp::WholeBodyEllipsoidCollisionChecker>(
            wb_cfg, session->collision_map, session->ctx->fk_provider,
            session->ctx->jacobian_provider, session->ctx->collision_ellipsoids);
    session->planning_whole_body_validator =
        std::make_shared<cp::WholeBodyEllipsoidCollisionChecker>(
            wb_cfg, session->collision_map, session->ctx->fk_provider,
            session->ctx->jacobian_provider, session->ctx->collision_ellipsoids);
    auto control_validator = session->whole_body_validator;
    auto planning_validator = session->planning_whole_body_validator;
    const auto pose_validator = planning_validator->makePoseValidatorFn();
    session->request.whole_body_pose_validator =
        [planning_validator, pose_validator](
            const Eigen::Vector3d &p, const Eigen::Matrix3d &R,
            const double safe_distance,
            const std::optional<Eigen::VectorXd> &q_seed,
            Eigen::VectorXd &q_solution) {
          return pose_validator(p, R, safe_distance, q_seed, q_solution);
        };
    const auto segment_validator = planning_validator->makeSegmentValidatorFn();
    session->request.whole_body_segment_validator =
        [planning_validator, segment_validator](
            const cp::CartesianWaypoint &from, const cp::CartesianWaypoint &to,
            const double safe_distance,
            const std::optional<Eigen::VectorXd> &q_seed,
            Eigen::VectorXd &q_end,
            cp::PathPlanningInput::WholeBodyPoseDiagnostic *failed_diag) {
          return segment_validator(from, to, safe_distance, q_seed, q_end,
                                   failed_diag);
        };
    const auto pose_diagnostic = planning_validator->makePoseDiagnosticFn();
    session->request.whole_body_pose_diagnostic =
        [planning_validator, pose_diagnostic](
            const Eigen::Vector3d &p, const Eigen::Matrix3d &R,
            const double safe_distance,
            const std::optional<Eigen::VectorXd> &q_seed) {
          return pose_diagnostic(p, R, safe_distance, q_seed);
        };
    const auto joint_state_validator =
        planning_validator->makeJointStateValidatorFn();
    session->request.joint_state_validator =
        [planning_validator, joint_state_validator](
            const Eigen::VectorXd &q, const double safe_distance,
            cp::PathPlanningInput::WholeBodyPoseDiagnostic *diag) {
          return joint_state_validator(q, safe_distance, diag);
        };
    const auto joint_segment_validator =
        planning_validator->makeJointSegmentValidatorFn();
    session->request.joint_segment_validator =
        [planning_validator, joint_segment_validator](
            const Eigen::VectorXd &q_from, const Eigen::VectorXd &q_to,
            const double safe_distance,
            cp::PathPlanningInput::WholeBodyPoseDiagnostic *diag) {
          return joint_segment_validator(q_from, q_to, safe_distance, diag);
        };
    session->request.joint_to_pose_fn =
        [fk = session->ctx->fk_provider](const Eigen::VectorXd &q,
                                         cp::CartesianWaypoint &wp) -> bool {
      if (!fk) {
        return false;
      }
      static const std::vector<std::string> kNoLinkPoseQueries;
      arm_controller::kinematics::LinkPoseResultList link_poses;
      Eigen::Vector3d ee_position = Eigen::Vector3d::Zero();
      Eigen::Matrix3d ee_rotation = Eigen::Matrix3d::Identity();
      if (!fk->computeLinkPoses(
              q,
              kNoLinkPoseQueries,
              link_poses,
              &ee_position,
              &ee_rotation)) {
        return false;
      }
      wp.position = ee_position;
      wp.orientation = ee_rotation;
      return true;
    };
    const auto start_plan_diag = planning_validator->diagnoseJointState(
        session->q_start, session->request.safe_distance);
    const auto start_control_diag = control_validator->diagnoseJointState(
        session->q_start, session->control_safe_distance);
    const auto start_hard_diag = planning_validator->diagnoseJointState(
        session->q_start, std::max(0.0, session->request.hard_clearance));

    const geometry_msgs::msg::Pose pose_goal_world =
        toPoseMsg(session->request.p_goal, session->request.R_goal);
    const geometry_msgs::msg::Pose pose_goal_base =
        session->ctx->moveit_adapter->worldPoseToBaseLinkPose(pose_goal_world);
    std::vector<double> seed_goal = session->q_current_vec;
    std::vector<double> q_goal_vec;
    double best_planning_margin = -std::numeric_limits<double>::infinity();
    double best_control_margin = -std::numeric_limits<double>::infinity();
    double best_relaxed_margin = -std::numeric_limits<double>::infinity();
    for (int attempt = 0; attempt < 8; ++attempt) {
      if (!session->ctx->tracik_adapter->computeIKClosest(
              pose_goal_base, seed_goal, q_goal_vec, 1, false) ||
          q_goal_vec.size() != session->q_current_vec.size()) {
        continue;
      }
      const Eigen::VectorXd q_goal = Eigen::Map<const Eigen::VectorXd>(
          q_goal_vec.data(), static_cast<Eigen::Index>(q_goal_vec.size()));
      const auto q_goal_diag = planning_validator->diagnoseJointState(
          q_goal, session->request.safe_distance);
      const auto q_goal_diag_control = control_validator->diagnoseJointState(
          q_goal, session->control_safe_distance);
      const auto q_goal_diag_relaxed = planning_validator->diagnoseJointState(
          q_goal, std::max(0.0, session->request.hard_clearance));
      best_planning_margin =
          std::max(best_planning_margin, q_goal_diag.min_margin);
      best_control_margin =
          std::max(best_control_margin, q_goal_diag_control.min_margin);
      best_relaxed_margin =
          std::max(best_relaxed_margin, q_goal_diag_relaxed.min_margin);

      bool duplicate = false;
      for (const auto &existing : session->request.q_goal_candidates) {
        if (existing.size() == q_goal.size() &&
            (existing - q_goal).cwiseAbs().maxCoeff() < 1e-3) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        session->request.q_goal_candidates.push_back(q_goal);
        if (session->request.q_goal_candidates.size() <= 3u) {
          RCLCPP_INFO(node_->get_logger(),
                      "[%s] planning_setup q_goal_candidate[%zu]=%s",
                      mapping.c_str(),
                      session->request.q_goal_candidates.size() - 1u,
                      formatVector(q_goal).c_str());
        }
      }
      seed_goal = q_goal_vec;
      if (!seed_goal.empty()) {
        const std::size_t idx =
            static_cast<std::size_t>(attempt % seed_goal.size());
        seed_goal[idx] += ((attempt % 2) == 0 ? 0.15 : -0.15);
      }
    }
    if (!session->request.q_goal_candidates.empty()) {
      const bool start_hard_clearance_usable =
          std::isfinite(start_hard_diag.min_margin) &&
          start_hard_diag.min_margin >= 0.0;
      const bool goal_hard_clearance_usable =
          std::isfinite(best_relaxed_margin) && best_relaxed_margin >= 0.0;
      if (session->request.safe_distance >
              session->control_safe_distance + 1e-9 &&
          start_hard_clearance_usable && goal_hard_clearance_usable &&
          (start_plan_diag.min_margin < 0.0 || best_planning_margin < 0.0)) {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] planner feasibility shell set to control safe %.3f "
                    "while keeping planning preference %.3f because start/goal "
                    "do not fit the planning shell but satisfy hard clearance: "
                    "start_margin(plan/control/hard)=%.5f/%.5f/%.5f "
                    "goal_best(plan/control/hard)=%.5f/%.5f/%.5f",
                    mapping.c_str(), session->control_safe_distance,
                    session->request.safe_distance, start_plan_diag.min_margin,
                    start_control_diag.min_margin, start_hard_diag.min_margin,
                    best_planning_margin, best_control_margin,
                    best_relaxed_margin);
        session->request.feasibility_safe_distance =
            session->control_safe_distance;
      }
      RCLCPP_INFO(
          node_->get_logger(),
          "[%s] planning_setup: goal_candidates=%zu margin(plan/control/hard)=%.5f/%.5f/%.5f",
          mapping.c_str(), session->request.q_goal_candidates.size(),
          best_planning_margin, best_control_margin, best_relaxed_margin);
    }
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "[%s] WholeBody validator disabled: tracik_ready=%s moveit=%s "
                "tracik=%s ellipsoids=%zu",
                mapping.c_str(), session->ctx->tracik_ready ? "true" : "false",
                session->ctx->moveit_adapter ? "true" : "false",
                session->ctx->tracik_adapter ? "true" : "false",
                session->ctx->collision_ellipsoids.size());
  }

  if (session->request.q_goal_candidates.empty()) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "[%s] planning_setup failed: no valid goal IK candidates.",
        mapping.c_str());
    last_execution_success_[mapping] = false;
    return false;
  }
  return true;
}
