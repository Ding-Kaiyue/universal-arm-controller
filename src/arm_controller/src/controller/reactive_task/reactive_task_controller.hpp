#pragma once

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "algorithm/cartesian_path_planner/collision/whole_body_ellipsoid_collision_checker.hpp"
#include "algorithm/cartesian_path_planner/config/planner_common_config.hpp"
#include "algorithm/cartesian_path_planner/config/smoothing_config.hpp"
#include "algorithm/cartesian_path_planner/map/camera_driver_esdf_map_client.hpp"
#include "algorithm/cartesian_path_planner/map/camera_driver_pointcloud_map_adapter.hpp"
#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"
#include "algorithm/cartesian_path_planner/global_trajectory/global_trajectory_manager.hpp"
#include "algorithm/global_planner/global_planner_interface.hpp"
#include "algorithm/neo/body_obstacle_constraint_builder.hpp"
#include "algorithm/neo/joint_preference_loader.hpp"
#include "algorithm/neo/manipulability_gradient.hpp"
#include "algorithm/neo/reactive_qp_builder.hpp"
#include "algorithm/neo/reactive_qp_solver.hpp"
#include "algorithm/neo/task_velocity_generator.hpp"
#include "arm_controller/kinematics/forward_kinematics.hpp"
#include "arm_controller/kinematics/jacobian_provider.hpp"
#include "controller/reactive_task/reactive_task_diagnostics_publisher.hpp"
#include "controller/reactive_task/reactive_task_environment_probe.hpp"
#include "controller/reactive_task/reactive_task_execution_context.hpp"
#include "controller/reactive_task/reactive_task_execution_state_machine.hpp"
#include "controller/reactive_task/reactive_task_local_reference_manager.hpp"
#include "controller/reactive_task/reactive_task_local_planner.hpp"
#include "controller/reactive_task/reactive_task_neo_pipeline.hpp"
#include "controller/reactive_task/reactive_task_safety_policy.hpp"
#include "controller/reactive_task/reactive_task_terminal_policy.hpp"
#include "controller/reactive_task/reactive_task_watchdog.hpp"
#include "controller_base/trajectory_controller_base.hpp"
#include "hardware/hardware_manager.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "trajectory_planning_v3/infrastructure/integration/tracik_adapter.hpp"
#include <Eigen/Core>
#include <geometry_msgs/msg/pose.hpp>

class ReactiveTaskController final
    : public TrajectoryControllerImpl<geometry_msgs::msg::Pose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ReactiveTaskController(const rclcpp::Node::SharedPtr &node);
  ~ReactiveTaskController() override;

  void start(const std::string &mapping = "") override;
  bool stop(const std::string &mapping = "") override;

  bool execute(const std::string &mapping,
               const std::vector<double> &parameters) override;

  struct ControllerRuntimeConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    arm_controller::controller::reactive_task::ReactiveTaskLocalPlanner::
        Config local_planner;
    arm_controller::algorithm::cartesian_path_planner::GlobalTrajectoryConfig
        global_trajectory;
    arm_controller::algorithm::cartesian_path_planner::PlannerCommonConfig
        planner_common;
    arm_controller::algorithm::cartesian_path_planner::SmoothingConfig
        planner_smoothing;
    double request_safe_distance{0.03};
    double request_planning_safe_distance{0.06};
    double request_hard_clearance{0.0};
    double request_goal_tolerance{0.03};
    Eigen::Vector3d map_margin_xyz{0.60, 0.60, 0.60};
    std::string distance_field_source{"camera_driver_esdf"};
    std::string collision_map_source{"camera_driver_pointcloud"};
    arm_controller::algorithm::cartesian_path_planner::
        CameraDriverPointcloudMapAdapter::Config camera_driver_pointcloud;
    arm_controller::algorithm::cartesian_path_planner::
        CameraDriverEsdfMapClient::Config camera_driver_esdf;
    double whole_body_segment_check_step_m{0.10};
    bool enable_dummy_obstacle{false};
    double dummy_obstacle_radius{0.035};
    Eigen::Vector3d dummy_obstacle_center_left_arm{0.25, -0.52, 0.60};
    Eigen::Vector3d dummy_obstacle_center_right_arm{0.07, 0.52, 0.72};
    int max_control_ticks{1500};
    double neo_control_cycle_sec{0.004};
    double goal_position_tolerance{0.01};
    double goal_orientation_tolerance_rad{0.08};
    double mit_kp{0.0};
    double mit_kd{0.01};
    int mit_max_motors{6};
  };

private:
  struct PlanningTask {
    std::string mapping;
    geometry_msgs::msg::Pose::SharedPtr msg;
  };

  struct MappingContext {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool initialized{false};
    std::string robot_type;
    std::string planning_group;
    std::string base_link;
    std::string tip_link;
    std::string urdf_path;
    std::string srdf_path;
    std::vector<std::string> joint_names;
    arm_controller::algorithm::reactive_qp::HumanLikeJointPreferenceConfig
        joint_preference_cfg;
    std::shared_ptr<
        trajectory_planning::infrastructure::integration::MoveItAdapter>
        moveit_adapter;
    std::shared_ptr<
        trajectory_planning::infrastructure::integration::TracIKAdapter>
        tracik_adapter;
    bool tracik_ready{false};

    Eigen::VectorXd qd_min;
    Eigen::VectorXd qd_max;
    arm_controller::algorithm::reactive_qp::JointLimitData joint_limits;

    std::shared_ptr<arm_controller::kinematics::PinocchioForwardKinematics>
        fk_provider;
    std::shared_ptr<arm_controller::kinematics::JacobianProvider>
        jacobian_provider;
    std::unique_ptr<
        arm_controller::algorithm::reactive_qp::ManipulabilityGradient>
        manipulability_gradient;
    arm_controller::algorithm::reactive_qp::LinkCollisionEllipsoidList
        collision_ellipsoids;
  };

  struct PlanningSession {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MappingContext *ctx{nullptr};
    std::vector<double> q_current_vec;
    Eigen::VectorXd q_start;
    Eigen::Vector3d fk_start_position{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d fk_start_rotation{Eigen::Matrix3d::Identity()};
    arm_controller::algorithm::cartesian_path_planner::PathPlanningInput
        request;
    double control_safe_distance{0.0};
    std::shared_ptr<const arm_controller::algorithm::cartesian_path_planner::
                        DistanceFieldInterface>
        map;
    std::shared_ptr<const arm_controller::algorithm::cartesian_path_planner::
                        DistanceFieldInterface>
        collision_map;
    std::shared_ptr<
        arm_controller::algorithm::global_planner::GlobalPlannerInterface>
        planner;
    std::shared_ptr<arm_controller::algorithm::cartesian_path_planner::
                        WholeBodyEllipsoidCollisionChecker>
        whole_body_validator;
    std::shared_ptr<arm_controller::algorithm::cartesian_path_planner::
                        WholeBodyEllipsoidCollisionChecker>
        planning_whole_body_validator;
    bool enable_obstacle_constraints{false};
  };

  struct PlanningRuntime {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::unique_ptr<
        arm_controller::algorithm::cartesian_path_planner::GlobalTrajectoryManager>
        global_trajectory;
    std::unique_ptr<arm_controller::algorithm::reactive_qp::ReactiveQpSolver>
        solver;
    arm_controller::algorithm::reactive_qp::TaskVelocityGenerator
        task_velocity_generator;
    arm_controller::algorithm::cartesian_path_planner::TimedCartesianSample
        sample;
    arm_controller::controller::reactive_task::ReactiveTaskExecutionContext
        exec_ctx;
    double planner_tick_sec{1e-4};
    double neo_tick_sec{1e-4};
    double feedback_stale_threshold_sec{0.2};
    double active_duration_sec{0.0};
    int configured_max_planner_ticks{1};
    int no_motion_cycle_limit{20};
    int no_progress_cycle_limit{50};
    bool sample_ok{true};
    bool reached_goal{false};
  };

  using MappingContextMap =
      std::map<std::string, MappingContext, std::less<std::string>,
               Eigen::aligned_allocator<std::pair<const std::string, MappingContext>>>;

  void
  trajectory_callback(const std::string &mapping,
                      const geometry_msgs::msg::Pose::SharedPtr msg) override;
  void plan_and_execute(const std::string &mapping,
                        const geometry_msgs::msg::Pose::SharedPtr msg) override;
  void command_queue_consumer_thread() override;
  void planning_worker_thread();

  bool loadReactiveConfig();
  bool initializeMappingContext(const std::string &mapping, std::string *error);
  void ensureCameraDriverDistanceFieldInitialized();

  std::shared_ptr<
      arm_controller::algorithm::global_planner::GlobalPlannerInterface>
  buildPlanner() const;
  bool preparePlanningSession(const std::string &mapping,
                              const geometry_msgs::msg::Pose::SharedPtr &msg,
                              PlanningSession *session);
  bool initializePlanningRuntime(const std::string &mapping,
                                 const PlanningSession &session,
                                 PlanningRuntime *runtime);
  bool runPlanningControlLoop(const std::string &mapping,
                              const PlanningSession &session,
                              PlanningRuntime *runtime);

  bool send_joint_velocities(const std::string &mapping,
                             const std::vector<double> &joint_velocities) const;

private:
  std::shared_ptr<HardwareManager> hardware_manager_;

  MappingContextMap mapping_contexts_;
  std::mutex mapping_contexts_mutex_;

  arm_controller::algorithm::reactive_qp::ReactiveQpExampleConfig reactive_cfg_;
  bool reactive_cfg_loaded_{false};
  ControllerRuntimeConfig runtime_cfg_;

  std::mutex live_distance_field_mutex_;
  std::shared_ptr<arm_controller::algorithm::cartesian_path_planner::
                      CameraDriverPointcloudMapAdapter>
      camera_driver_pointcloud_map_;
  std::shared_ptr<arm_controller::algorithm::cartesian_path_planner::
                      CameraDriverEsdfMapClient>
      camera_driver_esdf_map_;
  arm_controller::controller::reactive_task::ReactiveTaskExecutionStateMachine
      execution_state_machine_;
  arm_controller::controller::reactive_task::ReactiveTaskLocalReferenceManager
      local_reference_manager_;
  arm_controller::controller::reactive_task::ReactiveTaskTerminalPolicy
      terminal_policy_;
  arm_controller::controller::reactive_task::ReactiveTaskSafetyPolicy
      safety_policy_;
  arm_controller::controller::reactive_task::ReactiveTaskWatchdog watchdog_;
  arm_controller::controller::reactive_task::ReactiveTaskDiagnosticsPublisher
      diagnostics_publisher_;
  arm_controller::controller::reactive_task::ReactiveTaskEnvironmentProbe
      environment_probe_;
  arm_controller::controller::reactive_task::ReactiveTaskNeoPipeline
      neo_pipeline_;
  arm_controller::controller::reactive_task::ReactiveTaskLocalPlanner
      local_planner_;

  std::map<std::string, bool> last_execution_success_;

  std::unique_ptr<std::thread> queue_consumer_;
  std::atomic<bool> consumer_running_{false};

  std::unique_ptr<std::thread> planning_worker_;
  std::atomic<bool> planning_worker_running_{false};
  std::queue<PlanningTask> planning_queue_;
  std::mutex planning_queue_mutex_;
  std::condition_variable planning_queue_cv_;
  rclcpp::TimerBase::SharedPtr collision_ellipsoid_marker_timer_;
};
