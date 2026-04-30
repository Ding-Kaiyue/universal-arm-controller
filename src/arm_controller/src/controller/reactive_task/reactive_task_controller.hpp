#pragma once

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "algorithm/neo/joint_preference_loader.hpp"
#include "algorithm/neo/manipulability_gradient.hpp"
#include "algorithm/neo/body_obstacle_constraint_builder.hpp"
#include "algorithm/neo/reactive_qp_builder.hpp"
#include "algorithm/neo/reactive_qp_solver.hpp"
#include "algorithm/neo/task_velocity_generator.hpp"
#include "algorithm/cartesian_path_planner/core/cartesian_path_planner.hpp"
#include "algorithm/cartesian_path_planner/map/camera_driver_esdf_map_client.hpp"
#include "algorithm/cartesian_path_planner/map/camera_driver_pointcloud_map_adapter.hpp"
#include "algorithm/cartesian_path_planner/replanning/replanner_manager.hpp"
#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"
#include "arm_controller/kinematics/forward_kinematics.hpp"
#include "arm_controller/kinematics/jacobian_provider.hpp"
#include "controller_base/trajectory_controller_base.hpp"
#include "hardware/hardware_manager.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "trajectory_planning_v3/infrastructure/integration/tracik_adapter.hpp"

class ReactiveTaskController final : public TrajectoryControllerImpl<geometry_msgs::msg::Pose> {
public:
    explicit ReactiveTaskController(const rclcpp::Node::SharedPtr& node);
    ~ReactiveTaskController() override;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;

    bool execute(const std::string& mapping, const std::vector<double>& parameters) override;

    struct ControllerRuntimeConfig {
        arm_controller::algorithm::cartesian_path_planner::ReplannerConfig replanner;
        arm_controller::algorithm::cartesian_path_planner::PlannerCommonConfig planner_common;
        arm_controller::algorithm::cartesian_path_planner::AStarConfig planner_astar;
        arm_controller::algorithm::cartesian_path_planner::SmoothingConfig planner_smoothing;
        double request_safe_distance{0.03};
        double request_hard_clearance{0.0};
        double request_goal_tolerance{0.03};
        Eigen::Vector3d map_margin_xyz{0.60, 0.60, 0.60};
        std::string map_source{"dummy"};
        arm_controller::algorithm::cartesian_path_planner::CameraDriverPointcloudMapAdapter::Config
            camera_driver_pointcloud;
        arm_controller::algorithm::cartesian_path_planner::CameraDriverEsdfMapClient::Config
            camera_driver_esdf;
        bool whole_body_postcheck_non_blocking{false};
        int whole_body_postcheck_max_attempts{5};
        double whole_body_retry_forbidden_radius{0.045};
        double whole_body_retry_pushout_distance{0.02};
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
        bool initialized{false};
        std::string robot_type;
        std::vector<std::string> joint_names;
        arm_controller::algorithm::reactive_qp::HumanLikeJointPreferenceConfig joint_preference_cfg;
        std::shared_ptr<trajectory_planning::infrastructure::integration::MoveItAdapter> moveit_adapter;
        std::shared_ptr<trajectory_planning::infrastructure::integration::TracIKAdapter> tracik_adapter;
        bool tracik_ready{false};

        Eigen::VectorXd qd_min;
        Eigen::VectorXd qd_max;
        arm_controller::algorithm::reactive_qp::JointLimitData joint_limits;

        std::shared_ptr<arm_controller::kinematics::PinocchioForwardKinematics> fk_provider;
        std::shared_ptr<arm_controller::kinematics::JacobianProvider> jacobian_provider;
        std::unique_ptr<arm_controller::algorithm::reactive_qp::ManipulabilityGradient> manipulability_gradient;
        std::vector<arm_controller::algorithm::reactive_qp::LinkCollisionEllipsoid> collision_ellipsoids;
    };

    void trajectory_callback(const std::string& mapping, const geometry_msgs::msg::Pose::SharedPtr msg) override;
    void plan_and_execute(const std::string& mapping, const geometry_msgs::msg::Pose::SharedPtr msg) override;
    void command_queue_consumer_thread() override;
    void planning_worker_thread();

    bool loadReactiveConfig();
    bool initializeMappingContext(const std::string& mapping, std::string* error);
    void ensureCameraDriverDistanceFieldInitialized();

    std::shared_ptr<arm_controller::algorithm::cartesian_path_planner::CartesianPathPlanner>
    buildPlanner(
        const std::shared_ptr<const arm_controller::algorithm::cartesian_path_planner::DistanceFieldInterface>& map,
        const Eigen::Vector3d& map_min) const;

    bool send_joint_velocities(const std::string& mapping, const std::vector<double>& joint_velocities) const;
    void publishWholeBodyPostcheckFailureMarker(
        const std::string& mapping,
        const arm_controller::algorithm::cartesian_path_planner::PathPlanningInput::WholeBodyPostcheckFailureEvent& event,
        const std::shared_ptr<const arm_controller::algorithm::cartesian_path_planner::DistanceFieldInterface>& map,
        const MappingContext& ctx);
    void clearWholeBodyPostcheckFailureMarker(const std::string& mapping);
    void publishCollisionEllipsoidMarkers(
        const std::string& mapping,
        const Eigen::VectorXd& q_current,
        const MappingContext& ctx);
    void clearCollisionEllipsoidMarkers(const std::string& mapping);

private:
    std::shared_ptr<HardwareManager> hardware_manager_;

    std::map<std::string, MappingContext> mapping_contexts_;
    std::mutex mapping_contexts_mutex_;

    arm_controller::algorithm::reactive_qp::ReactiveQpExampleConfig reactive_cfg_;
    bool reactive_cfg_loaded_{false};
    ControllerRuntimeConfig runtime_cfg_;

    std::mutex live_distance_field_mutex_;
    std::shared_ptr<arm_controller::algorithm::cartesian_path_planner::CameraDriverPointcloudMapAdapter>
        camera_driver_pointcloud_map_;
    std::shared_ptr<arm_controller::algorithm::cartesian_path_planner::CameraDriverEsdfMapClient>
        camera_driver_esdf_map_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        whole_body_postcheck_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        collision_ellipsoid_marker_pub_;
    std::mutex collision_ellipsoid_marker_mutex_;
    std::map<std::string, std::size_t> collision_ellipsoid_marker_counts_;

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
