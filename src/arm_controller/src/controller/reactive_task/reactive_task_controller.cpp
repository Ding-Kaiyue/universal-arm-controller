#include "controller/reactive_task/reactive_task_controller.hpp"

#include "controller_interface.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

#include "algorithm/cartesian_path_planner/map/camera_driver_esdf_map_client.hpp"
#include "algorithm/cartesian_path_planner/map/camera_driver_pointcloud_map_adapter.hpp"

namespace rq = arm_controller::algorithm::reactive_qp;
namespace cp = arm_controller::algorithm::cartesian_path_planner;

ReactiveTaskController::ReactiveTaskController(const rclcpp::Node::SharedPtr& node)
    : TrajectoryControllerImpl<geometry_msgs::msg::Pose>("ReactiveTask", node),
      diagnostics_publisher_(node) {
    hardware_manager_ = HardwareManager::getInstance();
    reactive_cfg_loaded_ = loadReactiveConfig();
    terminal_policy_ =
        arm_controller::controller::reactive_task::ReactiveTaskTerminalPolicy();
    arm_controller::controller::reactive_task::ReactiveTaskWatchdog::Config
        watchdog_cfg;
    watchdog_cfg.goal_position_tolerance =
        runtime_cfg_.goal_position_tolerance;
    watchdog_cfg.goal_orientation_tolerance_rad =
        runtime_cfg_.goal_orientation_tolerance_rad;
    watchdog_ =
        arm_controller::controller::reactive_task::ReactiveTaskWatchdog(watchdog_cfg);
    local_planner_.configure(runtime_cfg_.local_planner);
    ensureCameraDriverDistanceFieldInitialized();
    collision_ellipsoid_marker_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            if (!hardware_manager_) {
                return;
            }

            const std::vector<std::string> mappings = hardware_manager_->get_all_mappings();
            for (const std::string& mapping : mappings) {
                std::string init_error;
                if (!initializeMappingContext(mapping, &init_error)) {
                    continue;
                }

                MappingContext* ctx = nullptr;
                {
                    std::lock_guard<std::mutex> lock(mapping_contexts_mutex_);
                    auto it = mapping_contexts_.find(mapping);
                    if (it == mapping_contexts_.end() || !it->second.initialized) {
                        continue;
                    }
                    ctx = &it->second;
                }
                if (ctx == nullptr) {
                    continue;
                }

                const std::vector<double> q_current_vec =
                    hardware_manager_->get_current_joint_positions_lockfree(mapping);
                if (q_current_vec.size() != ctx->joint_names.size()) {
                    continue;
                }
                const Eigen::VectorXd q_current = Eigen::Map<const Eigen::VectorXd>(
                    q_current_vec.data(),
                    static_cast<Eigen::Index>(q_current_vec.size()));
                diagnostics_publisher_.publishCollisionEllipsoids(
                    mapping,
                    q_current,
                    *ctx->fk_provider,
                    ctx->collision_ellipsoids);
            }
        });

    planning_worker_running_ = true;
    planning_worker_ = std::make_unique<std::thread>(&ReactiveTaskController::planning_worker_thread, this);

    consumer_running_ = true;
    queue_consumer_ = std::make_unique<std::thread>(&ReactiveTaskController::command_queue_consumer_thread, this);
}

void ReactiveTaskController::ensureCameraDriverDistanceFieldInitialized() {
    if (!reactive_cfg_loaded_) {
        return;
    }

    std::lock_guard<std::mutex> lock(live_distance_field_mutex_);
    if (runtime_cfg_.collision_map_source == "camera_driver_pointcloud" &&
        !camera_driver_pointcloud_map_) {
        camera_driver_pointcloud_map_ =
            std::make_shared<cp::CameraDriverPointcloudMapAdapter>(
                runtime_cfg_.camera_driver_pointcloud,
                node_);
        RCLCPP_INFO(
            node_->get_logger(),
            "[reactive_task] camera_driver pointcloud map ready.");
    }
    if (runtime_cfg_.distance_field_source == "camera_driver_esdf" &&
        !camera_driver_esdf_map_) {
        camera_driver_esdf_map_ =
            std::make_shared<cp::CameraDriverEsdfMapClient>(
                runtime_cfg_.camera_driver_esdf,
                node_);
        RCLCPP_INFO(
            node_->get_logger(),
            "[reactive_task] camera_driver ESDF client ready.");
    }
}

void ReactiveTaskController::start(const std::string& mapping) {
    if (!reactive_cfg_loaded_) {
        throw std::runtime_error(
            "reactive_task config not loaded. Check config/reactive_task_config.yaml");
    }
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "[" + mapping + "] reactive_task: not found in hardware configuration.");
    }

    TrajectoryControllerImpl::start(mapping);

    if (subscriptions_.find(mapping) == subscriptions_.end()) {
        init_subscriptions(mapping);
    }

    ensureCameraDriverDistanceFieldInitialized();

    std::string error;
    if (!initializeMappingContext(mapping, &error)) {
        throw std::runtime_error("reactive_task context init failed for '" + mapping + "': " + error);
    }

    {
        std::lock_guard<std::mutex> lock(mapping_contexts_mutex_);
        auto it = mapping_contexts_.find(mapping);
        if (it != mapping_contexts_.end() && it->second.initialized) {
            const std::vector<double> q_current_vec =
                hardware_manager_->get_current_joint_positions_lockfree(mapping);
            if (q_current_vec.size() == it->second.joint_names.size()) {
                const Eigen::VectorXd q_current = Eigen::Map<const Eigen::VectorXd>(
                    q_current_vec.data(),
                    static_cast<Eigen::Index>(q_current_vec.size()));
                diagnostics_publisher_.publishCollisionEllipsoids(
                    mapping,
                    q_current,
                    *it->second.fk_provider,
                    it->second.collision_ellipsoids);
            }
        }
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] reactive_task activated", mapping.c_str());
}

bool ReactiveTaskController::stop(const std::string& mapping) {
    TrajectoryControllerImpl::stop(mapping);
    cleanup_subscriptions(mapping);
    diagnostics_publisher_.clearVisuals(mapping);
    RCLCPP_INFO(node_->get_logger(), "[%s] reactive_task deactivated", mapping.c_str());
    return true;
}

void ReactiveTaskController::trajectory_callback(
    const std::string& mapping,
    const geometry_msgs::msg::Pose::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(planning_queue_mutex_);
        planning_queue_.push(PlanningTask{mapping, msg});
    }
    planning_queue_cv_.notify_one();
}

void ReactiveTaskController::planning_worker_thread() {
    while (planning_worker_running_) {
        PlanningTask task;
        {
            std::unique_lock<std::mutex> lock(planning_queue_mutex_);
            planning_queue_cv_.wait(lock, [this]() {
                return !planning_queue_.empty() || !planning_worker_running_;
            });
            if (!planning_worker_running_) {
                return;
            }
            task = planning_queue_.front();
            planning_queue_.pop();
        }
        plan_and_execute(task.mapping, task.msg);
    }
}
