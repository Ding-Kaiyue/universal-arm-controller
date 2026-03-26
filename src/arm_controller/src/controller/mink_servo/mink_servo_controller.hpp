#pragma once

#include "controller_base/velocity_controller_base.hpp"
#include "hardware/hardware_manager.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "arm_controller/utils/velocity_strict_solver.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <unordered_map>
#include <thread>
#include <atomic>
#include <mutex>

class MinkServoController final
    : public VelocityControllerImpl<geometry_msgs::msg::PoseStamped> {
public:
    explicit MinkServoController(const rclcpp::Node::SharedPtr& node);
    ~MinkServoController() override;

    void start(const std::string& mapping) override;
    bool stop(const std::string& mapping) override;
    bool send_velocity(const std::string& mapping, const std::vector<double>& target_pose) override;

private:
    void velocity_callback(const std::string& mapping,
                           const geometry_msgs::msg::PoseStamped::SharedPtr msg) override;
    void command_queue_consumer_thread() override;
    void control_loop_rt(const std::string& mapping);
    void mink_computation_thread(const std::string& mapping);
    void initialize_moveit_adapter(const std::string& mapping);
    void initialize_pinocchio_context(const std::string& mapping);
    bool send_joint_status_command(const std::string& mapping,
                                const std::vector<double>& q_current,
                                const std::vector<double>& qd_cmd);

private:
    struct PoseCommand {
        geometry_msgs::msg::PoseStamped pose;
        std::chrono::steady_clock::time_point stamp;
    };

    struct RtState {
        geometry_msgs::msg::PoseStamped target;
        std::chrono::steady_clock::time_point last_update;
        bool first_command_received{false};
    };

    struct PinocchioContext {
        bool ready{false};
        pinocchio::Model model;
        std::unique_ptr<pinocchio::Data> data;
        std::vector<int> q_indices;
        std::vector<int> v_indices;
        pinocchio::FrameIndex ee_frame{0};
    };

    struct ComputationResult {
        Eigen::VectorXd qd;
        Eigen::VectorXd qd_last;
        bool valid{false};
        std::chrono::steady_clock::time_point timestamp;
    };

    struct ComputationResultWithMutex {
        std::mutex mtx;
        ComputationResult result;
    };

    std::shared_ptr<HardwareManager> hardware_manager_;
    std::unordered_map<std::string, std::shared_ptr<trajectory_planning::infrastructure::integration::MoveItAdapter>> moveit_adapters_;
    std::unordered_map<std::string, PinocchioContext> pinocchio_contexts_;
    std::unordered_map<std::string, std::string> mapping_base_frames_;

    std::unordered_map<std::string, std::unique_ptr<SPSCQueue<PoseCommand, 128>>> rt_buffers_;
    std::unordered_map<std::string, RtState> rt_states_;
    std::unordered_map<std::string, std::thread> rt_threads_;
    std::unordered_map<std::string, std::shared_ptr<std::atomic<bool>>> rt_running_per_mapping_;
    std::unordered_map<std::string, std::unique_ptr<std::mutex>> rt_states_mutexes_;

    std::unordered_map<std::string, std::thread> computation_threads_;
    std::unordered_map<std::string, std::shared_ptr<std::atomic<bool>>> computation_running_per_mapping_;
    std::unordered_map<std::string, std::unique_ptr<ComputationResultWithMutex>> computation_results_;

    std::unique_ptr<std::thread> queue_consumer_;
    std::atomic<bool> consumer_running_{false};

    std::mutex rt_buffers_mutex_;
    std::mutex lifecycle_mutex_;
    std::chrono::steady_clock steady_clock_;
    arm_controller::utils::VelocityStrictSolver solver_;
};
