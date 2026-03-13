#ifndef __TRAJECTORY_REPLAY_CONTROLLER_HPP__
#define __TRAJECTORY_REPLAY_CONTROLLER_HPP__

#include "controller_base/teach_controller_base.hpp"
#include "controller_interfaces/msg/teaching_control.hpp"
#include "std_msgs/msg/string.hpp"
#include "arm_controller/hardware/hardware_manager.hpp"
#include "arm_controller/hardware/motor_data_reloader.hpp"
#include "trajectory_segmenter.hpp"
#include "trajectory_interpolator/trajectory_interpolator.hpp"
#include "trajectory_planning_v3/application/services/motion_planning_service.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "trajectory_planning_v3/infrastructure/integration/tracik_adapter.hpp"
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

class TrajectoryReplayController final : public TeachControllerBase {
public:
    explicit TrajectoryReplayController(const rclcpp::Node::SharedPtr& node);
    ~TrajectoryReplayController() override = default;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;

    void pause() override;
    void resume() override;
    void cancel() override;
    void complete() override;

    // IPC execute 方法 - 处理 start/pause/resume/cancel/complete 动作
    bool execute(const std::string& mapping, const std::string& command, const std::string& filename) override;

private:
    // IPC 命令队列消费线程
    void command_queue_consumer_thread();
    // 初始化轨迹规划服务
    void initialize_planning_services();
    void teach_callback(const controller_interfaces::msg::TeachingControl::SharedPtr msg) override;

    // 后台回放线程
    void replay_thread_func(const std::string& file_path);

    // 私有辅助方法
    void move_to_start_point(const std::vector<double>& start_position, const std::string& mapping);

    trajectory_interpolator::Trajectory interpolate_trajectory(
        const trajectory_interpolator::Trajectory& interpolator_trajectory,
        double max_velocity,
        double max_acceleration,
        double max_jerk,
        const std::string& mapping);

    void execute_trajectory(
        const trajectory_interpolator::Trajectory& interpolator_traj,
        const std::string& mapping);

    std::string replay_dir_;
    std::shared_ptr<HardwareManager> hardware_manager_;

    std::atomic<bool> replaying_{false};
    std::atomic<bool> paused_{false};
    std::unique_ptr<std::thread> replay_thread_;

    // 当前执行的轨迹ID
    std::map<std::string, std::string> execution_ids_;
    std::mutex execution_mutex_;

    // ✅ 当前正在回放的映射（支持per-mapping状态）
    std::map<std::string, bool> replaying_mappings_;
    std::mutex state_mutex_;

    // IPC 命令的临时存储（用于 consumer_thread 传递参数）
    std::string ipc_command_filename_;
    std::mutex ipc_command_mutex_;

    // IPC 命令队列消费线程相关
    std::unique_ptr<std::thread> queue_consumer_;
    std::atomic<bool> consumer_running_{false};

    // 轨迹规划相关 - 支持多臂mapping
    std::map<std::string, std::shared_ptr<trajectory_planning::application::services::MotionPlanningService>> motion_planning_services_;
    std::map<std::string, std::shared_ptr<trajectory_planning::infrastructure::integration::MoveItAdapter>> moveit_adapters_;
    std::map<std::string, std::shared_ptr<trajectory_planning::infrastructure::integration::TracIKAdapter>> tracik_adapters_;
    std::map<std::string, std::string> mapping_to_planning_group_;

    // 轨迹插值器
    std::unique_ptr<TrajectoryInterpolator> trajectory_interpolator_;

    // 模块成员变量
    std::unique_ptr<MotorDataReloader> motor_data_reloader_;
    std::unique_ptr<TrajectorySegmenter> trajectory_segmenter_;

    const double TIME_STEP = 0.002; // 执行的时间间隔
};

#endif
