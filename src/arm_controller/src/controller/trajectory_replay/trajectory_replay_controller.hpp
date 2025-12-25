#ifndef __TRAJECTORY_REPLAY_CONTROLLER_HPP__
#define __TRAJECTORY_REPLAY_CONTROLLER_HPP__

#include "controller_base/teach_controller_base.hpp"
#include "std_msgs/msg/string.hpp"
#include "arm_controller/hardware/hardware_manager.hpp"
#include "arm_controller/hardware/motor_data_reloader.hpp"
#include "trajectory_segmenter.hpp"
#include "trajectory_smoother.hpp"
#include "trajectory_interpolator/trajectory_interpolator.hpp"
#include "trajectory_planning_v3/application/services/motion_planning_service.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
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

    void pause(const std::string& mapping = "") override;
    void resume(const std::string& mapping = "") override;
    void cancel(const std::string& mapping = "") override;
    void complete(const std::string& mapping = "") override;

private:
    // 初始化轨迹规划服务
    void initialize_planning_services();
    void teach_callback(const std_msgs::msg::String::SharedPtr msg) override;
    void on_teaching_control(const std_msgs::msg::String::SharedPtr msg) override;

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
    std::string active_mapping_;

    std::atomic<bool> replaying_{false};
    std::atomic<bool> paused_{false};
    std::unique_ptr<std::thread> replay_thread_;

    // 当前执行的轨迹ID
    std::string current_execution_id_;

    // 轨迹规划相关 - 支持多臂mapping
    std::map<std::string, std::shared_ptr<trajectory_planning::application::services::MotionPlanningService>> motion_planning_services_;
    std::map<std::string, std::shared_ptr<trajectory_planning::infrastructure::integration::MoveItAdapter>> moveit_adapters_;
    std::map<std::string, std::string> mapping_to_planning_group_;

    // 轨迹插值器
    std::unique_ptr<TrajectoryInterpolator> trajectory_interpolator_;

    // 模块成员变量
    std::unique_ptr<MotorDataReloader> motor_data_reloader_;
    std::unique_ptr<TrajectorySegmenter> trajectory_segmenter_;
    std::unique_ptr<TrajectorySmoother> trajectory_smoother_;

    const double TIME_STEP = 0.05; // 执行的时间间隔
};

#endif
