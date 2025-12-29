#ifndef __POINT_REPLAY_CONTROLLER_HPP__
#define __POINT_REPLAY_CONTROLLER_HPP__

#include <controller_base/teach_controller_base.hpp>
#include <std_msgs/msg/string.hpp>
#include "arm_controller/hardware/hardware_manager.hpp"
#include "arm_controller/hardware/motor_data_reloader.hpp"
#include "trajectory_interpolator/trajectory_interpolator.hpp"
#include "trajectory_planning_v3/application/services/motion_planning_service.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

class PointReplayController final : public TeachControllerBase {
public:
    explicit PointReplayController(const rclcpp::Node::SharedPtr& node);
    ~PointReplayController() override = default;

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

    // 回放单个点
    void replay_point(const std::string& file_path);

    // 移动到目标点
    void move_to_point(const std::vector<double>& target_position, const std::string& mapping);

    // 辅助函数
    trajectory_interpolator::Trajectory interpolate_trajectory(
        const trajectory_interpolator::Trajectory& interpolator_trajectory,
        double max_velocity,
        double max_acceleration,
        double max_jerk,
        const std::string& mapping);

    void execute_trajectory(
        const trajectory_interpolator::Trajectory& trajectory,
        const std::string& mapping);

    // 硬件接口
    std::shared_ptr<HardwareManager> hardware_manager_;

    // 回放目录
    std::string replay_dir_;

    // 当前激活的 mapping
    std::string active_mapping_;

    // 当前执行的轨迹ID
    std::string current_execution_id_;

    // 轨迹规划相关 - 支持多臂mapping
    std::map<std::string, std::shared_ptr<trajectory_planning::application::services::MotionPlanningService>> motion_planning_services_;
    std::map<std::string, std::shared_ptr<trajectory_planning::infrastructure::integration::MoveItAdapter>> moveit_adapters_;
    std::map<std::string, std::string> mapping_to_planning_group_;

    // 轨迹插值器
    std::unique_ptr<TrajectoryInterpolator> trajectory_interpolator_;

    // 电机数据重放加载器
    std::unique_ptr<MotorDataReloader> motor_data_reloader_;
};

#endif      // __POINT_REPLAY_CONTROLLER_HPP__
