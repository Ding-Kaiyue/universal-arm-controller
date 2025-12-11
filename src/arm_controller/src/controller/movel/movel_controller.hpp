#ifndef __MOVEL_CONTROLLER_HPP__
#define __MOVEL_CONTROLLER_HPP__

#include <controller_base/trajectory_controller_base.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "trajectory_planning_v3/application/services/motion_planning_service.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "trajectory_interpolator/trajectory_interpolator.hpp"
#include "arm_controller/utils/trajectory_converter.hpp"

class MoveLController final : public TrajectoryControllerImpl<geometry_msgs::msg::Pose> {
public:
    explicit MoveLController(const rclcpp::Node::SharedPtr& node);
    ~MoveLController() override = default;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;

private:
    // 初始化轨迹规划服务
    void initialize_planning_services();
    void trajectory_callback(const geometry_msgs::msg::Pose::SharedPtr msg) override;
    void plan_and_execute(const std::string& mapping, const geometry_msgs::msg::Pose::SharedPtr msg) override;

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

    // 话题订阅 - 命令接收
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_;

    std::string active_mapping_;

    // 轨迹规划相关 - 支持多臂mapping
    std::map<std::string, std::shared_ptr<trajectory_planning::application::services::MotionPlanningService>> motion_planning_services_;
    std::map<std::string, std::shared_ptr<trajectory_planning::infrastructure::integration::MoveItAdapter>> moveit_adapters_;
    std::map<std::string, std::string> mapping_to_planning_group_;

    // 轨迹插值器
    std::unique_ptr<TrajectoryInterpolator> trajectory_interpolator_;
};

#endif  // __MOVEL_CONTROLLER_HPP_
