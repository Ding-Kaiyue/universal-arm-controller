#ifndef __MOVE2INITIAL_CONTROLLER_HPP__
#define __MOVE2INITIAL_CONTROLLER_HPP__

#include "controller_base/utility_controller_base.hpp"
#include "hardware/hardware_manager.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_planning_v3/application/services/motion_planning_service.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "trajectory_interpolator/trajectory_interpolator.hpp"
#include "arm_controller/utils/trajectory_converter.hpp"
#include <unordered_map>

class Move2InitialController final : public UtilityControllerBase {
public:
    explicit Move2InitialController(const rclcpp::Node::SharedPtr& node);
    ~Move2InitialController() override = default;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;

    bool needs_hook_state() const override { return true; }
    
private:
    // 初始化轨迹规划服务
    void initialize_planning_services();
    // 核心功能函数
    bool move_to_initial_position(const std::string& mapping);
    
    // 辅助函数
    trajectory_interpolator::Trajectory interpolate_trajectory(
        const trajectory_interpolator::Trajectory& interpolator_trajectory,
        double max_velocity,
        double max_acceleration,
        double max_jerk,
        const std::string& mapping);

    bool execute_trajectory(
        const trajectory_interpolator::Trajectory& trajectory,
        const std::string& mapping);

    // 硬件接口
    std::shared_ptr<HardwareManager> hardware_manager_;

    // 轨迹规划相关 - 支持多mapping
    std::unordered_map<std::string, std::shared_ptr<trajectory_planning::application::services::MotionPlanningService>> motion_planning_services_;
    std::unordered_map<std::string, std::shared_ptr<trajectory_planning::infrastructure::integration::MoveItAdapter>> moveit_adapters_;
    std::unordered_map<std::string, std::string> mapping_to_planning_group_;

    // 轨迹插值器
    std::unique_ptr<TrajectoryInterpolator> trajectory_interpolator_;
};

#endif  // __MOVE2INITIAL_CONTROLLER_HPP__
