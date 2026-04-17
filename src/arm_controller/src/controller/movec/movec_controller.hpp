#ifndef __MOVEC_CONTROLLER_HPP__
#define __MOVEC_CONTROLLER_HPP__

#include <controller_base/trajectory_controller_base.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "trajectory_planning_v3/application/services/motion_planning_service.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "trajectory_planning_v3/infrastructure/integration/tracik_adapter.hpp"
#include "trajectory_interpolator/trajectory_interpolator.hpp"
#include "arm_controller/utils/trajectory_converter.hpp"
#include "arm_controller/ipc/command_queue_ipc.hpp"
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>

class MoveCController final : public TrajectoryControllerImpl<geometry_msgs::msg::PoseArray> {
public:
    explicit MoveCController(const rclcpp::Node::SharedPtr& node);
    ~MoveCController() override = default;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override; 

    bool execute(const std::string& mapping, const std::vector<double>& parameters) override;

private:
    // 初始化轨迹规划服务
    void initialize_planning_services();
    void trajectory_callback(const std::string& mapping, const geometry_msgs::msg::PoseArray::SharedPtr msg) override;
    void plan_and_execute(const std::string& mapping, const geometry_msgs::msg::PoseArray::SharedPtr msg) override;

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
    
    // 队列消费线程 - 后台处理来自C++ API的命令
    void command_queue_consumer_thread() override;

    // 后台规划工作线程
    void planning_worker_thread();

    // 硬件接口
    std::shared_ptr<HardwareManager> hardware_manager_;

    // 轨迹规划相关 - 支持多臂mapping
    std::map<std::string, std::shared_ptr<trajectory_planning::application::services::MotionPlanningService>> motion_planning_services_;
    std::map<std::string, std::shared_ptr<trajectory_planning::infrastructure::integration::MoveItAdapter>> moveit_adapters_;
    std::map<std::string, std::shared_ptr<trajectory_planning::infrastructure::integration::TracIKAdapter>> tracik_adapters_;
    std::map<std::string, std::string> mapping_to_planning_group_;

    // 轨迹插值器
    std::unique_ptr<TrajectoryInterpolator> trajectory_interpolator_;

    // 队列消费者线程
    std::unique_ptr<std::thread> queue_consumer_;
    std::atomic<bool> consumer_running_{false};

    // 规划任务队列（用于异步规划）
    struct PlanningTask {
        std::string mapping;
        geometry_msgs::msg::PoseArray::SharedPtr msg;
    };
    std::queue<PlanningTask> planning_queue_;
    std::mutex planning_queue_mutex_;
    std::condition_variable planning_queue_cv_;
    std::unique_ptr<std::thread> planning_worker_;
    std::atomic<bool> planning_worker_running_{false};

    // 规划状态追踪
    std::map<std::string, bool> last_planning_success_;
    std::mutex planning_state_mutex_;
};


#endif  // __MOVEC_CONTROLLER_HPP__
