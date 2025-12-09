#ifndef __HARDWARE_MANAGER_HPP__
#define __HARDWARE_MANAGER_HPP__

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <mutex>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "hardware_driver/interface/robot_hardware.hpp"
#include "hardware_driver/driver/motor_driver_interface.hpp"
#include "trajectory_interpolator/moveit_spline_adapter.hpp"
#include "arm_controller/utils/motor_mode.hpp"

// 关节限位结构体
struct JointLimits {
    bool has_position_limits = false;
    double min_position = 0.0;
    double max_position = 0.0;
    bool has_velocity_limits = false;
    double max_velocity = 0.0;
    bool has_acceleration_limits = false;
    double max_acceleration = 0.0;
};

class HardwareManager : public hardware_driver::motor_driver::MotorStatusObserver,
                        public std::enable_shared_from_this<HardwareManager> {
public:
    // 单例模式获取实例
    static std::shared_ptr<HardwareManager> getInstance();

    // 初始化硬件驱动
    bool initialize(rclcpp::Node::SharedPtr node);

    // ============= 硬件驱动实例访问 =============
    std::shared_ptr<RobotHardware> get_hardware_driver() const;

    // ============= 核心接口 =============
    bool is_robot_stopped(const std::string& mapping) const;
    bool are_joints_within_limits(const std::string& mapping) const;
    bool is_system_healthy(const std::string& mapping) const;
    void reset_system_health(const std::string& mapping);
    void clear_emergency_stops(const std::string& mapping);

    // ============= 配置信息 =============
    const std::string& get_robot_type(const std::string& mapping) const;
    const std::string& get_interface(const std::string& mapping) const;
    std::vector<std::string> get_all_mappings() const;
    const std::vector<uint32_t>& get_motors_id(const std::string& mapping) const;
    const std::vector<std::string>& get_joint_names(const std::string& mapping) const;
    const std::string& get_frame_id(const std::string& mapping) const;  // 根据mapping获取frame_id
    const std::string& get_controller_name(const std::string& mapping) const;
    const std::string& get_planning_group(const std::string& mapping) const;
    uint8_t get_joint_count(const std::string& mapping) const;
    const std::vector<double>& get_initial_position(const std::string& mapping) const;
    const std::vector<double>& get_start_position(const std::string& mapping) const;

    // ============= 关节限位信息 =============
    const std::map<std::string, JointLimits>& get_joint_limits() const;
    void get_joint_limits(const std::string& joint_name, JointLimits& joint_limits);
    bool is_joint_emergency_stopped(const std::string& joint_name) const;
    int get_joint_violation_direction(const std::string& joint_name) const;

    // ============= 调试和状态管理 =============
    void print_system_status() const;

    // ============= 关节状态获取和控制 =============
    std::vector<double> get_current_joint_positions(const std::string& mapping) const;
    std::vector<double> get_current_joint_velocities(const std::string& mapping) const;
    bool send_hold_state_command(const std::string& mapping, const std::vector<double>& positions);

    // ============= 轨迹执行接口 =============
    bool executeTrajectory(const std::string& interface, const trajectory_interpolator::Trajectory& trajectory);

    // ============= 异步轨迹执行和控制接口 =============
    // 异步执行轨迹，返回执行ID用于后续控制
    std::string execute_trajectory_async(
        const std::string& mapping,
        const Trajectory& trajectory,
        bool show_progress = true);

    // 暂停指定mapping的轨迹执行（每个mapping同时只能执行一个轨迹）
    bool pause_trajectory(const std::string& mapping);

    // 恢复指定mapping的轨迹执行
    bool resume_trajectory(const std::string& mapping);

    // 取消指定mapping的轨迹执行
    bool cancel_trajectory(const std::string& mapping);

    // 等待指定mapping的轨迹执行完成（阻塞）
    bool wait_for_trajectory_completion(const std::string& mapping, int timeout_ms = 0);

    // ============= 电机使能/失能接口 =============
    bool enable_motors(const std::string& mapping, uint8_t mode);
    bool disable_motors(const std::string& mapping, uint8_t mode);

    // ============= MotorStatusObserver接口实现 =============
    void on_motor_status_update(const std::string& interface,
                               uint32_t motor_id,
                               const hardware_driver::motor_driver::Motor_Status& status) override;
    ~HardwareManager() = default;

private:
    HardwareManager() = default;

    // 禁用拷贝和赋值
    HardwareManager(const HardwareManager&) = delete;
    HardwareManager& operator=(const HardwareManager&) = delete;
    
    // ============= 核心状态 ============
    std::shared_ptr<RobotHardware> hardware_driver_;
    rclcpp::Node::SharedPtr node_;

    static inline std::shared_ptr<HardwareManager> instance_ = nullptr;
    static inline std::mutex instance_mutex_;

    // ============= 关节状态发布 =============
    mutable std::mutex joint_state_mutex_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    // ============= 配置信息 (按mapping) =============
    std::map<std::string, std::string> mapping_to_interface_;               // mapping -> interface
    std::unordered_map<std::string, std::string> interface_to_mapping_;     // interface -> mapping
    std::unordered_map<std::string, sensor_msgs::msg::JointState> mapping_joint_states_;    // mapping -> joint state
    std::map<std::string, std::vector<uint32_t>> motor_config_;             // mapping -> motor IDs
    std::map<std::string, std::vector<std::string>> joint_names_config_;    // mapping -> joint names
    std::map<std::string, std::string> controller_name_config_;             // mapping -> controller name
    std::map<std::string, std::string> planning_group_config_;              // mapping -> MoveIt planning group
    std::map<std::string, std::string> frame_id_config_;                    // mapping -> frame_id
    std::map<std::string, std::vector<double>> initial_position_config_;    // mapping -> initial position
    std::map<std::string, std::vector<double>> start_position_config_;      // mapping -> start position
    std::map<std::string, JointLimits> joint_limits_config_;                // mapping -> joint limits
    std::map<std::string, std::string> robot_type_config_;                  // mapping -> robot type


    // =============  状态监控变量 =============
    mutable std::mutex status_mutex_;
    // bool robot_stopped_ = true;
    bool system_healthy_ = true;
    std::map<std::string, bool> is_robot_stopped_;  
    std::map<std::string, bool> mapping_health_status_;
    std::map<std::string, bool> joint_emergency_stop_;  // 紧急停止状态
    std::map<std::string, int> joint_violation_direction_; // +1 正向越界，-1 反向越界，0 正常
    std::map<std::string, double> motor_temperatures_;  // 电机温度记录

    // 安全保护相关
    static constexpr double POSITION_MARGIN = 0.1;  // 位置安全边界 (弧度) - 约5.7度
    bool safety_enabled_ = true;

    std::atomic<int> health_check_counter_{0};  // 健康检查计数器

    // ============= 轨迹执行管理 =============
    mutable std::mutex execution_mutex_;
    std::map<std::string, std::string> mapping_to_execution_id_;  // mapping -> 当前执行ID（同一时间每个mapping只有一个轨迹执行）

    // ============= 内部方法 =============
    void update_joint_state(const std::string& interface, uint32_t motor_id,
                           const hardware_driver::motor_driver::Motor_Status& status);
    void publish_joint_state();
    bool load_joint_limits_config();
    bool load_hardware_config();
    void clear_mappings();  // 清除所有映射
    bool parse_mapping(const std::string& mapping_name, const YAML::Node& mapping_node); // 解析单个mapping
    void initialize_joint_state(const std::string& mapping_name); // 初始化JointState

    // 安全检查方法
    void check_safety_limits(const std::string& interface, uint32_t motor_id,
                           const hardware_driver::motor_driver::Motor_Status& status);
    void emergency_stop_joint(const std::string& interface, uint32_t motor_id);

};

#endif // __HARDWARE_MANAGER_HPP__
