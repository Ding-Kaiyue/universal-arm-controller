#ifndef __TRAJECTORY_RECORD_CONTROLLER_HPP__
#define __TRAJECTORY_RECORD_CONTROLLER_HPP__

#include "controller_base/teach_controller_base.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "arm_controller/utils/joint_recorder.hpp"
#include "arm_controller/hardware/hardware_manager.hpp"
#include <memory>

class TrajectoryRecordController final: public TeachControllerBase {
public:
    explicit TrajectoryRecordController(const rclcpp::Node::SharedPtr& node);
    ~TrajectoryRecordController() override = default;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;

private:

    void teach_callback(const std_msgs::msg::String::SharedPtr msg);
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publish_trajectory_record_name(const std::string &name);

    // ============= 重力补偿相关 =============
    // 重力补偿定时器 (使用 pinocchio 内部计算)
    rclcpp::TimerBase::SharedPtr gravity_compensation_timer_;
    // 重力补偿定时器回调
    void gravity_compensation_timer_callback();
    // 发送重力补偿力矩到电机
    bool send_gravity_compensation(const std::string& mapping, const std::vector<double>& efforts);

    // 话题订阅 - 命令接收
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

    // 关节录制器实例
    std::unique_ptr<JointRecorder> recorder_;

    // 用于启动录制的最新关节状态
    sensor_msgs::msg::JointState::SharedPtr latest_joint_states_;

    // 录制输出目录
    std::string record_output_dir_;

    // 硬件接口
    std::shared_ptr<HardwareManager> hardware_manager_;

    // 当前激活的 mapping
    std::string active_mapping_;
};

#endif      // __TRAJECTORY_RECORD_CONTROLLER_HPP__
