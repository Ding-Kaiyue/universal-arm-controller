#ifndef __TRAJECTORY_RECORD_CONTROLLER_HPP__
#define __TRAJECTORY_RECORD_CONTROLLER_HPP__

#include "controller_base/teach_controller_base.hpp"
#include "std_msgs/msg/string.hpp"
#include "arm_controller/hardware/hardware_manager.hpp"
#include "arm_controller/hardware/motor_data_recorder.hpp"
#include "trajectory_smoother.hpp"
#include <memory>

class TrajectoryRecordController final: public TeachControllerBase {
public:
    explicit TrajectoryRecordController(const rclcpp::Node::SharedPtr& node);
    ~TrajectoryRecordController() override = default;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;

    // ✅ 虚方法实现：用于控制录制过程（符合 TeachControllerBase 接口）
    void pause(const std::string& mapping = "") override;
    void resume(const std::string& mapping = "") override;
    void cancel(const std::string& mapping = "") override;
    void complete(const std::string& mapping = "") override;

private:
    void teach_callback(const std_msgs::msg::String::SharedPtr msg) override;
    void on_teaching_control(const std_msgs::msg::String::SharedPtr msg) override;

    // 持续重力补偿线程
    void gravity_compensation_thread_func();
    void start_gravity_compensation_thread();
    void stop_gravity_compensation_thread();

    // ✅ 轨迹平滑处理（在录制完成后进行）
    void smooth_recorded_trajectory(const std::string& file_path);

    // ✅ 电机数据记录器实例（观察者模式）
    std::shared_ptr<MotorDataRecorder> motor_recorder_;

    // 录制输出目录
    std::string record_dir_;

    // 当前录制的文件路径（用于 cancel 时删除）
    std::string current_recording_file_path_;

    // 硬件接口
    std::shared_ptr<HardwareManager> hardware_manager_;

    // 当前激活的 mapping
    std::string active_mapping_;

    std::atomic<bool> recording_ = false;   // 是否正在录制
    std::atomic<bool> paused_ = false;      // 录制是否暂停

    // 重力补偿线程相关
    std::unique_ptr<std::thread> gravity_compensation_thread_;
    std::atomic<bool> gravity_compensation_running_{false};
    const double GRAVITY_COMPENSATION_INTERVAL_MS = 10.0;  // 10ms 更新一次

    // ✅ 轨迹平滑处理器
    std::unique_ptr<TrajectorySmoother> trajectory_smoother_;
};

#endif      // __TRAJECTORY_RECORD_CONTROLLER_HPP__
