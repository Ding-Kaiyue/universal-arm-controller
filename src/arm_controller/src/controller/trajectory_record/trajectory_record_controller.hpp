#ifndef __TRAJECTORY_RECORD_CONTROLLER_HPP__
#define __TRAJECTORY_RECORD_CONTROLLER_HPP__

#include "controller_base/teach_controller_base.hpp"
#include "controller_interfaces/msg/teaching_control.hpp"
// #include "std_msgs/msg/string.hpp"
#include "arm_controller/hardware/hardware_manager.hpp"
#include "arm_controller/hardware/recorder_manager.hpp"
#include "trajectory_smoother.hpp"
#include "arm_controller/ipc/command_queue_ipc.hpp"
#include <thread>
#include <atomic>
#include <queue>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <set>

class TrajectoryRecordController final: public TeachControllerBase {
public:
    explicit TrajectoryRecordController(const rclcpp::Node::SharedPtr& node);
    ~TrajectoryRecordController() override = default;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;

    void pause() override;
    void resume() override;
    void cancel() override;
    void complete() override;

    bool execute(const std::string& mapping, const std::string& command, const std::string& filename) override;
private:
    void teach_callback(const controller_interfaces::msg::TeachingControl::SharedPtr msg);

    // 持续重力补偿线程（每个 mapping 一个独立线程）
    void gravity_compensation_thread_func(const std::string& mapping);
    void start_gravity_compensation_thread(const std::string& mapping);
    void stop_gravity_compensation_thread(const std::string& mapping);

    void command_queue_consumer_thread() override;

    // 轨迹平滑处理（在录制完成后进行）
    void smooth_recorded_trajectory(const std::string& file_path);

    // 硬件接口
    std::shared_ptr<HardwareManager> hardware_manager_;

    // ✅ Recorder 生命周期管理（只管文件，不管 mapping）
    RecorderManager recorder_manager_;

    // 录制输出目录
    std::string record_dir_;

    // 重力补偿线程相关（每个 mapping 一个独立线程）
    std::map<std::string, std::unique_ptr<std::thread>> gravity_compensation_threads_;
    std::map<std::string, std::atomic<bool>> gravity_compensation_running_;  // 每个 mapping 一个运行标志
    std::mutex gravity_compensation_threads_mutex_;  // 保护线程 map 的并发访问
    const double GRAVITY_COMPENSATION_INTERVAL_MS = 10.0;  // 10ms 更新一次

    // ✅ 轨迹平滑处理器
    std::unique_ptr<TrajectorySmoother> trajectory_smoother_;

    // IPC 命令队列消费线程相关
    std::unique_ptr<std::thread> queue_consumer_;
    std::atomic<bool> consumer_running_{false};

    // ✅ Per-session complete flag - 防止单个录制会话中 complete 被执行多次
    std::atomic<bool> complete_executed_in_session_{false};

    // ✅ 当前录制会话的目标 mappings（支持双臂配置下单臂录制）
    std::vector<std::string> recording_mappings_;
    std::mutex recording_mappings_mutex_;
};

#endif      // __TRAJECTORY_RECORD_CONTROLLER_HPP__
