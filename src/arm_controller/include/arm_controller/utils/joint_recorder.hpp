#ifndef __JOINT_RECORDER_HPP__
#define __JOINT_RECORDER_HPP__

#include <fstream>
#include <filesystem>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <string>
#include <iomanip>
#include <chrono>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

/**
 * @brief 关节状态录制器，用于示教和回放
 *
 * 该类将关节状态录制到文本文件，并可将其作为轨迹回放。
 * 使用异步写入线程避免在录制期间阻塞主线程。
 */
class JointRecorder {
public:
    /**
     * @brief 构造关节录制器对象
     * @param record_rate_hz 录制频率（Hz），默认值：100Hz
     */
    explicit JointRecorder(double record_rate_hz = 100.0);

    /**
     * @brief 析构关节录制器对象
     */
    ~JointRecorder();

    /**
     * @brief 开始录制关节状态
     * @param file_path 保存录制文件的路径
     * @param sample_msg 样本关节状态消息，用于确定数据结构
     * @return 如果录制成功启动则返回 true
     */
    bool start(const std::string& file_path,
               const sensor_msgs::msg::JointState::SharedPtr& sample_msg);

    /**
     * @brief 停止录制
     */
    void stop();

    /**
     * @brief 将关节状态消息加入录制队列
     * 在 joint_states 回调中调用
     * @param msg 要录制的关节状态消息
     */
    void enqueue(const sensor_msgs::msg::JointState::SharedPtr& msg);

    /**
     * @brief 加载并回放录制的轨迹
     * @param txt_file_path 录制的轨迹文件路径
     * @return 准备好回放的 JointTrajectory 消息
     */
    trajectory_msgs::msg::JointTrajectory repeat(const std::string& txt_file_path);

    /**
     * @brief 检查是否正在录制
     * @return 如果正在录制则返回 true
     */
    bool isRecording() const { return recording_; }

private:
    /**
     * @brief 写入线程循环，将队列中的消息写入文件
     */
    void writerLoop();

    // 文件输出流
    std::ofstream file_;

    // 录制状态
    std::atomic<bool> recording_;

    // 时间跟踪
    rclcpp::Time last_record_time_;
    rclcpp::Time first_record_time_;

    // 文件路径
    std::string rec_file_path_;

    // 录制周期（秒）
    double record_period_;

    // 异步写入线程
    std::thread writer_thread_;
    std::queue<sensor_msgs::msg::JointState::SharedPtr> msg_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> stop_flag_;
};

#endif  // __JOINT_RECORDER_HPP__
