#ifndef __MOTOR_DATA_RECORDER_HPP__
#define __MOTOR_DATA_RECORDER_HPP__

#include "hardware_driver/driver/motor_driver_interface.hpp"
#include <chrono>
#include <memory>
#include <map>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <unordered_map>

/**
 * @brief 电机数据记录器 - 超高性能版本（使用 mmap）
 * @details
 * - 实现 MotorStatusObserver 接口，通过观察者模式接收电机状态
 * - 内存映射文件（mmap）实现，性能极优：<10μs per record
 * - CPU 占用：<1%
 * - CSV 格式输出
 */
class MotorDataRecorder : public hardware_driver::motor_driver::MotorStatusObserver {
public:
    /**
     * @brief 构造函数 - 初始化 mmap 文件
     * @param output_file 输出文件路径
     * @param mapping 当前 mapping 名称
     * @param buffer_size mmap 缓冲区大小（默认 100MB，足够 ~8 小时示教数据）
     */
    explicit MotorDataRecorder(const std::string& output_file,
                              const std::string& mapping,
                              size_t buffer_size = 100 * 1024 * 1024);

    /**
     * @brief 析构函数 - 关闭 mmap，同步到磁盘
     */
    ~MotorDataRecorder() override;

    /**
     * @brief 实现 MotorStatusObserver 接口 - 处理电机状态更新
     */
    void on_motor_status_update(const std::string& interface,
                               uint32_t motor_id,
                               const hardware_driver::motor_driver::Motor_Status& status) override;

    /**
     * @brief 强制同步缓冲到磁盘
     */
    void sync_to_disk();

    /**
     * @brief 暂停录制（停止写入数据，但保留已录数据）
     * @param mapping 要暂停的 mapping 名称，必须与记录器的 mapping 匹配
     */
    void pause(const std::string& mapping) {
        if (mapping == mapping_) {
            is_paused_[mapping] = true;
        }
    }

    /**
     * @brief 恢复录制（继续写入数据）
     * @param mapping 要恢复的 mapping 名称，必须与记录器的 mapping 匹配
     */
    void resume(const std::string& mapping) {
        if (mapping == mapping_) {
            is_paused_[mapping] = false;
        }
    }

    /**
     * @brief 检查是否处于暂停状态
     */
    bool is_paused() const {
        auto it = is_paused_.find(mapping_);
        return it != is_paused_.end() ? it->second : false;
    }

    /**
     * @brief 获取已写入的字节数
     */
    size_t get_bytes_written() const { return buffer_pos_; }

    /**
     * @brief 获取输出文件路径
     */
    const std::string& get_output_file() const { return output_file_; }

    /**
     * @brief 检查是否成功初始化
     */
    bool is_open() const { return mmap_buffer_ != nullptr; }

    /**
     * @brief 获取当前 mapping
     */
    const std::string& get_mapping() const { return mapping_; }

private:
    std::string output_file_;
    std::string mapping_;                           // 当前 mapping 名称
    int fd_;                                    // 文件描述符
    char* mmap_buffer_;                         // mmap 内存映射
    size_t total_buffer_size_;                  // 总缓冲区大小
    size_t buffer_pos_;                         // 当前写入位置
    std::chrono::steady_clock::time_point start_time_;
    std::unordered_map<std::string, bool> is_paused_ = {{mapping_, false}};  // 暂停标志
};

#endif  // __MOTOR_DATA_RECORDER_HPP__
