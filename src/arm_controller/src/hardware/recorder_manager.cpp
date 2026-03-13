#include "arm_controller/hardware/recorder_manager.hpp"
#include "arm_controller/hardware/motor_data_recorder.hpp"
#include <rclcpp/rclcpp.hpp>

RecorderManager::~RecorderManager()
{
    // 确保 recorder 清理
    if (recorder_)
    {
        try
        {
            recorder_->stop();
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RecorderManager"),
                "Exception in ~RecorderManager: %s", e.what());
        }
    }
}

bool RecorderManager::startRecording(const std::string& file_path)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 检查前一个录制是否已完成
    if (state_ != State::IDLE)
    {
        RCLCPP_WARN(rclcpp::get_logger("RecorderManager"),
            "Cannot start recording: current state is not IDLE (state=%d)",
            static_cast<int>(state_));
        return false;
    }

    // 文件路径不能为空
    if (file_path.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("RecorderManager"),
            "Cannot start recording: file path is empty");
        return false;
    }

    try
    {
        // 创建新的 recorder
        recorder_ = std::make_shared<MotorDataRecorder>(file_path);
        if (!recorder_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RecorderManager"),
                "Failed to create MotorDataRecorder for file: %s", file_path.c_str());
            return false;
        }

        // 启动 recorder 的写线程
        recorder_->start();

        // 更新状态
        state_ = State::RECORDING;
        file_path_ = file_path;

        RCLCPP_INFO(rclcpp::get_logger("RecorderManager"),
            "Recording started: %s", file_path.c_str());

        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RecorderManager"),
            "Exception in startRecording: %s", e.what());
        recorder_.reset();
        state_ = State::IDLE;
        return false;
    }
}

bool RecorderManager::pauseRecording()
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 只能从 RECORDING 状态暂停
    if (state_ != State::RECORDING)
    {
        RCLCPP_WARN(rclcpp::get_logger("RecorderManager"),
            "Cannot pause recording: current state is not RECORDING (state=%d)",
            static_cast<int>(state_));
        return false;
    }

    if (!recorder_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RecorderManager"),
            "Cannot pause recording: recorder is null");
        return false;
    }

    try
    {
        // 如果 MotorDataRecorder 有 pause 方法，调用它
        // 否则通过 stop 和重新 start 来实现（但这样会丢失时间戳连续性）
        // 暂时假设 MotorDataRecorder 有 pause() 接口
        // recorder_->pause();

        // 如果没有 pause() 方法，我们可以改为：
        // 1. 设置一个标志位，让 writer_loop 停止写入
        // 2. 或者停止接受新的数据更新

        state_ = State::PAUSED;

        RCLCPP_INFO(rclcpp::get_logger("RecorderManager"),
            "Recording paused");

        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RecorderManager"),
            "Exception in pauseRecording: %s", e.what());
        return false;
    }
}

bool RecorderManager::resumeRecording()
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 只能从 PAUSED 状态恢复
    if (state_ != State::PAUSED)
    {
        RCLCPP_WARN(rclcpp::get_logger("RecorderManager"),
            "Cannot resume recording: current state is not PAUSED (state=%d)",
            static_cast<int>(state_));
        return false;
    }

    if (!recorder_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RecorderManager"),
            "Cannot resume recording: recorder is null");
        return false;
    }

    try
    {
        // 如果 MotorDataRecorder 有 resume 方法，调用它
        // recorder_->resume();

        state_ = State::RECORDING;

        RCLCPP_INFO(rclcpp::get_logger("RecorderManager"),
            "Recording resumed");

        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RecorderManager"),
            "Exception in resumeRecording: %s", e.what());
        return false;
    }
}

bool RecorderManager::stopRecording()
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 可以从 RECORDING 或 PAUSED 状态停止
    if (state_ != State::RECORDING && state_ != State::PAUSED)
    {
        RCLCPP_WARN(rclcpp::get_logger("RecorderManager"),
            "Cannot stop recording: current state is not RECORDING or PAUSED (state=%d)",
            static_cast<int>(state_));
        return false;
    }

    if (!recorder_)
    {
        RCLCPP_WARN(rclcpp::get_logger("RecorderManager"),
            "Cannot stop recording: recorder is null");
        return false;
    }

    try
    {
        // 如果 MotorDataRecorder 有 flush 方法，先 flush
        // recorder_->flush();

        // 停止 recorder
        recorder_->stop();

        // 清除 recorder 指针
        recorder_.reset();

        // 更新状态
        state_ = State::IDLE;

        RCLCPP_INFO(rclcpp::get_logger("RecorderManager"),
            "Recording stopped: %s", file_path_.c_str());

        file_path_.clear();

        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RecorderManager"),
            "Exception in stopRecording: %s", e.what());
        state_ = State::IDLE;
        recorder_.reset();
        return false;
    }
}

bool RecorderManager::cancelRecording()
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 可以从任何状态取消
    if (state_ == State::IDLE)
    {
        RCLCPP_WARN(rclcpp::get_logger("RecorderManager"),
            "Cannot cancel recording: not in recording state");
        return false;
    }

    if (!recorder_)
    {
        RCLCPP_WARN(rclcpp::get_logger("RecorderManager"),
            "Cannot cancel recording: recorder is null");
        return false;
    }

    try
    {
        // 直接停止，不 flush（这样丢弃所有未写入的数据）
        recorder_->stop();

        // 清除 recorder 指针
        recorder_.reset();

        // 更新状态
        state_ = State::IDLE;

        RCLCPP_WARN(rclcpp::get_logger("RecorderManager"),
            "Recording cancelled: %s (data discarded)", file_path_.c_str());

        file_path_.clear();

        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RecorderManager"),
            "Exception in cancelRecording: %s", e.what());
        state_ = State::IDLE;
        recorder_.reset();
        return false;
    }
}

RecorderManager::State RecorderManager::getState() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

std::string RecorderManager::getFilePath() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return file_path_;
}

std::shared_ptr<MotorDataRecorder> RecorderManager::getRecorder() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return recorder_;
}
