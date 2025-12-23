#ifndef __POINT_RECORD_CONTROLLER_HPP__
#define __POINT_RECORD_CONTROLLER_HPP__

#include <controller_base/teach_controller_base.hpp>
#include <std_msgs/msg/string.hpp>
#include "arm_controller/hardware/hardware_manager.hpp"
#include "arm_controller/hardware/motor_data_recorder.hpp"
#include <memory>

class PointRecordController final : public TeachControllerBase {
public:
    explicit PointRecordController(const rclcpp::Node::SharedPtr& node);
    ~PointRecordController() override = default;

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

    // 硬件接口
    std::shared_ptr<HardwareManager> hardware_manager_;

    // 录制输出目录
    std::string record_output_dir_;

    // 当前记录的文件路径（用于 cancel 时删除）
    std::string current_point_file_path_;

    // 当前激活的 mapping
    std::string active_mapping_;

    // ✅ 电机数据记录器（使用 mmap 快速写入 CSV）
    std::shared_ptr<MotorDataRecorder> motor_recorder_;

};

#endif      // __POINT_RECORD_CONTROLLER_HPP__
