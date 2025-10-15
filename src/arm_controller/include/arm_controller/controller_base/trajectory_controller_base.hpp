#ifndef __TRAJECTORY_CONTROLLER_BASE_HPP__
#define __TRAJECTORY_CONTROLLER_BASE_HPP__

#include <arm_controller/controller_base/mode_controller_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <any>
#include <vector>
#include <string>
#include <arm_controller/hardware/hardware_manager.hpp>

class TrajectoryControllerBase : public ModeControllerBase {
public:
    explicit TrajectoryControllerBase(std::string mode) : ModeControllerBase(mode) {}
    virtual ~TrajectoryControllerBase() = default;
    virtual void handle_message(std::any msg) override = 0;
};

template<typename T>
class TrajectoryControllerImpl : public TrajectoryControllerBase {
public:
    explicit TrajectoryControllerImpl(std::string mode, rclcpp::Node::SharedPtr node) 
        : TrajectoryControllerBase(mode), node_(node) {}
    virtual ~TrajectoryControllerImpl() override = default;

    virtual void plan_and_execute(const std::string& mapping, const typename T::SharedPtr msg) = 0;

    virtual void trajectory_callback(const typename T::SharedPtr msg) = 0;

    void handle_message(std::any msg) override final{
        try {
            auto typed_msg = std::any_cast<typename T::SharedPtr>(msg);
            trajectory_callback(typed_msg);
        } catch (const std::bad_any_cast& e) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("TrajectoryControllerImpl"), "Failed to cast message to type " << typeid(T).name());
        }
    }

    virtual void start(const std::string& mapping) override = 0;
    virtual bool stop(const std::string& mapping) override = 0;
    
    // 轨迹控制器通常需要钩子状态来安全停止
    bool needs_hook_state() const override { return true; }

protected:
    rclcpp::Node::SharedPtr node_;

    // // 辅助方法：根据frame_id确定mapping
    // std::string determine_mapping_from_frame_id(const std::string& frame_id) {
    //     if (frame_id.empty()) {
    //         return "single_arm";  // 默认单臂
    //     }

    //     // 直接根据frame_id的前缀或内容确定mapping
    //     if (frame_id == "base_link") {
    //         return "single_arm";
    //     } else if (frame_id == "left_base_link" || frame_id.find("left_") == 0) {
    //         return "left_arm";
    //     } else if (frame_id == "right_base_link" || frame_id.find("right_") == 0) {
    //         return "right_arm";
    //     } else {
    //         return "single_arm";  // 默认单臂
    //     }
    // }

    // // 辅助方法：根据关节名称直接确定mapping
    // std::string determine_mapping_from_joint_names(const std::vector<std::string>& joint_names) {
    //     if (joint_names.empty()) {
    //         return "single_arm";
    //     }

    //     // 检查第一个关节名称的前缀来确定mapping
    //     const std::string& first_joint = joint_names[0];

    //     if (first_joint.find("left_") == 0) {
    //         return "left_arm";
    //     } else if (first_joint.find("right_") == 0) {
    //         return "right_arm";
    //     } else {
    //         return "single_arm";  // 默认单臂 (joint1, joint2等)
    //     }
    // }

};

#endif  // TRAJECTORY_CONTROLLER_BASE_HPP_
