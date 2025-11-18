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

    // 子类可以重写此方法以从消息中提取 mapping 信息
    virtual std::string get_mapping_from_message([[maybe_unused]] const typename T::SharedPtr msg) {
        return "";  // 默认实现
    }
};

#endif  // TRAJECTORY_CONTROLLER_BASE_HPP_
