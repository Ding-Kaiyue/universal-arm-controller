/*********************************************************************
 * @file        button_event_handler.hpp
 * @brief       按键事件处理器 - 连接硬件按键和SDK功能
 *********************************************************************/

#ifndef __BUTTON_EVENT_HANDLER_HPP__
#define __BUTTON_EVENT_HANDLER_HPP__

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <atomic>
#include "hardware_driver/driver/button_driver_interface.hpp"
#include "controller_interfaces/srv/sdk_cmd.hpp"

namespace arm_controller {

/**
 * @brief 按键事件处理器
 *
 * 实现ButtonEventObserver接口，接收硬件按键事件，
 * 调用SDK服务来执行示教、复现等功能。
 */
class ButtonEventHandler : public hardware_driver::button_driver::ButtonEventObserver,
                           public std::enable_shared_from_this<ButtonEventHandler> {
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点 (用于创建服务客户端)
     */
    explicit ButtonEventHandler(rclcpp::Node::SharedPtr node);
    ~ButtonEventHandler() override = default;

    /**
     * @brief 按键事件回调 (ButtonEventObserver接口)
     * @param interface CAN接口名称
     * @param status 按键状态
     */
    void on_button_event(const std::string& interface,
                        hardware_driver::button_driver::ButtonStatus status) override;

    /**
     * @brief 设置复现完成回调
     * @param callback 复现完成时调用的回调函数
     */
    using ReplayCompleteCallback = std::function<void(const std::string& interface)>;
    void set_replay_complete_callback(ReplayCompleteCallback callback);

    /**
     * @brief 通知复现完成 (由trajectory_replay_controller调用)
     * @param interface CAN接口名称
     */
    void notify_replay_complete(const std::string& interface);

    /**
     * @brief 获取上次按键的接口名称
     * @return 接口名称
     */
    const std::string& get_last_interface() const { return last_interface_; }

private:
    /// 处理进入示教事件
    void handle_entry_teach(const std::string& interface);

    /// 处理退出示教事件
    void handle_exit_teach(const std::string& interface);

    /// 处理轨迹复现事件
    void handle_teach_repeat(const std::string& interface);

    /// 调用SDK服务
    bool call_sdk_service(uint8_t command, const std::string& trajectory_name = "");

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<controller_interfaces::srv::SdkCmd>::SharedPtr sdk_client_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    ReplayCompleteCallback replay_complete_callback_;
    std::atomic<bool> is_teaching_{false};       ///< 是否在示教中
    std::atomic<bool> is_replaying_{false};      ///< 是否在复现中
    std::string current_trajectory_name_;        ///< 当前轨迹名称
    std::string last_interface_;                 ///< 上次按键的接口
};

}  // namespace arm_controller

#endif  // __BUTTON_EVENT_HANDLER_HPP__
