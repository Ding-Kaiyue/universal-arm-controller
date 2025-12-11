/*********************************************************************
 * @file        button_event_handler.cpp
 * @brief       按键事件处理器实现
 *********************************************************************/

#include "button_event_handler.hpp"
#include "robot_sdk/sdk_commands.hpp"

namespace arm_controller {

using ButtonStatus = hardware_driver::button_driver::ButtonStatus;

ButtonEventHandler::ButtonEventHandler(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    // 创建可重入回调组
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // 创建SDK服务客户端
    sdk_client_ = node_->create_client<controller_interfaces::srv::SdkCmd>(
        "/sdk_cmd",
        rmw_qos_profile_services_default,
        callback_group_);

    // 生成默认轨迹名称 (使用时间戳)
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "button_traj_" << time_t;
    current_trajectory_name_ = ss.str();

    RCLCPP_INFO(node_->get_logger(), "ButtonEventHandler 初始化完成");
}

void ButtonEventHandler::on_button_event(const std::string& interface, ButtonStatus status)
{
    RCLCPP_INFO(node_->get_logger(), "收到按键事件: interface=%s, status=%d",
                interface.c_str(), static_cast<int>(status));

    last_interface_ = interface;

    switch (status) {
        case ButtonStatus::ENTRY_TEACH:
            handle_entry_teach(interface);
            break;

        case ButtonStatus::EXIT_TEACH:
            handle_exit_teach(interface);
            break;

        case ButtonStatus::TEACH_REPEAT:
            handle_teach_repeat(interface);
            break;

        default:
            RCLCPP_WARN(node_->get_logger(), "未知按键状态: %d", static_cast<int>(status));
            break;
    }
}

void ButtonEventHandler::set_replay_complete_callback(ReplayCompleteCallback callback)
{
    replay_complete_callback_ = callback;
}

void ButtonEventHandler::notify_replay_complete(const std::string& interface)
{
    if (is_replaying_) {
        is_replaying_ = false;
        RCLCPP_INFO(node_->get_logger(), "复现完成: interface=%s", interface.c_str());

        // 调用回调发送FXJS信号 (LED熄灭)
        if (replay_complete_callback_) {
            replay_complete_callback_(interface);
        }
    }
}

void ButtonEventHandler::handle_entry_teach(const std::string& interface)
{
    if (is_teaching_) {
        RCLCPP_WARN(node_->get_logger(), "已在示教模式中，忽略重复进入请求");
        return;
    }

    if (is_replaying_) {
        RCLCPP_WARN(node_->get_logger(), "正在复现中，无法进入示教模式");
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "进入示教模式 (interface=%s)", interface.c_str());

    // 生成新的轨迹名称
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "button_traj_" << time_t;
    current_trajectory_name_ = ss.str();

    // 调用SDK开始示教
    if (call_sdk_service(robot_sdk::SDK_TEACH_START, current_trajectory_name_)) {
        is_teaching_ = true;
        RCLCPP_INFO(node_->get_logger(), "示教开始，轨迹名称: %s", current_trajectory_name_.c_str());
    } else {
        RCLCPP_ERROR(node_->get_logger(), "启动示教失败");
    }
}

void ButtonEventHandler::handle_exit_teach(const std::string& interface)
{
    if (!is_teaching_) {
        RCLCPP_WARN(node_->get_logger(), "未在示教模式中，忽略退出请求");
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "退出示教模式 (interface=%s)", interface.c_str());

    // 调用SDK停止示教
    if (call_sdk_service(robot_sdk::SDK_TEACH_STOP)) {
        is_teaching_ = false;
        RCLCPP_INFO(node_->get_logger(), "示教结束，轨迹已保存: %s", current_trajectory_name_.c_str());
    } else {
        RCLCPP_ERROR(node_->get_logger(), "停止示教失败");
    }
}

void ButtonEventHandler::handle_teach_repeat(const std::string& interface)
{
    if (is_teaching_) {
        RCLCPP_WARN(node_->get_logger(), "正在示教中，无法开始复现");
        return;
    }

    if (is_replaying_) {
        RCLCPP_WARN(node_->get_logger(), "已在复现中，忽略重复请求");
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "开始轨迹复现 (interface=%s, trajectory=%s)",
                interface.c_str(), current_trajectory_name_.c_str());

    // 调用SDK开始复现
    if (call_sdk_service(robot_sdk::SDK_TEACH_REPEAT, current_trajectory_name_)) {
        is_replaying_ = true;
        RCLCPP_INFO(node_->get_logger(), "复现开始: %s", current_trajectory_name_.c_str());
    } else {
        RCLCPP_ERROR(node_->get_logger(), "启动复现失败");
    }
}

bool ButtonEventHandler::call_sdk_service(uint8_t command, const std::string& trajectory_name)
{
    if (!sdk_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "SDK服务不可用");
        return false;
    }

    auto request = std::make_shared<controller_interfaces::srv::SdkCmd::Request>();
    request->command = command;
    request->trajectory_name = trajectory_name;
    request->mapping = "single_arm";

    // 使用promise/future等待结果
    auto promise = std::make_shared<std::promise<bool>>();
    auto future = promise->get_future();

    auto callback = [this, promise, command](
        rclcpp::Client<controller_interfaces::srv::SdkCmd>::SharedFuture response_future) {
        try {
            auto response = response_future.get();
            if (response->success) {
                RCLCPP_INFO(node_->get_logger(), "SDK命令 %d 执行成功", command);
                promise->set_value(true);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "SDK命令 %d 执行失败: %s",
                            command, response->message.c_str());
                promise->set_value(false);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "SDK服务调用异常: %s", e.what());
            promise->set_value(false);
        }
    };

    sdk_client_->async_send_request(request, callback);

    // 等待结果（带超时）
    if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
        return future.get();
    } else {
        RCLCPP_ERROR(node_->get_logger(), "SDK服务调用超时");
        return false;
    }
}

}  // namespace arm_controller
