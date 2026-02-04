#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <controller_interfaces/srv/work_mode.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class TopicPublisherNode : public rclcpp::Node {
public:
    TopicPublisherNode() : Node("topic_publisher_node") {
        // 创建发布器
        left_arm_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/controller_api/movej_action/left_arm", 10);
        right_arm_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/controller_api/movej_action/right_arm", 10);

        // 创建 WorkMode 服务客户端
        mode_client_ = this->create_client<controller_interfaces::srv::WorkMode>(
            "/controller_api/controller_mode");

        RCLCPP_INFO(this->get_logger(), "Topic Publisher Node initialized");
    }

    void run() {
        // 等待服务可用
        while (!mode_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for controller_mode service...");
        }

        // 设置 left_arm 为 MoveJ 模式
        RCLCPP_INFO(this->get_logger(), "Setting left_arm to MoveJ mode...");
        call_work_mode_service("left_arm", "MoveJ");
        std::this_thread::sleep_for(1000ms);

        // 设置 right_arm 为 MoveJ 模式
        RCLCPP_INFO(this->get_logger(), "Setting right_arm to MoveJ mode...");
        call_work_mode_service("right_arm", "MoveJ");
        std::this_thread::sleep_for(1000ms);

        // 发送 left_arm joint1 运动到 45 度 (0.7854 rad)
        RCLCPP_INFO(this->get_logger(), "Publishing left_arm command (joint1 -> 45 deg)...");
        publish_movej_command(left_arm_pub_, 0.7854);  // 45 degrees in radians
        std::this_thread::sleep_for(1000ms);

        // 发送 right_arm joint1 运动到 45 度 (0.7854 rad)
        RCLCPP_INFO(this->get_logger(), "Publishing right_arm command (joint1 -> 45 deg)...");
        publish_movej_command(right_arm_pub_, 0.7854);  // 45 degrees in radians

        RCLCPP_INFO(this->get_logger(), "Commands published successfully");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_arm_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_arm_pub_;
    rclcpp::Client<controller_interfaces::srv::WorkMode>::SharedPtr mode_client_;

    void call_work_mode_service(
        const std::string& mapping,
        const std::string& mode) {

        auto request = std::make_shared<controller_interfaces::srv::WorkMode::Request>();
        request->mode = mode;
        request->mapping = mapping;

        auto future = mode_client_->async_send_request(request);

        // 等待响应，超时5秒
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(),
                       "WorkMode service call successful for %s: %s",
                       mapping.c_str(), response->success ? "true" : "false");
        } else {
            RCLCPP_ERROR(this->get_logger(),
                        "Failed to call WorkMode service for %s", mapping.c_str());
        }
    }

    void publish_movej_command(
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub,
        double joint1_target) {

        auto msg = std::make_shared<sensor_msgs::msg::JointState>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "";

        // 设置 6 个关节，只有 joint1 有目标值
        msg->position = {joint1_target, 0.0, 0.0, 0.0, 0.0, 0.0};

        pub->publish(*msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TopicPublisherNode>();
    node->run();

    rclcpp::shutdown();
    return 0;
}

