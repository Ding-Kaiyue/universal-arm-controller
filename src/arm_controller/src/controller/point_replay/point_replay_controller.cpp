#include "point_replay_controller.hpp"
#include "controller_interface.hpp"

PointReplayController::PointReplayController(const rclcpp::Node::SharedPtr & node)
    : RecordControllerBase("PointReplay", node) 
{
    std::string input_topic, output_topic;
    node_->get_parameter("controllers.PointReplay.input_topic", input_topic);
    node_->get_parameter("controllers.PointReplay.output_topic", output_topic);

    sub_ = node_->create_subscription<std_msgs::msg::String>(
        input_topic, 10, std::bind(&PointReplayController::point_replay_callback, this, std::placeholders::_1)
    );
    pub_ = node_->create_publisher<std_msgs::msg::String>(output_topic, 10);
}

void PointReplayController::start() {
    // is_active_[mapping] = true; // 由基类 start() 设置
    RCLCPP_INFO(node_->get_logger(), "PointReplayController activated");
    UtilityControllerBase::start(mapping);
}

bool PointReplayController::stop() {
    // sub_.reset();
    // is_active_[mapping] = false; // 由基类 stop() 设置
    RCLCPP_INFO(node_->get_logger(), "PointReplayController deactivated");
    return true;
    UtilityControllerBase::stop(mapping);
}

void PointReplayController::point_replay_callback(const std_msgs::msg::String::SharedPtr msg) {
    // 检查已在 Lambda 中通过 is_active(mapping) 完成
    if (msg->data == "") return;
    RCLCPP_INFO(node_->get_logger(), "Received point replay command: '%s'", msg->data.c_str());
    publish_point_replay_name(msg->data);
}

void PointReplayController::publish_point_replay_name(const std::string &name) {
    auto msg = std_msgs::msg::String();
    msg.data = name;
    pub_->publish(msg);
}

