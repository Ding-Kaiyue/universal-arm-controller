#include "trajectory_record_controller.hpp"
#include "controller_interface.hpp"

TrajectoryRecordController::TrajectoryRecordController(const rclcpp::Node::SharedPtr & node)
    : RecordControllerBase("TrajectoryRecord", node)
{
    std::string input_topic, output_topic;
    node_->get_parameter("controllers.TrajectoryRecord.input_topic", input_topic);
    node_->get_parameter("controllers.TrajectoryRecord.output_topic", output_topic);

    sub_ = node_->create_subscription<std_msgs::msg::String>(
        input_topic, 10, std::bind(&TrajectoryRecordController::trajectory_record_callback, this, std::placeholders::_1)
    );
    pub_ = node_->create_publisher<std_msgs::msg::String>(output_topic, 10);
}


void TrajectoryRecordController::start() {
    ModeControllerBase::start("");
    RCLCPP_INFO(node_->get_logger(), "TrajectoryRecordController activated");
}

bool TrajectoryRecordController::stop() {
    RCLCPP_INFO(node_->get_logger(), "TrajectoryRecordController deactivated");
    return ModeControllerBase::stop("");
}

void TrajectoryRecordController::trajectory_record_callback(const std_msgs::msg::String::SharedPtr msg) {
    // is_active(mapping) 检查已在 Lambda 中完成
    if (msg->data == "") return;
    RCLCPP_INFO(node_->get_logger(), "Received trajectory record command: '%s'", msg->data.c_str());
    publish_trajectory_record_name(msg->data);
}

void TrajectoryRecordController::publish_trajectory_record_name(const std::string &name) {
    auto msg = std_msgs::msg::String();
    msg.data = name;
    pub_->publish(msg);
}

