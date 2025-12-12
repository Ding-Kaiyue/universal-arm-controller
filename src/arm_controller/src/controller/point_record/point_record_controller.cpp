#include "point_record_controller.hpp"
#include "controller_interface.hpp"

PointRecordController::PointRecordController(const rclcpp::Node::SharedPtr & node)
    : TeachControllerBase("PointRecord", node)
{
    std::string input_topic, output_topic;
    node_->get_parameter("controllers.PointRecord.input_topic", input_topic);
    node_->get_parameter("controllers.PointRecord.output_topic", output_topic);

    sub_ = node_->create_subscription<std_msgs::msg::String>(
        input_topic, 10, std::bind(&PointRecordController::point_record_callback, this, std::placeholders::_1)
    );
    pub_ = node_->create_publisher<std_msgs::msg::String>(output_topic, 10);
}

void PointRecordController::start() {
    is_active_ = true;
    RCLCPP_INFO(node_->get_logger(), "PointRecordController activated");
}

bool PointRecordController::stop() {
    // sub_.reset();
    is_active_ = false;
    RCLCPP_INFO(node_->get_logger(), "PointRecordController deactivated");
    return true;
}

void PointRecordController::point_record_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (!is_active_) return;
    if (msg->data == "") return;
    RCLCPP_INFO(node_->get_logger(), "Received point record command: '%s'", msg->data.c_str());
    publish_point_record_name(msg->data);
}

void PointRecordController::publish_point_record_name(const std::string &name) {
    auto msg = std_msgs::msg::String();
    msg.data = name;
    pub_->publish(msg);
}

