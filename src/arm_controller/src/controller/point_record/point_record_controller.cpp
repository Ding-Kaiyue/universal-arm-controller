#include "point_record_controller.hpp"
#include "controller_interface.hpp"

PointRecordController::PointRecordController(const rclcpp::Node::SharedPtr & node)
    : RecordControllerBase("PointRecord", node)
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
    // is_active_[mapping] = true; // 由基类 start() 设置
    RCLCPP_INFO(node_->get_logger(), "PointRecordController activated");
    UtilityControllerBase::start(mapping);
}

bool PointRecordController::stop() {
    // sub_.reset();
    // is_active_[mapping] = false; // 由基类 stop() 设置
    RCLCPP_INFO(node_->get_logger(), "PointRecordController deactivated");
    return true;
    UtilityControllerBase::stop(mapping);
}

void PointRecordController::point_record_callback(const std_msgs::msg::String::SharedPtr msg) {
    // 检查已在 Lambda 中通过 is_active(mapping) 完成
    if (msg->data == "") return;
    RCLCPP_INFO(node_->get_logger(), "Received point record command: '%s'", msg->data.c_str());
    publish_point_record_name(msg->data);
}

void PointRecordController::publish_point_record_name(const std::string &name) {
    auto msg = std_msgs::msg::String();
    msg.data = name;
    pub_->publish(msg);
}

