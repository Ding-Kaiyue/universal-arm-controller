#ifndef __POINT_RECORD_CONTROLLER_HPP__
#define __POINT_RECORD_CONTROLLER_HPP__

#include <controller_base/record_controller_base.hpp>
#include <std_msgs/msg/string.hpp>

class PointRecordController final : public RecordControllerBase {
public:
    explicit PointRecordController(const rclcpp::Node::SharedPtr& node);
    ~PointRecordController() override = default;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;
    
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

    void point_record_callback(const std_msgs::msg::String::SharedPtr msg);
    void publish_point_record_name(const std::string &name);
};

#endif      // __POINT_RECORD_CONTROLLER_HPP__
