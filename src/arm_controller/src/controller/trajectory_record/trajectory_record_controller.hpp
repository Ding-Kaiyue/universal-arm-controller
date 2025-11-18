#ifndef __TRAJECTORY_RECORD_CONTROLLER_HPP__
#define __TRAJECTORY_RECORD_CONTROLLER_HPP__

#include <controller_base/record_controller_base.hpp>
#include <std_msgs/msg/string.hpp>

class TrajectoryRecordController final: public RecordControllerBase {
public:
    explicit TrajectoryRecordController(const rclcpp::Node::SharedPtr& node);
    ~TrajectoryRecordController() override = default;
    
    void start() override;
    bool stop() override;

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

    void trajectory_record_callback(const std_msgs::msg::String::SharedPtr msg);
    void publish_trajectory_record_name(const std::string &name);
};

#endif      // __TRAJECTORY_RECORD_CONTROLLER_HPP__
