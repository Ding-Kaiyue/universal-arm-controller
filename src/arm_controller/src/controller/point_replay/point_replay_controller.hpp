#ifndef __POINT_REPLAY_CONTROLLER_HPP__
#define __POINT_REPLAY_CONTROLLER_HPP__

#include <controller_base/teach_controller_base.hpp>
#include <std_msgs/msg/string.hpp>

class PointReplayController final : public TeachControllerBase {
public:
    explicit PointReplayController(const rclcpp::Node::SharedPtr& node);
    ~PointReplayController() override = default;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;
 
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

    void point_replay_callback(const std_msgs::msg::String::SharedPtr msg);
    void publish_point_replay_name(const std::string &name);
};

#endif      // __POINT_REPLAY_CONTROLLER_HPP__
