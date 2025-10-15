#ifndef __TRAJECTORY_REPLAY_CONTROLLER_HPP__
#define __TRAJECTORY_REPLAY_CONTROLLER_HPP__

#include <controller_base/record_controller_base.hpp>
#include <std_msgs/msg/string.hpp>

class TrajectoryReplayController final: public RecordControllerBase {
public:
    explicit TrajectoryReplayController(const rclcpp::Node::SharedPtr& node);
    ~TrajectoryReplayController() override = default;
    
    void start() override;
    bool stop() override;

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

    void trajectory_replay_callback(const std_msgs::msg::String::SharedPtr msg);
    void publish_trajectory_replay_name(const std::string &name);
};

#endif      // __TRAJECTORY_REPLAY_CONTROLLER_HPP__
