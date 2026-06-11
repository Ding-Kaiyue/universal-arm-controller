#include "whole_body_controller/node/whole_body_controller_node.hpp"

namespace whole_body_controller {

WholeBodyControllerNode::WholeBodyControllerNode()
    : rclcpp::Node("whole_body_controller_node") {
    RCLCPP_INFO(this->get_logger(), "Initializing Whole Body Controller Node");
    coordinator_ = std::make_unique<WholeBodyCoordinator>(*this);
    coordinator_->start();
}

}  // namespace whole_body_controller

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<whole_body_controller::WholeBodyControllerNode>());
    rclcpp::shutdown();
    return 0;
}
