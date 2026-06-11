#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "whole_body_controller/controller/whole_body_coordinator.hpp"

namespace whole_body_controller {

class WholeBodyControllerNode final : public rclcpp::Node {
public:
    WholeBodyControllerNode();
    ~WholeBodyControllerNode() override = default;

private:
    std::unique_ptr<WholeBodyCoordinator> coordinator_;
};

}  // namespace whole_body_controller
