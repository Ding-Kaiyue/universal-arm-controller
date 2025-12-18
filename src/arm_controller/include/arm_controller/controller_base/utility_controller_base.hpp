#ifndef __UTILITY_CONTROLLER_BASE_HPP__
#define __UTILITY_CONTROLLER_BASE_HPP__

#include "arm_controller/controller_base/mode_controller_base.hpp"
#include <rclcpp/rclcpp.hpp>
#include <any>

class UtilityControllerBase : public ModeControllerBase {
public:
    explicit UtilityControllerBase(std::string mode, rclcpp::Node::SharedPtr node)
        : ModeControllerBase(mode), node_(node) {}
    virtual ~UtilityControllerBase() = default;
    // void handle_message(std::any /*msg*/) override final {}

    virtual void start(const std::string& mapping) override = 0;
    virtual bool stop(const std::string& mapping) override = 0;
protected:
    rclcpp::Node::SharedPtr node_;
};

#endif // __UTILITY_CONTROLLER_BASE_HPP__
