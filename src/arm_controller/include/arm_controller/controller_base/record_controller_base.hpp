#ifndef __RECORD_CONTROLLER_BASE_HPP__
#define __RECORD_CONTROLLER_BASE_HPP__

#include "arm_controller/controller_base/mode_controller_base.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <any>

class RecordControllerBase : public ModeControllerBase {
public:
    explicit RecordControllerBase(std::string mode, rclcpp::Node::SharedPtr node) 
        : ModeControllerBase(mode), node_(node) {}
    virtual ~RecordControllerBase() = default;

    // void handle_message(std::any /*msg*/) override final {}

    virtual void start(const std::string& mapping) override = 0;
    virtual bool stop(const std::string& mapping) override = 0;
protected:
    rclcpp::Node::SharedPtr node_;
};

#endif // __RECORD_CONTROLLER_BASE_HPP__
