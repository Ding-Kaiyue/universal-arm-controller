#include "arm_controller/controller_interface.hpp"

// 只保留测试需要的控制器
#include "hold_state/hold_state_controller.hpp"
#include "system_start/system_start_controller.hpp"
// #include "joint_velocity/joint_velocity_controller.hpp"
// #include "cartesian_velocity/cartesian_velocity_controller.hpp"
// #include "move2initial/move2initial_controller.hpp"
// #include "move2start/move2start_controller.hpp"
#include "ros2_action_control/ros2_action_control_controller.hpp"
#include "movec/movec_controller.hpp"
#include "movej/movej_controller.hpp"
#include "movel/movel_controller.hpp"
// #include "point_record/point_record_controller.hpp"
// #include "point_replay/point_replay_controller.hpp"
// #include "trajectory_record/trajectory_record_controller.hpp"
// #include "trajectory_replay/trajectory_replay_controller.hpp"

std::unordered_map<std::string, ControllerInterface::Creator> get_available_controllers() {
    return {
        // 只注册测试需要的控制器
        {"HoldStateController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<HoldStateController>(node); }},
        {"SystemStartController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<SystemStartController>(node); }},
        // {"JointVelocityController", [](rclcpp::Node::SharedPtr node) {
        //     return std::make_shared<JointVelocityController>(node); }},
        // {"CartesianVelocityController", [](rclcpp::Node::SharedPtr node) {
        //     return std::make_shared<CartesianVelocityController>(node); }},
        // {"Move2InitialController", [](rclcpp::Node::SharedPtr node) {
        //     return std::make_shared<Move2InitialController>(node); }},
        // {"Move2StartController", [](rclcpp::Node::SharedPtr node) {
        //     return std::make_shared<Move2StartController>(node); }},
        {"ROS2ActionControlController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<ROS2ActionControlController>(node); }},
        {"MoveCController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<MoveCController>(node); }},
        {"MoveJController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<MoveJController>(node); }},
        {"MoveLController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<MoveLController>(node); }}

        // {"PointRecordController", [](rclcpp::Node::SharedPtr node) {
        //     return std::make_shared<PointRecordController>(node); }},
        // {"PointReplayController", [](rclcpp::Node::SharedPtr node) {
        //     return std::make_shared<PointReplayController>(node); }},
        // {"TrajectoryRecordController", [](rclcpp::Node::SharedPtr node) {
        //     return std::make_shared<TrajectoryRecordController>(node); }},
        // {"TrajectoryReplayController", [](rclcpp::Node::SharedPtr node) {
        //     return std::make_shared<TrajectoryReplayController>(node); }}
    };
}

