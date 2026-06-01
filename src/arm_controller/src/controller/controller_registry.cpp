#include "arm_controller/controller_interface.hpp"

#include "hold_state/hold_state_controller.hpp"
#include "move2initial/move2initial_controller.hpp"
#include "move2start/move2start_controller.hpp"
#include "system_start/system_start_controller.hpp"
#if ARM_CONTROLLER_ENABLE_VELOCITY_CONTROLLERS
#include "joint_velocity/joint_velocity_controller.hpp"
#include "cartesian_velocity/cartesian_velocity_controller.hpp"
#include "command_streaming/command_streaming_controller.hpp"
#endif
#include "ros2_action_control/ros2_action_control_controller.hpp"
#if ARM_CONTROLLER_ENABLE_MOTION_CONTROLLERS
#include "movec/movec_controller.hpp"
#include "movej/movej_controller.hpp"
#include "movel/movel_controller.hpp"
#include "reactive_task/reactive_task_controller.hpp"
#endif
// #include "point_record/point_record_controller.hpp"
// #include "point_replay/point_replay_controller.hpp"
#if ARM_CONTROLLER_ENABLE_TEACH_CONTROLLERS
#include "trajectory_record/trajectory_record_controller.hpp"
#include "trajectory_replay/trajectory_replay_controller.hpp"
#endif

std::unordered_map<std::string, ControllerInterface::Creator> get_available_controllers() {
    return {
        {"SystemStartController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<SystemStartController>(node); }},
        {"HoldStateController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<HoldStateController>(node); }},
        {"Move2InitialController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<Move2InitialController>(node); }},
        {"Move2StartController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<Move2StartController>(node); }},
#if ARM_CONTROLLER_ENABLE_VELOCITY_CONTROLLERS
        {"JointVelocityController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<JointVelocityController>(node); }},
        {"CartesianVelocityController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<CartesianVelocityController>(node); }},
        {"CommandStreamingController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<CommandStreamingController>(node); }},
#endif
        {"ROS2ActionControlController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<ROS2ActionControlController>(node); }},
#if ARM_CONTROLLER_ENABLE_MOTION_CONTROLLERS
        {"MoveCController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<MoveCController>(node); }},
        {"MoveJController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<MoveJController>(node); }},
        {"MoveLController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<MoveLController>(node); }},
        {"ReactiveTaskController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<ReactiveTaskController>(node); }},
#endif
        // {"PointRecordController", [](rclcpp::Node::SharedPtr node) {
        //     return std::make_shared<PointRecordController>(node); }},
        // {"PointReplayController", [](rclcpp::Node::SharedPtr node) {
        //     return std::make_shared<PointReplayController>(node); }},
#if ARM_CONTROLLER_ENABLE_TEACH_CONTROLLERS
        {"TrajectoryRecordController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<TrajectoryRecordController>(node); }},
        {"TrajectoryReplayController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<TrajectoryReplayController>(node); }}
#endif
    };
}
