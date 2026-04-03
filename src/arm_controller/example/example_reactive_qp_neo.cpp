#include <chrono>
#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "algorithm/reactive_qp/reactive_qp_builder.hpp"
#include "algorithm/reactive_qp/reactive_qp_solver.hpp"
#include "algorithm/reactive_qp/task_velocity_generator.hpp"
#include "arm_controller/kinematics/jacobian_provider.hpp"

namespace rq = arm_controller::algorithm::reactive_qp;

namespace {

std::vector<std::string> defaultJointNamesForMapping(const std::string& mapping) {
    std::vector<std::string> names;
    names.reserve(6);

    std::string prefix;
    if (mapping == "left_arm") {
        prefix = "left_joint";
    } else if (mapping == "right_arm") {
        prefix = "right_joint";
    } else {
        prefix = "joint";
    }

    for (int i = 1; i <= 6; ++i) {
        names.push_back(prefix + std::to_string(i));
    }
    return names;
}

bool waitForCurrentJointPositions(
    const rclcpp::Node::SharedPtr& node,
    const std::vector<std::string>& joint_names,
    std::vector<double>& out_q,
    double timeout_sec = 3.0) {

    std::mutex msg_mutex;
    sensor_msgs::msg::JointState::SharedPtr latest_msg;

    auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [&](sensor_msgs::msg::JointState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(msg_mutex);
            latest_msg = std::move(msg);
        });

    const auto t0 = std::chrono::steady_clock::now();
    rclcpp::Rate rate(200.0);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        sensor_msgs::msg::JointState::SharedPtr msg_copy;
        {
            std::lock_guard<std::mutex> lock(msg_mutex);
            msg_copy = latest_msg;
        }

        if (msg_copy && msg_copy->name.size() == msg_copy->position.size()) {
            std::unordered_map<std::string, std::size_t> name_to_idx;
            name_to_idx.reserve(msg_copy->name.size());
            for (std::size_t i = 0; i < msg_copy->name.size(); ++i) {
                name_to_idx[msg_copy->name[i]] = i;
            }

            bool all_found = true;
            std::vector<double> q;
            q.reserve(joint_names.size());
            for (const auto& jn : joint_names) {
                auto it = name_to_idx.find(jn);
                if (it == name_to_idx.end()) {
                    all_found = false;
                    break;
                }
                q.push_back(msg_copy->position[it->second]);
            }

            if (all_found) {
                out_q = std::move(q);
                (void)sub;
                return true;
            }
        }

        const auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (elapsed > timeout_sec) {
            break;
        }
        rate.sleep();
    }

    (void)sub;
    return false;
}

bool loadJointLimitsFromYaml(
    const std::string& robot_type,
    const std::vector<std::string>& joint_names,
    Eigen::VectorXd& qd_min,
    Eigen::VectorXd& qd_max,
    Eigen::VectorXd& q_min,
    Eigen::VectorXd& q_max,
    std::string* error) {

    try {
        const std::string arm_controller_share =
            ament_index_cpp::get_package_share_directory("arm_controller");
        const std::string limits_path =
            arm_controller_share + "/config/" + robot_type + "_joint_limits.yaml";
        const YAML::Node root = YAML::LoadFile(limits_path);
        const YAML::Node limits_root = root["joint_limits"];
        if (!limits_root || !limits_root.IsMap()) {
            if (error != nullptr) {
                *error = "Missing or invalid 'joint_limits' map in " + limits_path;
            }
            return false;
        }

        const int dof = static_cast<int>(joint_names.size());
        qd_min = Eigen::VectorXd::Zero(dof);
        qd_max = Eigen::VectorXd::Zero(dof);
        q_min = Eigen::VectorXd::Zero(dof);
        q_max = Eigen::VectorXd::Zero(dof);

        for (int i = 0; i < dof; ++i) {
            const std::string& joint_name = joint_names[static_cast<std::size_t>(i)];
            const YAML::Node j = limits_root[joint_name];
            if (!j || !j.IsMap()) {
                if (error != nullptr) {
                    *error = "Joint '" + joint_name + "' not found in " + limits_path;
                }
                return false;
            }

            const bool has_pos =
                j["has_position_limits"] ? j["has_position_limits"].as<bool>() : false;
            const bool has_vel =
                j["has_velocity_limits"] ? j["has_velocity_limits"].as<bool>() : false;

            const double min_pos =
                j["min_position"] ? j["min_position"].as<double>() : -3.14;
            const double max_pos =
                j["max_position"] ? j["max_position"].as<double>() : 3.14;
            const double max_vel =
                j["max_velocity"] ? j["max_velocity"].as<double>() : 1.0;

            q_min(i) = has_pos ? min_pos : -3.14;
            q_max(i) = has_pos ? max_pos : 3.14;
            qd_min(i) = -(has_vel ? max_vel : 1.0);
            qd_max(i) = has_vel ? max_vel : 1.0;
        }
        return true;
    } catch (const std::exception& e) {
        if (error != nullptr) {
            *error = e.what();
        }
        return false;
    }
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("example_reactive_qp_neo");

    // Usage:
    //   ./example_reactive_qp_neo [mapping] [robot_type]
    // Example:
    //   ./example_reactive_qp_neo left_arm dual_arm620
    const std::string mapping = (argc > 1) ? argv[1] : "left_arm";
    const std::string robot_type = (argc > 2) ? argv[2] : "dual_arm620";

    // 1) Generate desired task-space twist from pose + optional feedforward twist.
    rq::TaskVelocityGenerator task_velocity_generator;
    rq::TaskVelocityInput task_input;
    rq::TaskVelocityConfig task_config;
    task_config.max_linear_speed = 1.0;
    task_config.max_angular_speed = 1.0;

    task_input.T_current = Eigen::Isometry3d::Identity();
    task_input.has_target_pose = true;
    task_input.T_target = Eigen::Isometry3d::Identity();
    task_input.T_target.translation() = Eigen::Vector3d(0.05, -0.02, 0.01);

    const rq::TaskVelocityOutput task_output =
        task_velocity_generator.compute(task_input, task_config);

    // 2) Build Pinocchio model from real URDF and map selected arm joints.
    pinocchio::Model model;
    try {
        const std::string robot_desc_path =
            ament_index_cpp::get_package_share_directory("robot_description");
        const std::string urdf_path = robot_desc_path + "/urdf/" + robot_type + ".urdf";
        pinocchio::urdf::buildModel(urdf_path, model);
    } catch (const std::exception& e) {
        std::cerr << "[example_reactive_qp_neo] Failed to build Pinocchio model from URDF: "
                  << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    const std::vector<std::string> joint_names = defaultJointNamesForMapping(mapping);

    std::vector<int> q_indices;
    std::vector<int> v_indices;
    q_indices.reserve(joint_names.size());
    v_indices.reserve(joint_names.size());

    for (const auto& jn : joint_names) {
        if (!model.existJointName(jn)) {
            std::cerr << "[example_reactive_qp_neo] Joint not found in URDF model: " << jn << std::endl;
            rclcpp::shutdown();
            return 1;
        }
        const pinocchio::JointIndex jid = model.getJointId(jn);
        q_indices.push_back(static_cast<int>(model.joints[jid].idx_q()));
        v_indices.push_back(static_cast<int>(model.joints[jid].idx_v()));
    }

    pinocchio::FrameIndex ee_frame = 0;
    const pinocchio::JointIndex last_joint_id = model.getJointId(joint_names.back());
    for (pinocchio::FrameIndex fid = 0; fid < model.frames.size(); ++fid) {
        if (model.frames[fid].parentJoint == last_joint_id) {
            ee_frame = fid;
        }
    }

    arm_controller::kinematics::PinocchioJacobianProvider jacobian_provider(
        node, model, q_indices, v_indices, ee_frame);
    if (!jacobian_provider.initialize()) {
        std::cerr << "[example_reactive_qp_neo] Failed to initialize PinocchioJacobianProvider."
                  << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    // 3) Read real q_current from /joint_states.
    std::vector<double> q_current_vec;
    if (!waitForCurrentJointPositions(node, joint_names, q_current_vec, 5.0)) {
        std::cerr << "[example_reactive_qp_neo] Failed to get real q_current from /joint_states for mapping '"
                  << mapping << "'." << std::endl;
        std::cerr << "Expected joints: ";
        for (const auto& jn : joint_names) {
            std::cerr << jn << " ";
        }
        std::cerr << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    const int dof = static_cast<int>(q_current_vec.size());
    Eigen::VectorXd q_current(dof);
    for (int i = 0; i < dof; ++i) {
        q_current(i) = q_current_vec[static_cast<std::size_t>(i)];
    }

    const Eigen::MatrixXd jacobian_task =
        jacobian_provider.computeJacobian(q_current, "", Eigen::Vector3d::Zero());
    if (jacobian_task.rows() != 6 || jacobian_task.cols() != dof || !jacobian_task.allFinite()) {
        std::cerr << "[example_reactive_qp_neo] Jacobian compute failed: shape="
                  << jacobian_task.rows() << "x" << jacobian_task.cols() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    // 4) Load real velocity/position limits from robot model joint-limits YAML.
    Eigen::VectorXd qd_min;
    Eigen::VectorXd qd_max;
    Eigen::VectorXd q_min;
    Eigen::VectorXd q_max;
    std::string limits_error;
    if (!loadJointLimitsFromYaml(
            robot_type, joint_names, qd_min, qd_max, q_min, q_max, &limits_error)) {
        std::cerr << "[example_reactive_qp_neo] Failed to load joint limits: "
                  << limits_error << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    // 4) Build QP problem from v_des + Pinocchio Jacobian.
    rq::ReactiveQpBuildInput qp_input;
    qp_input.q_current = q_current;
    qp_input.jacobian_task = jacobian_task;
    qp_input.desired_twist = task_output.v_des;
    qp_input.manipulability_gradient = Eigen::VectorXd::Zero(dof);
    qp_input.qd_min = qd_min;
    qp_input.qd_max = qd_max;
    qp_input.joint_limits.q_min = q_min;
    qp_input.joint_limits.q_max = q_max;

    rq::ReactiveQpBuildConfig qp_config;
    qp_config.enable_joint_limit_damper = false;
    qp_config.enable_obstacle_damper = false;
    qp_config.hessian.task_tracking_weight = 1.0;
    qp_config.hessian.joint_velocity_weight = 1e-4;
    qp_config.hessian.slack_weight = 100.0;
    qp_config.hessian.manipulability_weight = 0.0;

    rq::ReactiveQpProblem qp_problem;
    std::string error;
    if (!rq::ReactiveQpBuilder::build(qp_input, qp_config, qp_problem, &error)) {
        std::cerr << "[example_reactive_qp_neo] Build failed: " << error << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    // 5) Solve and extract qdot.
    rq::ReactiveQpSolver solver;
    Eigen::VectorXd solution;
    if (!solver.solve(qp_problem, solution, &error)) {
        std::cerr << "[example_reactive_qp_neo] Solve failed: " << error << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    const Eigen::VectorXd qdot = solution.head(dof);
    const Eigen::VectorXd slack = solution.tail(6);

    std::cout << "mapping: " << mapping << std::endl;
    std::cout << "robot_type: " << robot_type << std::endl;
    std::cout << "q_current: " << q_current.transpose() << std::endl;
    std::cout << "v_des: " << task_output.v_des.transpose() << std::endl;
    std::cout << "J(0, :): " << jacobian_task.row(0) << std::endl;
    std::cout << "qdot : " << qdot.transpose() << std::endl;
    std::cout << "slack: " << slack.transpose() << std::endl;
    std::cout << "Result: SUCCESS" << std::endl;

    rclcpp::shutdown();
    return 0;
}
