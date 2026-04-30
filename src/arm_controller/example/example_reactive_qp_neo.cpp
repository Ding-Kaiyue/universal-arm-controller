#include <chrono>
#include <algorithm>
#include <iomanip>
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

#include "algorithm/neo/reactive_qp_builder.hpp"
#include "algorithm/neo/reactive_qp_solver.hpp"
#include "algorithm/neo/task_velocity_generator.hpp"
#include "algorithm/neo/manipulability_gradient.hpp"
#include "algorithm/neo/joint_preference_loader.hpp"
#include "algorithm/neo/body_obstacle_constraint_builder.hpp"
#include "algorithm/sphere_model/link_sphere_model.hpp"
#include "algorithm/cartesian_path_planner/map/dummy_distance_field.hpp"
#include "algorithm/cartesian_path_planner/map/obstacle_primitives.hpp"
#include "arm_controller/kinematics/jacobian_provider.hpp"
#include "arm_controller/kinematics/forward_kinematics.hpp"

namespace rq = arm_controller::algorithm::reactive_qp;
namespace cp = arm_controller::algorithm::cartesian_path_planner;

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

void applyHumanLikeJointPreferenceTable(
    const std::vector<rq::HumanLikeJointPreference>& table,
    Eigen::VectorXd& q_min,
    Eigen::VectorXd& q_max,
    const Eigen::VectorXd& q_current,
    Eigen::VectorXd& out_qdot_ref,
    Eigen::VectorXd& out_posture_weights,
    double posture_k) {
    const int dof = static_cast<int>(q_current.size());
    out_qdot_ref = Eigen::VectorXd::Zero(dof);
    out_posture_weights = Eigen::VectorXd::Zero(dof);
    for (int i = 0; i < dof; ++i) {
        if (i >= q_min.size() || i >= q_max.size() || i >= static_cast<int>(table.size())) {
            continue;
        }
        const auto& p = table[static_cast<std::size_t>(i)];
        if (p.enable_range_clamp) {
            // Avoid immediate CBF infeasibility when current joint is outside the preferred range.
            // In that case, keep hard limits unchanged for now and rely on posture bias to pull back.
            const bool outside_preferred =
                (q_current(i) < p.preferred_min) || (q_current(i) > p.preferred_max);
            if (!outside_preferred) {
                q_min(i) = std::max(q_min(i), p.preferred_min);
                q_max(i) = std::min(q_max(i), p.preferred_max);
                if (q_min(i) >= q_max(i)) {
                    const double mid = 0.5 * (q_min(i) + q_max(i));
                    q_min(i) = mid - 1e-3;
                    q_max(i) = mid + 1e-3;
                }
            }
        }
        const double center = std::clamp(p.preferred_center, q_min(i) + 0.05, q_max(i) - 0.05);
        out_qdot_ref(i) = posture_k * (center - q_current(i));
        out_posture_weights(i) = std::max(0.0, p.posture_weight);
    }
}

void printHumanLikeJointPreferenceTable(const std::vector<rq::HumanLikeJointPreference>& table) {
    std::cout << "human_like_joint_preference_table:" << std::endl;
    std::cout << "  idx  name           clamp   min      max      center   weight" << std::endl;
    for (std::size_t i = 0; i < table.size(); ++i) {
        const auto& p = table[i];
        std::cout << "  " << std::setw(3) << i
                  << "  " << std::setw(12) << p.joint_name
                  << "  " << std::setw(5) << (p.enable_range_clamp ? "yes" : "no")
                  << "  " << std::setw(7) << std::fixed << std::setprecision(3) << p.preferred_min
                  << "  " << std::setw(7) << std::fixed << std::setprecision(3) << p.preferred_max
                  << "  " << std::setw(7) << std::fixed << std::setprecision(3) << p.preferred_center
                  << "  " << std::setw(6) << std::fixed << std::setprecision(3) << p.posture_weight
                  << std::endl;
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
    const std::string reactive_cfg_path =
        ament_index_cpp::get_package_share_directory("arm_controller") +
        "/config/reactive_task_config.yaml";

    std::string cfg_error;
    rq::ReactiveQpExampleConfig example_cfg;
    if (!rq::ReactiveQpExampleConfigLoader::loadFromYaml(
            reactive_cfg_path, example_cfg, &cfg_error)) {
        std::cerr << "[example_reactive_qp_neo] Failed to load example config from "
                  << reactive_cfg_path << ": " << cfg_error << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    const bool enable_obstacle_damper = example_cfg.qp_build.enable_obstacle_damper;
    const double obs_distance = example_cfg.obstacle_distance;

    // 1) Generate desired task-space twist from pose + optional feedforward twist.
    rq::TaskVelocityGenerator task_velocity_generator;
    rq::TaskVelocityInput task_input;
    rq::TaskVelocityConfig task_config = example_cfg.task_velocity;

    task_input.T_current = Eigen::Isometry3d::Identity();
    task_input.has_target_pose = true;
    task_input.T_target = Eigen::Isometry3d::Identity();
    task_input.T_target.translation() = example_cfg.target_translation;

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

    auto jacobian_provider = std::make_shared<arm_controller::kinematics::PinocchioJacobianProvider>(
        node, model, q_indices, v_indices, ee_frame);
    arm_controller::kinematics::PinocchioForwardKinematics fk_provider(
        node, model, q_indices, ee_frame);

    if (!fk_provider.initialize()) {
        std::cerr << "[example_reactive_qp_neo] Failed to initialize PinocchioForwardKinematics."
                  << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    if (!jacobian_provider->initialize()) {
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
        jacobian_provider->computeJacobian(q_current, "", Eigen::Vector3d::Zero());
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
    rq::HumanLikeJointPreferenceConfig joint_pref_cfg;
    if (!rq::JointPreferenceLoader::loadFromYaml(
            reactive_cfg_path, joint_names, joint_pref_cfg, &limits_error)) {
        std::cerr << "[example_reactive_qp_neo] Failed to load joint preferences from "
                  << reactive_cfg_path << ": " << limits_error << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    const auto& joint_pref_table = joint_pref_cfg.joints;
    printHumanLikeJointPreferenceTable(joint_pref_table);

    Eigen::VectorXd posture_qdot_ref;
    Eigen::VectorXd posture_joint_weights;
    applyHumanLikeJointPreferenceTable(
        joint_pref_table,
        q_min,
        q_max,
        q_current,
        posture_qdot_ref,
        posture_joint_weights,
        joint_pref_cfg.posture_k);

    // 4) Build QP problem from v_des + Pinocchio Jacobian.
    rq::ReactiveQpBuildInput qp_input;
    qp_input.q_current = q_current;
    qp_input.jacobian_task = jacobian_task;
    qp_input.desired_twist = task_output.v_des;
    rq::ManipulabilityGradient manipulability_gradient_solver(jacobian_provider);
    rq::ManipulabilityGradientConfig manipulability_cfg = example_cfg.manipulability;
    double log_m_value = 0.0;
    if (!manipulability_gradient_solver.compute(
            q_current, manipulability_cfg, qp_input.manipulability_gradient, &log_m_value)) {
        std::cerr << "[example_reactive_qp_neo] Failed to compute manipulability gradient."
                  << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    qp_input.posture_velocity_reference = posture_qdot_ref;
    qp_input.posture_joint_weights = posture_joint_weights;
    qp_input.qd_min = qd_min;
    qp_input.qd_max = qd_max;
    qp_input.joint_limits.q_min = q_min;
    qp_input.joint_limits.q_max = q_max;
    if (enable_obstacle_damper) {
        arm_controller::kinematics::ForwardKinematicsOutput fk_out;
        if (!fk_provider.compute(q_current, fk_out)) {
            std::cerr << "[example_reactive_qp_neo] FK compute failed for body obstacle constraints."
                      << std::endl;
            rclcpp::shutdown();
            return 1;
        }

        auto distance_field = std::make_shared<cp::DummyDistanceField>(
            Eigen::Vector3d(-2.0, -2.0, -2.0),
            Eigen::Vector3d(2.0, 2.0, 2.0));
        cp::SphereObstacle obstacle_sphere;
        obstacle_sphere.center = fk_out.ee_position + Eigen::Vector3d(obs_distance, 0.0, 0.0);
        obstacle_sphere.radius = 0.08;
        distance_field->addSphere(obstacle_sphere);

        const std::string hardware_cfg_path =
            ament_index_cpp::get_package_share_directory("arm_controller") +
            "/config/hardware_config.yaml";
        std::vector<rq::LinkCollisionEllipsoid> link_ellipsoids;
        std::string sphere_model_error;
        if (!arm_controller::algorithm::sphere_model::LinkSphereModel::buildEllipsoidsForMapping(
                hardware_cfg_path, mapping, model, link_ellipsoids, &sphere_model_error)) {
            std::cerr << "[example_reactive_qp_neo] Ellipsoid model build failed: "
                      << sphere_model_error << std::endl;
            rclcpp::shutdown();
            return 1;
        }
        std::string obstacle_error;
        const int generated = rq::BodyObstacleConstraintBuilder::appendLinkEllipsoidConstraints(
            q_current,
            fk_out.link_poses,
            link_ellipsoids,
            *jacobian_provider,
            distance_field,
            qp_input.obstacle_constraints,
            &obstacle_error);

        if (generated <= 0) {
            std::cerr << "[example_reactive_qp_neo] No body obstacle constraints generated: "
                      << obstacle_error << std::endl;
            rclcpp::shutdown();
            return 1;
        }
    }

    rq::ReactiveQpBuildConfig qp_config = example_cfg.qp_build;
    qp_config.enable_obstacle_damper = enable_obstacle_damper;

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
    const Eigen::VectorXd task_pred_vw = jacobian_task * qdot + slack;
    const Eigen::VectorXd task_residual_vw = task_pred_vw - task_output.v_des;

    std::cout << "mapping: " << mapping << std::endl;
    std::cout << "robot_type: " << robot_type << std::endl;
    std::cout << "enable_obstacle_damper: " << (enable_obstacle_damper ? "true" : "false") << std::endl;
    if (enable_obstacle_damper) {
        std::cout << "obstacle_config: distance=" << obs_distance
                  << ", safety=" << qp_config.obstacle_damper.safety_distance
                  << ", influence=" << qp_config.obstacle_damper.influence_distance
                  << ", gain=" << qp_config.obstacle_damper.cbf_gain << std::endl;
        std::cout << "body_obstacle_constraints: "
                  << qp_input.obstacle_constraints.size() << std::endl;
    }
    std::cout << "q_current: " << q_current.transpose() << std::endl;
    std::cout << "v_des: " << task_output.v_des.transpose() << std::endl;
    std::cout << "J (full 6x" << dof << "):\n" << jacobian_task << std::endl;
    std::cout << "task_pred assuming J=[v;w]: " << task_pred_vw.transpose() << std::endl;
    std::cout << "task_residual [v;w]: " << task_residual_vw.transpose()
              << ", norm=" << task_residual_vw.norm() << std::endl;
    std::cout << "qdot : " << qdot.transpose() << std::endl;
    std::cout << "slack: " << slack.transpose() << std::endl;
    std::cout << "Result: SUCCESS" << std::endl;

    rclcpp::shutdown();
    return 0;
}
