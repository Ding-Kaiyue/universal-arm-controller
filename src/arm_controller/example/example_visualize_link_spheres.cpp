#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>

#include "algorithm/sphere_model/link_sphere_model.hpp"
#include "arm_controller/kinematics/forward_kinematics.hpp"

namespace fs = std::filesystem;
namespace sm = arm_controller::algorithm::sphere_model;
namespace rq = arm_controller::algorithm::reactive_qp;

namespace {

bool waitForCurrentJointPositions(
    const rclcpp::Node::SharedPtr& node,
    const std::vector<std::string>& joint_names,
    std::vector<double>& out_q,
    double timeout_sec = 3.0) {
    std::mutex msg_mutex;
    sensor_msgs::msg::JointState::SharedPtr latest_msg;

    auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
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

bool loadMappingConfig(
    const std::string& hardware_cfg_path,
    const std::string& mapping,
    std::string& robot_type,
    std::vector<std::string>& joint_names,
    std::vector<double>& start_position,
    std::string* error) {
    try {
        const YAML::Node root = YAML::LoadFile(hardware_cfg_path);
        const YAML::Node hw = root["hardware"];
        if (!hw || !hw.IsMap()) {
            if (error) {
                *error = "Missing 'hardware' map.";
            }
            return false;
        }
        const YAML::Node m = hw[mapping];
        if (!m || !m.IsMap()) {
            if (error) {
                *error = "Mapping '" + mapping + "' not found.";
            }
            return false;
        }
        robot_type = m["robot_type"] ? m["robot_type"].as<std::string>() : "";
        if (m["joint_names"] && m["joint_names"].IsSequence()) {
            for (const auto& jn : m["joint_names"]) {
                joint_names.push_back(jn.as<std::string>());
            }
        }
        if (m["start_position"] && m["start_position"].IsSequence()) {
            for (const auto& v : m["start_position"]) {
                start_position.push_back(v.as<double>());
            }
        }
        if (joint_names.empty()) {
            if (error) {
                *error = "joint_names is empty in mapping '" + mapping + "'.";
            }
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        if (error) {
            *error = e.what();
        }
        return false;
    }
}

std::string getVizDir() {
    const char* env = std::getenv("LINK_SPHERE_VIZ_DIR");
    if (env == nullptr || std::string(env).empty()) {
        return "/tmp/link_sphere_viz";
    }
    return std::string(env);
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("example_visualize_link_spheres");

    const std::string mapping = (argc > 1) ? argv[1] : "left_arm";
    const bool use_joint_states = (argc > 2) ? (std::stoi(argv[2]) != 0) : false;

    const std::string arm_share =
        ament_index_cpp::get_package_share_directory("arm_controller");
    const std::string robot_desc_share =
        ament_index_cpp::get_package_share_directory("robot_description");
    const std::string hardware_cfg_path = arm_share + "/config/hardware_config.yaml";

    std::string robot_type;
    std::vector<std::string> joint_names;
    std::vector<double> start_position;
    std::string error;
    if (!loadMappingConfig(
            hardware_cfg_path, mapping, robot_type, joint_names, start_position, &error)) {
        std::cerr << "[visualize_link_spheres] loadMappingConfig failed: " << error << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    pinocchio::Model model;
    try {
        const std::string urdf_path = robot_desc_share + "/urdf/" + robot_type + ".urdf";
        pinocchio::urdf::buildModel(urdf_path, model);
    } catch (const std::exception& e) {
        std::cerr << "[visualize_link_spheres] buildModel failed: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    std::vector<int> q_indices;
    q_indices.reserve(joint_names.size());
    for (const auto& jn : joint_names) {
        if (!model.existJointName(jn)) {
            std::cerr << "[visualize_link_spheres] joint not found in model: " << jn << std::endl;
            rclcpp::shutdown();
            return 1;
        }
        const pinocchio::JointIndex jid = model.getJointId(jn);
        q_indices.push_back(static_cast<int>(model.joints[jid].idx_q()));
    }

    pinocchio::FrameIndex ee_frame = 0;
    const pinocchio::JointIndex last_joint_id = model.getJointId(joint_names.back());
    for (pinocchio::FrameIndex fid = 0; fid < model.frames.size(); ++fid) {
        if (model.frames[fid].parentJoint == last_joint_id) {
            ee_frame = fid;
        }
    }

    arm_controller::kinematics::PinocchioForwardKinematics fk_provider(
        node, model, q_indices, ee_frame);
    if (!fk_provider.initialize()) {
        std::cerr << "[visualize_link_spheres] FK initialize failed." << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    std::vector<double> q_vec;
    bool used_realtime_joint_states = false;
    if (use_joint_states) {
        if (!waitForCurrentJointPositions(node, joint_names, q_vec, 5.0)) {
            std::cerr << "[visualize_link_spheres] failed to read /joint_states; fallback to start_position."
                      << std::endl;
        } else {
            used_realtime_joint_states = true;
        }
    }
    if (q_vec.empty()) {
        q_vec = start_position;
    }
    if (q_vec.size() != joint_names.size()) {
        q_vec.assign(joint_names.size(), 0.0);
    }
    Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(q_vec.data(), q_vec.size());

    std::cout << "joint_source: "
              << (used_realtime_joint_states ? "/joint_states" : "start_position_or_zero")
              << std::endl;
    std::cout << "q_used: [";
    for (std::size_t i = 0; i < q_vec.size(); ++i) {
        std::cout << q_vec[i];
        if (i + 1 != q_vec.size()) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;

    rq::LinkCollisionEllipsoidList ellipsoids;
    if (!sm::LinkSphereModel::buildEllipsoidsForMapping(
            hardware_cfg_path, mapping, model, ellipsoids, &error)) {
        std::cerr << "[visualize_link_spheres] buildEllipsoidsForMapping failed: " << error << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    arm_controller::kinematics::ForwardKinematicsOutput fk_out;
    if (!fk_provider.compute(q, fk_out)) {
        std::cerr << "[visualize_link_spheres] FK compute failed." << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    const std::string out_dir = getVizDir();
    std::error_code ec;
    fs::create_directories(out_dir, ec);
    if (ec) {
        std::cerr << "[visualize_link_spheres] create dir failed: " << out_dir << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    const std::string out_json = out_dir + "/link_spheres_" + mapping + ".json";
    std::ofstream ofs(out_json);
    if (!ofs.is_open()) {
        std::cerr << "[visualize_link_spheres] open output json failed: " << out_json << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    ofs << std::fixed << std::setprecision(6);
    ofs << "{\n";
    ofs << "  \"mapping\": \"" << mapping << "\",\n";
    ofs << "  \"robot_type\": \"" << robot_type << "\",\n";
    ofs << "  \"ellipsoids\": [\n";
    for (std::size_t i = 0; i < ellipsoids.size(); ++i) {
        const auto& e = ellipsoids[i];
        const auto it = fk_out.link_poses.find(e.link_name);
        if (it == fk_out.link_poses.end()) {
            continue;
        }
        const Eigen::Vector3d p_world = it->second * e.center_in_link;
        const Eigen::Matrix3d R_world = it->second.linear();
        ofs << "    {"
            << "\"link_name\": \"" << e.link_name << "\", "
            << "\"debug_name\": \"" << e.debug_name << "\", "
            << "\"radii\": [" << e.radii.x() << ", " << e.radii.y() << ", " << e.radii.z() << "], "
            << "\"center_world\": [" << p_world.x() << ", " << p_world.y() << ", " << p_world.z() << "]"
            << ", \"rotation_world\": ["
            << R_world(0, 0) << ", " << R_world(0, 1) << ", " << R_world(0, 2) << ", "
            << R_world(1, 0) << ", " << R_world(1, 1) << ", " << R_world(1, 2) << ", "
            << R_world(2, 0) << ", " << R_world(2, 1) << ", " << R_world(2, 2) << "]"
            << "}";
        ofs << (i + 1 == ellipsoids.size() ? "\n" : ",\n");
    }
    ofs << "  ]\n";
    ofs << "}\n";

    std::cout << "mapping: " << mapping << std::endl;
    std::cout << "robot_type: " << robot_type << std::endl;
    std::cout << "num_ellipsoids: " << ellipsoids.size() << std::endl;
    std::cout << "output_json: " << out_json << std::endl;
    std::cout << "done." << std::endl;

    rclcpp::shutdown();
    return 0;
}
