#include "arm_controller/dynamics/gravity_compensator.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <Eigen/Dense>

#include <map>
#include <mutex>
#include <iostream>

namespace arm_controller {
namespace dynamics {

// PIMPL 实现类
class GravityCompensator::Impl {
public:
    // Pinocchio 模型和数据 (每个 mapping 一份)
    pinocchio::Model model;
    pinocchio::Data data;
    bool model_loaded{false};

    struct JointGroup {
        std::vector<size_t> indices;   // 在 model.nq 中的 idx_q
    };

    std::map<std::string, JointGroup> groups;

    // 线程安全
    mutable std::mutex mutex;
};

GravityCompensator::GravityCompensator() : impl_(std::make_unique<Impl>()) {}

GravityCompensator::~GravityCompensator() = default;

bool GravityCompensator::loadUrdf(const std::string& urdf_path) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    try {
        pinocchio::urdf::buildModel(urdf_path, impl_->model);
        impl_->data = pinocchio::Data(impl_->model);
        impl_->model_loaded = true;

        // 设置重力加速度（Z方向向下，9.81 m/s^2）
        impl_->model.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.81);

        std::cout << "[GravityCompensator] URDF loaded: "
                  << urdf_path
                  << " (nq=" << impl_->model.nq
                  << ", nv=" << impl_->model.nv << ")"
                  << ", gravity set to [0, 0, -9.81]"
                  << std::endl;

        return true;
    } catch (const std::exception& e) {
        std::cerr << "[GravityCompensator] Failed to load URDF: "
                  << e.what() << std::endl;
        return false;
    }
}

bool GravityCompensator::registerMapping(
    const std::string& mapping,
    const std::vector<std::string>& joint_names) {

    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->model_loaded) {
        std::cerr << "[GravityCompensator] Model not loaded, cannot register mapping: "
                  << mapping << std::endl;
        return false;
    }

    Impl::JointGroup group;

    for (const auto& joint_name : joint_names) {
        if (!impl_->model.existJointName(joint_name)) {
            std::cerr << "[GravityCompensator] Joint not found in URDF: "
                      << joint_name << std::endl;
            return false;
        }

        const auto joint_id = impl_->model.getJointId(joint_name);
        const auto idx_q = impl_->model.joints[joint_id].idx_q();
        group.indices.push_back(static_cast<size_t>(idx_q));
    }

    impl_->groups[mapping] = std::move(group);

    std::cout << "[GravityCompensator] Mapping registered: "
              << mapping << " (dof=" << impl_->groups[mapping].indices.size()
              << ")" << std::endl;

    return true;
}

bool GravityCompensator::hasMapping(const std::string& mapping) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->groups.find(mapping) != impl_->groups.end();
}

size_t GravityCompensator::getDof(const std::string& mapping) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->groups.find(mapping);
    if (it == impl_->groups.end()) {
        return 0;
    }
    return it->second.indices.size();
}

std::vector<double> GravityCompensator::computeGravity(
    const std::string& mapping,
    const std::vector<double>& joint_positions) {

    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->groups.find(mapping);
    if (it == impl_->groups.end()) {
        std::cerr << "[GravityCompensator] Mapping not found: "
                  << mapping << std::endl;
        return {};
    }

    const auto& indices = it->second.indices;
    if (joint_positions.size() != indices.size()) {
        std::cerr << "[GravityCompensator] Joint position size mismatch for mapping "
                  << mapping << ": expect " << indices.size()
                  << ", got " << joint_positions.size() << std::endl;
        return {};
    }

    Eigen::VectorXd q = Eigen::VectorXd::Zero(impl_->model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(impl_->model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(impl_->model.nv);

    // 使用正确的索引映射设置关节位置
    for (size_t i = 0; i < joint_positions.size(); ++i) {
        q(indices[i]) = joint_positions[i];
    }

    // 使用 RNEA 计算重力力矩
    Eigen::VectorXd tau = pinocchio::rnea(impl_->model, impl_->data, q, v, a);

    // 根据 indices 提取对应的力矩
    std::vector<double> result;
    result.reserve(joint_positions.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        result.push_back(tau(indices[i]));
    }

    return result;
}

}  // namespace dynamics
}  // namespace arm_controller
