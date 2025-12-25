#include "arm_controller/dynamics/gravity_compensator.hpp"

// Pinocchio 头文件只在 cpp 中包含，加快编译速度
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <map>
#include <mutex>
#include <iostream>

namespace arm_controller {
namespace dynamics {

// PIMPL 实现类
class GravityCompensator::Impl {
public:
    // Pinocchio 模型和数据 (每个 mapping 一份)
    std::map<std::string, pinocchio::Model> models;
    std::map<std::string, pinocchio::Data> data;
    std::map<std::string, std::string> urdf_paths;

    // 线程安全
    mutable std::mutex mutex;
};

GravityCompensator::GravityCompensator() : impl_(std::make_unique<Impl>()) {}

GravityCompensator::~GravityCompensator() = default;

bool GravityCompensator::loadModel(const std::string& mapping, const std::string& urdf_path) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    try {
        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_path, model);

        impl_->models[mapping] = model;
        impl_->data[mapping] = pinocchio::Data(model);
        impl_->urdf_paths[mapping] = urdf_path;

        std::cout << "[GravityCompensator] Model loaded for '" << mapping
                  << "' from " << urdf_path
                  << " (nq=" << model.nq << ", nv=" << model.nv << ")" << std::endl;

        return true;
    } catch (const std::exception& e) {
        std::cerr << "[GravityCompensator] Failed to load model for '" << mapping
                  << "': " << e.what() << std::endl;
        return false;
    }
}

bool GravityCompensator::hasModel(const std::string& mapping) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->models.find(mapping) != impl_->models.end();
}

std::vector<double> GravityCompensator::computeGravityTorques(
    const std::string& mapping,
    const std::vector<double>& joint_positions) {

    std::lock_guard<std::mutex> lock(impl_->mutex);

    // 检查模型是否存在
    auto model_it = impl_->models.find(mapping);
    if (model_it == impl_->models.end()) {
        std::cerr << "[GravityCompensator] Model not found for mapping: " << mapping << std::endl;
        return std::vector<double>();
    }

    auto& model = model_it->second;
    auto& data = impl_->data[mapping];

    // 构建配置向量 q
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    size_t num_joints = std::min(joint_positions.size(), static_cast<size_t>(model.nq));
    for (size_t i = 0; i < num_joints; ++i) {
        q(i) = joint_positions[i];
    }

    // 零速度和零加速度 (只计算重力项)
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

    // 使用 RNEA 算法计算重力力矩
    // tau = M(q) * 0 + C(q, 0) * 0 + g(q) = g(q)
    Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v, a);

    // 转换为 std::vector
    std::vector<double> gravity_torques(tau.data(), tau.data() + tau.size());

    return gravity_torques;
}

size_t GravityCompensator::getNumJoints(const std::string& mapping) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->models.find(mapping);
    if (it == impl_->models.end()) {
        return 0;
    }
    return static_cast<size_t>(it->second.nq);
}

std::string GravityCompensator::getUrdfPath(const std::string& mapping) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->urdf_paths.find(mapping);
    if (it == impl_->urdf_paths.end()) {
        return "";
    }
    return it->second;
}

}  // namespace dynamics
}  // namespace arm_controller
