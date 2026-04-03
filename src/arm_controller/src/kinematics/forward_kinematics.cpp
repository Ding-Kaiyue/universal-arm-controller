#include "arm_controller/kinematics/forward_kinematics.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace arm_controller::kinematics {

PinocchioForwardKinematics::PinocchioForwardKinematics(
    const rclcpp::Node::SharedPtr& node,
    const pinocchio::Model& model,
    const std::vector<int>& q_indices,
    pinocchio::FrameIndex default_ee_frame)
    : node_(node),
      model_(model),
      q_indices_(q_indices),
      default_ee_frame_(default_ee_frame) {}

bool PinocchioForwardKinematics::initialize() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (initialized_) {
        return true;
    }
    if (q_indices_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "PinocchioForwardKinematics: q index mapping is empty");
        return false;
    }
    data_ = std::make_unique<pinocchio::Data>(model_);
    initialized_ = true;
    return true;
}

bool PinocchioForwardKinematics::resolveFrameId(
    const std::string& link_name,
    pinocchio::FrameIndex& frame_id) const {
    if (link_name.empty()) {
        frame_id = default_ee_frame_;
        return true;
    }
    if (!model_.existFrame(link_name)) {
        return false;
    }
    frame_id = model_.getFrameId(link_name);
    return true;
}

bool PinocchioForwardKinematics::compute(
    const Eigen::VectorXd& q,
    ForwardKinematicsOutput& out,
    const std::string& ee_link_name) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!initialized_) {
        if (!const_cast<PinocchioForwardKinematics*>(this)->initialize()) {
        return false;
        }
    }
    if (q.size() != static_cast<int>(q_indices_.size())) {
        RCLCPP_ERROR(node_->get_logger(),
                    "PinocchioForwardKinematics: q size (%ld) mismatch mapping (%zu)",
                    static_cast<long>(q.size()), q_indices_.size());
        return false;
    }

    pinocchio::FrameIndex ee_frame_id = default_ee_frame_;
    if (!resolveFrameId(ee_link_name, ee_frame_id)) {
        RCLCPP_ERROR(node_->get_logger(),
                    "PinocchioForwardKinematics: ee frame '%s' not found",
                    ee_link_name.c_str());
        return false;
    }

    Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model_.nq);
    for (int i = 0; i < q.size(); ++i) {
        q_full(q_indices_[i]) = q(i);
    }

    pinocchio::forwardKinematics(model_, *data_, q_full);
    pinocchio::updateFramePlacements(model_, *data_);

    out.link_poses.clear();
    out.link_poses.reserve(model_.nframes);
    const pinocchio::FrameIndex num_frames =
        static_cast<pinocchio::FrameIndex>(model_.nframes);
    for (pinocchio::FrameIndex i = 0; i < num_frames; ++i) {
        const auto& frame = model_.frames[i];
        if (frame.type != pinocchio::BODY) {
        continue;
        }
        const auto& placement = data_->oMf[i];
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = placement.rotation();
        T.translation() = placement.translation();
        out.link_poses[frame.name] = T;
    }

    const auto& ee_placement = data_->oMf[ee_frame_id];
    out.ee_pose = Eigen::Isometry3d::Identity();
    out.ee_pose.linear() = ee_placement.rotation();
    out.ee_pose.translation() = ee_placement.translation();
    out.ee_position = ee_placement.translation();
    out.ee_rotation = ee_placement.rotation();

    return true;
}

}  // namespace arm_controller::kinematics
