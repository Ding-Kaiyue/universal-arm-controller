#include "arm_controller/kinematics/jacobian_provider.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

namespace arm_controller::kinematics {

MoveItJacobianProvider::MoveItJacobianProvider(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<trajectory_planning::infrastructure::integration::MoveItAdapter>& moveit_adapter)
    : node_(node), moveit_adapter_(moveit_adapter) {}

bool MoveItJacobianProvider::initialize() {
  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (initialized_) {
    return true;
  }
  if (!moveit_adapter_) {
    RCLCPP_ERROR(node_->get_logger(), "MoveItJacobianProvider: adapter is null");
    return false;
  }

  robot_model_ = moveit_adapter_->getRobotModel();
  if (!robot_model_) {
    RCLCPP_ERROR(node_->get_logger(), "MoveItJacobianProvider: failed to get robot model");
    return false;
  }

  robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
  joint_model_group_ = robot_model_->getJointModelGroup(moveit_adapter_->getPlanningGroupName());
  if (!joint_model_group_) {
    RCLCPP_ERROR(node_->get_logger(), "MoveItJacobianProvider: failed to get joint model group '%s'",
                 moveit_adapter_->getPlanningGroupName().c_str());
    return false;
  }

  const auto& links = joint_model_group_->getLinkModelNames();
  if (links.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "MoveItJacobianProvider: planning group has no links");
    return false;
  }

  default_ee_link_model_ = robot_state_->getLinkModel(links.back());
  if (!default_ee_link_model_) {
    RCLCPP_ERROR(node_->get_logger(), "MoveItJacobianProvider: failed to resolve default ee link '%s'",
                 links.back().c_str());
    return false;
  }

  initialized_ = true;
  return true;
}

const moveit::core::LinkModel* MoveItJacobianProvider::resolveLinkModel(const std::string& link_name) const {
  if (link_name.empty()) {
    return default_ee_link_model_;
  }
  auto it = link_cache_.find(link_name);
  if (it != link_cache_.end()) {
    return it->second;
  }
  const auto* link = robot_state_->getLinkModel(link_name);
  if (link) {
    link_cache_.emplace(link_name, link);
  }
  return link;
}

Eigen::MatrixXd MoveItJacobianProvider::computeJacobian(
    const Eigen::VectorXd& q,
    const std::string& link_name,
    const Eigen::Vector3d& point_in_link) const {
  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (!initialized_) {
    if (!const_cast<MoveItJacobianProvider*>(this)->initialize()) {
      return Eigen::MatrixXd();
    }
  }

  if (q.size() != static_cast<int>(joint_model_group_->getActiveJointModels().size())) {
    RCLCPP_ERROR(node_->get_logger(),
                 "MoveItJacobianProvider: q size (%ld) mismatch active joints (%zu)",
                 static_cast<long>(q.size()),
                 joint_model_group_->getActiveJointModels().size());
    return Eigen::MatrixXd();
  }

  std::vector<double> q_vec(q.data(), q.data() + q.size());
  robot_state_->setJointGroupPositions(joint_model_group_, q_vec);
  robot_state_->update();

  const auto* link_model = resolveLinkModel(link_name);
  if (!link_model) {
    RCLCPP_ERROR(node_->get_logger(), "MoveItJacobianProvider: link '%s' not found",
                 link_name.c_str());
    return Eigen::MatrixXd();
  }

  Eigen::MatrixXd jacobian;
  if (!robot_state_->getJacobian(joint_model_group_, link_model, point_in_link, jacobian)) {
    RCLCPP_ERROR(node_->get_logger(), "MoveItJacobianProvider: getJacobian failed");
    return Eigen::MatrixXd();
  }

  return jacobian;
}

PinocchioJacobianProvider::PinocchioJacobianProvider(
    const rclcpp::Node::SharedPtr& node,
    const pinocchio::Model& model,
    const std::vector<int>& q_indices,
    const std::vector<int>& v_indices,
    pinocchio::FrameIndex default_frame)
    : node_(node),
      model_(model),
      q_indices_(q_indices),
      v_indices_(v_indices),
      default_frame_(default_frame) {}

bool PinocchioJacobianProvider::initialize() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (initialized_) {
    return true;
  }
  if (q_indices_.empty() || v_indices_.empty() || q_indices_.size() != v_indices_.size()) {
    RCLCPP_ERROR(node_->get_logger(), "PinocchioJacobianProvider: invalid q/v index mapping");
    return false;
  }
  data_ = std::make_unique<pinocchio::Data>(model_);
  initialized_ = true;
  return true;
}

bool PinocchioJacobianProvider::resolveFrameId(
    const std::string& link_name,
    pinocchio::FrameIndex& frame_id) const {
  if (link_name.empty()) {
    frame_id = default_frame_;
    return true;
  }
  if (!model_.existFrame(link_name)) {
    return false;
  }
  frame_id = model_.getFrameId(link_name);
  return true;
}

Eigen::MatrixXd PinocchioJacobianProvider::computeJacobian(
    const Eigen::VectorXd& q,
    const std::string& link_name,
    const Eigen::Vector3d& point_in_link) const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!initialized_) {
    if (!const_cast<PinocchioJacobianProvider*>(this)->initialize()) {
      return Eigen::MatrixXd();
    }
  }
  if (q.size() != static_cast<int>(q_indices_.size())) {
    RCLCPP_ERROR(node_->get_logger(),
                 "PinocchioJacobianProvider: q size (%ld) mismatch mapping (%zu)",
                 static_cast<long>(q.size()), q_indices_.size());
    return Eigen::MatrixXd();
  }

  pinocchio::FrameIndex frame_id = default_frame_;
  if (!resolveFrameId(link_name, frame_id)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "PinocchioJacobianProvider: frame '%s' not found",
                 link_name.c_str());
    return Eigen::MatrixXd();
  }

  Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model_.nq);
  for (int i = 0; i < q.size(); ++i) {
    q_full(q_indices_[i]) = q(i);
  }

  pinocchio::forwardKinematics(model_, *data_, q_full);
  pinocchio::updateFramePlacements(model_, *data_);

  Eigen::MatrixXd J6_full = Eigen::MatrixXd::Zero(6, model_.nv);
  pinocchio::computeFrameJacobian(
      model_, *data_, q_full, frame_id,
      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J6_full);

  const int dof = static_cast<int>(v_indices_.size());
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, dof);
  for (int i = 0; i < dof; ++i) {
    J.col(i) = J6_full.col(v_indices_[i]);
  }

  if (!point_in_link.isZero(1e-12)) {
    const Eigen::Vector3d r_world = data_->oMf[frame_id].rotation() * point_in_link;
    for (int i = 0; i < dof; ++i) {
      J.block<3, 1>(0, i) += J.block<3, 1>(3, i).cross(r_world);
    }
  }

  return J;
}

}  // namespace arm_controller::kinematics
