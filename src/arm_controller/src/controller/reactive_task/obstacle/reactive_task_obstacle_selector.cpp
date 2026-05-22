#include "controller/reactive_task/obstacle/reactive_task_obstacle_selector.hpp"

#include <algorithm>
#include <cmath>

namespace arm_controller::controller::reactive_task {
namespace {

double minDistanceToLocalReference(
    const Eigen::Vector3d& point,
    const Eigen::Isometry3d& current_pose,
    const cp::TimedCartesianSampleList& reference_samples,
    const std::shared_ptr<arm_controller::kinematics::PinocchioForwardKinematics>&
        fk_provider,
    const rq::LinkCollisionEllipsoidList* collision_ellipsoids,
    const arm_controller::kinematics::ForwardKinematicsOutput::LinkPoseMap*
        current_link_poses) {
  double best = (point - current_pose.translation()).norm();
  if (collision_ellipsoids != nullptr && current_link_poses != nullptr) {
    for (const auto& ellipsoid : *collision_ellipsoids) {
      const auto pose_it = current_link_poses->find(ellipsoid.link_name);
      if (pose_it == current_link_poses->end()) {
        continue;
      }
      const Eigen::Vector3d ellipsoid_center =
          pose_it->second * ellipsoid.center_in_link;
      if (ellipsoid_center.allFinite()) {
        best = std::min(best, (point - ellipsoid_center).norm());
      }
    }
  }

  if (!fk_provider) {
    return best;
  }

  std::vector<std::string> collision_link_names;
  if (collision_ellipsoids != nullptr) {
    collision_link_names.reserve(collision_ellipsoids->size());
    for (const auto& ellipsoid : *collision_ellipsoids) {
      if (!ellipsoid.link_name.empty() &&
          std::find(collision_link_names.begin(), collision_link_names.end(),
                    ellipsoid.link_name) == collision_link_names.end()) {
        collision_link_names.push_back(ellipsoid.link_name);
      }
    }
  }

  for (const auto& sample : reference_samples) {
    if (!sample.has_ik_joint_target || sample.ik_joint_target.size() <= 0 ||
        !sample.ik_joint_target.allFinite()) {
      continue;
    }
    arm_controller::kinematics::LinkPoseResultList link_poses;
    Eigen::Isometry3d ee_pose = Eigen::Isometry3d::Identity();
    if (!fk_provider->computeLinkPoses(sample.ik_joint_target,
                                       collision_link_names,
                                       link_poses,
                                       nullptr,
                                       nullptr,
                                       &ee_pose)) {
      continue;
    }
    best = std::min(best, (point - ee_pose.translation()).norm());
    if (collision_ellipsoids == nullptr || link_poses.empty()) {
      continue;
    }
    for (const auto& ellipsoid : *collision_ellipsoids) {
      const auto pose_it = std::find_if(
          link_poses.begin(), link_poses.end(),
          [&ellipsoid](const arm_controller::kinematics::LinkPoseResult& pose) {
            return pose.link_name == ellipsoid.link_name;
          });
      if (pose_it == link_poses.end()) {
        continue;
      }
      const Eigen::Vector3d ellipsoid_center =
          pose_it->pose * ellipsoid.center_in_link;
      if (ellipsoid_center.allFinite()) {
        best = std::min(best, (point - ellipsoid_center).norm());
      }
    }
  }

  return best;
}

struct ObstacleCandidate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double distance_to_reference{0.0};
};

}  // namespace

ObstacleSelectionOutput ReactiveTaskObstacleSelector::select(
    const ObstacleSelectionInput& input) const {
  ObstacleSelectionOutput output;
  if (!input.enabled || !input.pointcloud_map) {
    return output;
  }

  const auto occupied_centers = input.pointcloud_map->occupiedCellCenters(0u);
  output.cell_count = occupied_centers.size();
  output.voxel_radius =
      0.5 * std::sqrt(3.0) * input.pointcloud_map->voxelSize();
  output.padding = std::max(0.0, input.obstacle_padding_m);
  output.obstacle_radius = output.voxel_radius + output.padding;
  output.selection_radius = std::max(0.01, input.obstacle_selection_radius_m);

  std::vector<ObstacleCandidate, Eigen::aligned_allocator<ObstacleCandidate>>
      candidates;
  candidates.reserve(occupied_centers.size());
  for (const auto& center : occupied_centers) {
    if (!center.allFinite()) {
      continue;
    }
    const double distance_to_reference = minDistanceToLocalReference(
        center,
        input.current_pose,
        input.reference_samples,
        input.fk_provider,
        input.collision_ellipsoids,
        input.current_link_poses);
    if (distance_to_reference <= output.selection_radius) {
      candidates.push_back(ObstacleCandidate{center, distance_to_reference});
    }
  }

  std::sort(candidates.begin(),
            candidates.end(),
            [](const ObstacleCandidate& a, const ObstacleCandidate& b) {
              return a.distance_to_reference < b.distance_to_reference;
            });

  output.candidate_count = candidates.size();
  const std::size_t obstacle_limit =
      static_cast<std::size_t>(std::max(0, input.max_obstacle_spheres));
  const std::size_t obstacle_count =
      std::min(obstacle_limit, candidates.size());
  output.obstacles.reserve(obstacle_count);
  for (std::size_t i = 0; i < obstacle_count; ++i) {
    LocalPlannerSphereObstacle obstacle;
    obstacle.name = input.obstacle_name_prefix + "_" + std::to_string(i);
    obstacle.center = candidates[i].center;
    obstacle.radius = output.obstacle_radius;
    output.obstacles.push_back(std::move(obstacle));
  }

  return output;
}

}  // namespace arm_controller::controller::reactive_task
