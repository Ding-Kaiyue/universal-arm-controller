#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "algorithm/cartesian_path_planner/map/camera_driver_pointcloud_map_adapter.hpp"
#include "algorithm/cartesian_path_planner/types.hpp"
#include "algorithm/neo/body_obstacle_constraint_builder.hpp"
#include "arm_controller/kinematics/forward_kinematics.hpp"
#include "controller/reactive_task/local_planner/reactive_task_local_planner.hpp"

namespace arm_controller::controller::reactive_task {

namespace cp = arm_controller::algorithm::cartesian_path_planner;
namespace rq = arm_controller::algorithm::reactive_qp;

struct ObstacleSelectionInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool enabled{true};
  std::string obstacle_name_prefix{"camera_obstacle"};
  std::shared_ptr<cp::CameraDriverPointcloudMapAdapter> pointcloud_map;
  double obstacle_padding_m{0.0};
  double obstacle_selection_radius_m{0.25};
  int max_obstacle_spheres{0};
  Eigen::Isometry3d current_pose{Eigen::Isometry3d::Identity()};
  cp::TimedCartesianSampleList reference_samples;
  std::shared_ptr<arm_controller::kinematics::PinocchioForwardKinematics>
      fk_provider;
  const rq::LinkCollisionEllipsoidList* collision_ellipsoids{nullptr};
  const arm_controller::kinematics::ForwardKinematicsOutput::LinkPoseMap*
      current_link_poses{nullptr};
};

struct ObstacleSelectionOutput {
  LocalPlannerSphereObstacleList obstacles;
  std::size_t candidate_count{0};
  std::size_t cell_count{0};
  double obstacle_radius{0.0};
  double voxel_radius{0.0};
  double padding{0.0};
  double selection_radius{0.0};
};

class ReactiveTaskObstacleSelector {
public:
  ObstacleSelectionOutput select(const ObstacleSelectionInput& input) const;
};

}  // namespace arm_controller::controller::reactive_task
