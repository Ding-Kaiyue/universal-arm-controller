#pragma once

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "algorithm/cartesian_path_planner/types.hpp"

namespace arm_controller::controller::reactive_task {

namespace cp = arm_controller::algorithm::cartesian_path_planner;

using LocalPlannerJointVectorList =
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>>;

struct LocalReferencePlannerConfig {
  int horizon_steps{10};
  double dt_sec{0.05};
  double update_period_sec{0.05};
  bool enable_collision_cost{true};
  bool enable_collision_constraint{true};
  int max_obstacle_spheres{32};
  double obstacle_selection_radius_m{0.25};
  double obstacle_padding_m{0.0};
};

struct LocalPlannerSphereObstacle {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::string name;
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double radius{0.0};
};
using LocalPlannerSphereObstacleList =
    std::vector<LocalPlannerSphereObstacle,
                Eigen::aligned_allocator<LocalPlannerSphereObstacle>>;

struct LocalReferencePlannerOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool ok{false};
  bool used{false};
  Eigen::Isometry3d target_pose{Eigen::Isometry3d::Identity()};
  Eigen::Matrix<double, 6, 1> target_twist{
      Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> nominal_twist{
      Eigen::Matrix<double, 6, 1>::Zero()};
  int sampled_steps{0};
  std::string error;
  bool has_optimized_joint_target{false};
  Eigen::VectorXd optimized_joint_target;
  Eigen::VectorXd optimized_joint_velocity;
  double optimized_joint_target_time_sec{0.0};
  LocalPlannerJointVectorList optimized_joint_trajectory;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
      target_poses;
  std::vector<
      Eigen::Matrix<double, 6, 1>,
      Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>>
      target_twists;
  double trajectory_dt_sec{0.0};
};

struct LocalReferencePlannerInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using JointToPoseFn =
      std::function<bool(const Eigen::VectorXd &, Eigen::Isometry3d *)>;

  Eigen::Isometry3d current_pose{Eigen::Isometry3d::Identity()};
  cp::TimedCartesianSampleList reference_samples;
  bool reference_finished{false};
  std::vector<std::string> joint_names;
  Eigen::VectorXd q_current;
  Eigen::VectorXd q_goal;
  bool q_goal_valid{false};
  JointToPoseFn joint_to_pose;
  std::string robot_type;
  std::string planning_group;
  std::string base_link;
  std::string tip_link;
  std::string urdf_path;
  std::string srdf_path;
  LocalPlannerSphereObstacleList sphere_obstacles;
};

class LocalReferencePlanner {
public:
  virtual ~LocalReferencePlanner() = default;

  virtual bool compute(const LocalReferencePlannerInput& input,
                       LocalReferencePlannerOutput* output) const = 0;
};

class ArmLocalPlanner final : public LocalReferencePlanner {
public:
  using Config = LocalReferencePlannerConfig;
  using SphereObstacle = LocalPlannerSphereObstacle;
  using SphereObstacleList = LocalPlannerSphereObstacleList;
  using JointVectorList = LocalPlannerJointVectorList;
  using Input = LocalReferencePlannerInput;
  using Output = LocalReferencePlannerOutput;

  ArmLocalPlanner();
  explicit ArmLocalPlanner(Config cfg);

  void configure(Config cfg);
  bool compute(const Input &input, Output *output) const override;

private:
  std::shared_ptr<const void>
  getOrCreateTesseractEnvironment(const Input &input, std::string *error) const;
  std::string makeEnvironmentCacheKey(const Input &input) const;

  Config cfg_;
  mutable std::mutex tesseract_env_mutex_;
  mutable std::map<std::string, std::shared_ptr<const void>>
      tesseract_env_cache_;
};

} // namespace arm_controller::controller::reactive_task
