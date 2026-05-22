#pragma once

#include <Eigen/Core>

#include <chrono>
#include <future>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "controller/reactive_task/local_planner/reactive_task_local_planner.hpp"
#include "controller/reactive_task/local_planner/reactive_task_local_planner_runtime.hpp"

namespace arm_controller::controller::reactive_task {

class AsyncLocalPlannerRunner {
public:
  struct ApplyReadyInput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string mapping;
    rclcpp::Logger logger{rclcpp::get_logger("async_local_planner_runner")};
    rclcpp::Clock::SharedPtr clock;
    Eigen::VectorXd current_qdot_reference;
    bool current_qdot_reference_valid{false};
    double now_sec{0.0};
    double max_usable_age_sec{0.12};
  };

  struct ApplyReadyResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool had_ready_output{false};
    bool accepted{false};
    bool stale_generation{false};
    bool planner_failed{false};
    Eigen::Matrix<double, 6, 1> nominal_twist{
        Eigen::Matrix<double, 6, 1>::Zero()};
  };

  bool pending() const { return pending_valid_; }
  int generation() const { return generation_; }
  void advanceGeneration() { ++generation_; }
  bool outputCurrent() const { return pending_generation_ == generation_; }
  void handleTerminalTracking(LocalPlannerRuntime* runtime,
                              bool entered_terminal_tracking,
                              const std::string& mapping,
                              rclcpp::Logger logger);

  bool dueForRequest(const LocalPlannerRuntime& runtime,
                     double now_sec,
                     double period_sec) const;

  void start(const LocalReferencePlanner& planner,
             LocalReferencePlannerInput&& input,
             int tick,
             double request_time_sec);

  void dropReady(const std::string& mapping,
                 rclcpp::Logger logger,
                 const char* reason);

  ApplyReadyResult applyReady(LocalPlannerRuntime* runtime,
                              ApplyReadyInput input);

private:
  bool pendingReady() const;
  bool takeReadyOutput(ArmLocalPlanner::Output* output);

  std::future<ArmLocalPlanner::Output> pending_future_;
  bool pending_valid_{false};
  double pending_request_time_sec_{-std::numeric_limits<double>::infinity()};
  int pending_tick_{0};
  int generation_{0};
  int pending_generation_{0};
  Eigen::VectorXd pending_q_start_;
  Eigen::VectorXd pending_q_goal_;
};

}  // namespace arm_controller::controller::reactive_task
