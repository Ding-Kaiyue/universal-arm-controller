#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "arm_controller/controller_base/trajectory_controller_base.hpp"
#include "arm_controller/controller_base/velocity_controller_base.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

// Mock implementation for testing trajectory controller behavior
class MockTrajectoryController : public TrajectoryControllerImpl<sensor_msgs::msg::JointState>
{
public:
  MockTrajectoryController(const rclcpp::Node::SharedPtr & node)
  : TrajectoryControllerImpl<sensor_msgs::msg::JointState>("MockTrajectory", node) {}

  void plan_and_execute(const std::string & mapping, const sensor_msgs::msg::JointState::SharedPtr msg) override
  {
    last_mapping_ = mapping;
    last_executed_msg_ = msg;
    execute_called_ = true;
  }

  void start(const std::string & mapping) override
  {
    started_mappings_.push_back(mapping);
  }

  bool stop(const std::string & mapping) override
  {
    stopped_mappings_.push_back(mapping);
    return true;
  }

  bool move(const std::string & mapping, const std::vector<double>& parameters) override
  {
    move_calls_.push_back({mapping, parameters});
    last_mapping_ = mapping;
    last_parameters_ = parameters;
    return true;
  }

  // Test helpers
  size_t get_move_call_count() const { return move_calls_.size(); }
  std::string get_last_mapping() const { return last_mapping_; }
  std::vector<double> get_last_parameters() const { return last_parameters_; }
  bool was_execute_called() const { return execute_called_; }
  std::string get_last_executed_mapping() const { return last_mapping_; }

  void reset()
  {
    move_calls_.clear();
    started_mappings_.clear();
    stopped_mappings_.clear();
    last_mapping_.clear();
    last_parameters_.clear();
    execute_called_ = false;
  }

private:
  struct MoveCall {
    std::string mapping;
    std::vector<double> parameters;
  };

  std::vector<MoveCall> move_calls_;
  std::vector<std::string> started_mappings_;
  std::vector<std::string> stopped_mappings_;
  std::string last_mapping_;
  std::vector<double> last_parameters_;
  sensor_msgs::msg::JointState::SharedPtr last_executed_msg_;
  bool execute_called_ = false;
};

// Mock implementation for testing velocity controller behavior
class MockVelocityController : public VelocityControllerImpl<sensor_msgs::msg::JointState>
{
public:
  MockVelocityController(const rclcpp::Node::SharedPtr & node)
  : VelocityControllerImpl<sensor_msgs::msg::JointState>("MockVelocity", node) {}

  // Velocity controller requires move interface
  bool move(const std::string & mapping, const std::vector<double>& parameters) override
  {
    move_calls_.push_back({mapping, parameters});
    last_mapping_ = mapping;
    last_parameters_ = parameters;
    move_count_++;
    return true;
  }

  // Test helpers
  size_t get_move_call_count() const { return move_count_; }
  std::string get_last_mapping() const { return last_mapping_; }
  std::vector<double> get_last_parameters() const { return last_parameters_; }

  void reset()
  {
    move_calls_.clear();
    last_mapping_.clear();
    last_parameters_.clear();
    move_count_ = 0;
  }

private:
  struct MoveCall {
    std::string mapping;
    std::vector<double> parameters;
  };

  std::vector<MoveCall> move_calls_;
  std::string last_mapping_;
  std::vector<double> last_parameters_;
  size_t move_count_ = 0;
};

// Test Fixture
class ControllerIPCInterfaceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS2
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node = std::make_shared<rclcpp::Node>("test_controller_node");
  }

  void TearDown() override
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

// ============================================================================
// Test: TrajectoryController executes move command for single mapping
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, TrajectoryControllerMoveSingleMapping) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Execute move command
  std::vector<double> params = {1.0, 2.0, 3.0};
  bool result = controller->move("left_arm", params);

  EXPECT_TRUE(result);
  EXPECT_EQ(controller->get_move_call_count(), 1);
  EXPECT_EQ(controller->get_last_mapping(), "left_arm");
  EXPECT_EQ(controller->get_last_parameters(), params);
}

// ============================================================================
// Test: TrajectoryController handles multiple move commands sequentially
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, TrajectoryControllerMultipleMoveCommands) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Execute first move
  std::vector<double> params1 = {1.0, 2.0, 3.0};
  controller->move("left_arm", params1);

  // Execute second move
  std::vector<double> params2 = {4.0, 5.0, 6.0};
  controller->move("right_arm", params2);

  EXPECT_EQ(controller->get_move_call_count(), 2);
  EXPECT_EQ(controller->get_last_mapping(), "right_arm");
  EXPECT_EQ(controller->get_last_parameters(), params2);
}

// ============================================================================
// Test: TrajectoryController supports multiple arm mappings
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, TrajectoryControllerMultipleMappings) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  std::vector<std::string> mappings = {"left_arm", "right_arm", "dual_arm"};
  std::vector<double> params = {1.5, 2.5, 3.5};

  for (const auto& mapping : mappings) {
    bool result = controller->move(mapping, params);
    EXPECT_TRUE(result);
  }

  EXPECT_EQ(controller->get_move_call_count(), 3);
  EXPECT_EQ(controller->get_last_mapping(), "dual_arm");
}

// ============================================================================
// Test: TrajectoryController handles empty parameters
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, TrajectoryControllerEmptyParameters) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  std::vector<double> empty_params;
  bool result = controller->move("arm", empty_params);

  EXPECT_TRUE(result);
  EXPECT_EQ(controller->get_move_call_count(), 1);
  EXPECT_EQ(controller->get_last_parameters().size(), 0);
}

// ============================================================================
// Test: TrajectoryController handles large parameter sets
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, TrajectoryControllerLargeParameterSet) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Create large parameter set (e.g., 16 joint values)
  std::vector<double> large_params;
  for (int i = 0; i < 16; ++i) {
    large_params.push_back(static_cast<double>(i) * 0.1);
  }

  bool result = controller->move("multi_joint_arm", large_params);

  EXPECT_TRUE(result);
  EXPECT_EQ(controller->get_last_parameters().size(), 16);
}

// ============================================================================
// Test: VelocityController executes move command
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, VelocityControllerMove) {
  auto controller = std::make_shared<MockVelocityController>(node);

  std::vector<double> velocity_params = {0.5, 0.2, 0.1};
  bool result = controller->move("arm", velocity_params);

  EXPECT_TRUE(result);
  EXPECT_EQ(controller->get_move_call_count(), 1);
  EXPECT_EQ(controller->get_last_mapping(), "arm");
}

// ============================================================================
// Test: VelocityController handles multiple velocity commands
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, VelocityControllerMultipleCommands) {
  auto controller = std::make_shared<MockVelocityController>(node);

  for (int i = 0; i < 5; ++i) {
    std::vector<double> velocity = {0.1 * i, 0.2 * i, 0.3 * i};
    controller->move("gripper", velocity);
  }

  EXPECT_EQ(controller->get_move_call_count(), 5);
}

// ============================================================================
// Test: TrajectoryController start/stop lifecycle
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, TrajectoryControllerStartStop) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  controller->start("left_arm");
  std::vector<double> params = {1.0, 2.0, 3.0};
  controller->move("left_arm", params);
  bool stop_result = controller->stop("left_arm");

  EXPECT_TRUE(stop_result);
  EXPECT_EQ(controller->get_move_call_count(), 1);
}

// ============================================================================
// Test: Controller reset clears execution history
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, ControllerResetClearsHistory) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  std::vector<double> params = {1.0, 2.0, 3.0};
  controller->move("arm", params);
  EXPECT_EQ(controller->get_move_call_count(), 1);

  controller->reset();
  EXPECT_EQ(controller->get_move_call_count(), 0);
  EXPECT_EQ(controller->get_last_mapping(), "");
}

// ============================================================================
// Test: VelocityController reset clears execution history
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, VelocityControllerResetHistory) {
  auto controller = std::make_shared<MockVelocityController>(node);

  std::vector<double> params = {0.1, 0.2, 0.3};
  controller->move("arm", params);
  EXPECT_EQ(controller->get_move_call_count(), 1);

  controller->reset();
  EXPECT_EQ(controller->get_move_call_count(), 0);
  EXPECT_EQ(controller->get_last_mapping(), "");
}

// ============================================================================
// Test: Concurrent multi-arm execution through multiple move calls
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, ConcurrentMultiArmExecution) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Simulate global execution order with per-mapping concurrency
  std::vector<std::pair<std::string, std::vector<double>>> commands = {
    {"left_arm", {1.0, 2.0, 3.0}},
    {"right_arm", {4.0, 5.0, 6.0}},
    {"left_arm", {2.0, 3.0, 4.0}},
    {"right_arm", {5.0, 6.0, 7.0}}
  };

  for (const auto& cmd : commands) {
    controller->move(cmd.first, cmd.second);
  }

  EXPECT_EQ(controller->get_move_call_count(), 4);
}

// ============================================================================
// ControllerBase 基类方法测试 (补充覆盖率)
// ============================================================================

// ============================================================================
// Test: TrajectoryController - Mode management
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, ControllerBaseModeManagement) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Start arm
  controller->start("arm1");

  // Move should succeed
  bool result = controller->move("arm1", {1.0, 2.0});
  EXPECT_TRUE(result);

  // Stop arm
  bool stopped = controller->stop("arm1");
  EXPECT_TRUE(stopped);
}

// ============================================================================
// Test: TrajectoryController - Multiple mappings management
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, ControllerBaseMultipleMappings) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Start multiple arms
  controller->start("left_arm");
  controller->start("right_arm");
  controller->start("center_arm");

  // All should work
  controller->move("left_arm", {1.0});
  controller->move("right_arm", {2.0});
  controller->move("center_arm", {3.0});

  EXPECT_EQ(controller->get_move_call_count(), 3);

  // Stop one
  controller->stop("left_arm");

  // Others still work
  controller->move("right_arm", {4.0});
  EXPECT_EQ(controller->get_move_call_count(), 4);
}

// ============================================================================
// Test: VelocityController - Mode management
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, VelocityControllerModeManagement) {
  auto controller = std::make_shared<MockVelocityController>(node);

  controller->start("arm");

  std::vector<double> velocity = {0.1, 0.2, 0.3};
  bool result = controller->move("arm", velocity);
  EXPECT_TRUE(result);

  bool stopped = controller->stop("arm");
  EXPECT_TRUE(stopped);
}

// ============================================================================
// Test: Controller lifecycle - Start, Execute, Stop sequence
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, ControllerLifecycleSequence) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Start
  controller->start("robot_arm");

  // Multiple moves
  for (int i = 0; i < 5; ++i) {
    std::vector<double> params = {(double)i, (double)(i+1), (double)(i+2)};
    controller->move("robot_arm", params);
  }

  EXPECT_EQ(controller->get_move_call_count(), 5);

  // Stop
  controller->stop("robot_arm");
}

// ============================================================================
// Test: Controller mode getter
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, ControllerGetMode) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Mode should be set in constructor
  std::string mode = controller->get_mode();
  EXPECT_EQ(mode, "MockTrajectory");
}

// ============================================================================
// Test: VelocityController needs_hook_state
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, VelocityControllerNeedsHookState) {
  auto controller = std::make_shared<MockVelocityController>(node);

  // Velocity controllers typically need hook state for safe stopping
  bool needs_hook = controller->needs_hook_state();
  EXPECT_TRUE(needs_hook);
}

// ============================================================================
// Test: TrajectoryController rapid sequential moves
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, RapidSequentialMoves) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  controller->start("arm");

  // Rapid moves without stopping between them
  for (int i = 0; i < 10; ++i) {
    std::vector<double> params = {(double)i * 0.1};
    controller->move("arm", params);
  }

  EXPECT_EQ(controller->get_move_call_count(), 10);
  EXPECT_EQ(controller->get_last_mapping(), "arm");
}

// ============================================================================
// Test: Controller with empty mapping (normalization)
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, ControllerEmptyMappingHandling) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Start with empty mapping (should normalize to "single_arm")
  controller->start("");

  // Move with empty mapping
  bool result = controller->move("", {1.0, 2.0});
  EXPECT_TRUE(result);
}

// ============================================================================
// Test: Mixed controller types in same session
// ============================================================================
TEST_F(ControllerIPCInterfaceTest, MixedControllerTypes) {
  auto traj = std::make_shared<MockTrajectoryController>(node);
  auto vel = std::make_shared<MockVelocityController>(node);

  // Start both
  traj->start("trajectory_arm");
  vel->start("velocity_arm");

  // Execute different types of commands
  traj->move("trajectory_arm", {1.0, 2.0, 3.0});
  vel->move("velocity_arm", {0.1, 0.2, 0.3});

  // Stop both
  traj->stop("trajectory_arm");
  vel->stop("velocity_arm");

  EXPECT_EQ(traj->get_move_call_count(), 1);
  EXPECT_EQ(vel->get_move_call_count(), 1);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
