// Copyright 2025 Universal Arm Controller Contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <vector>
#include "arm_controller/ipc/command_queue_ipc.hpp"
#include "arm_controller/ipc/ipc_types.hpp"
#include "arm_controller/ipc/command_producer.hpp"

class IPCCommandQueueTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // Initialize IPC only once for all tests
    auto& queue = arm_controller::CommandQueueIPC::getInstance();
    queue.initialize();
  }

  void SetUp() override
  {
    // No per-test setup needed
  }

  void TearDown() override
  {
    // Cleanup handled by singleton
  }
};

// ============================================================================
// Test: TrajectoryCommandIPC basic structure initialization
// ============================================================================
TEST_F(IPCCommandQueueTest, BasicCommandInitialization)
{
  arm_controller::TrajectoryCommandIPC cmd;

  // Verify default initialization
  EXPECT_EQ(cmd.param_count, 0);
  EXPECT_STREQ(cmd.mode, "");
  EXPECT_STREQ(cmd.mapping, "");
  EXPECT_STREQ(cmd.command_id, "");
}

// ============================================================================
// Test: Set and get mode
// ============================================================================
TEST_F(IPCCommandQueueTest, SetGetMode)
{
  arm_controller::TrajectoryCommandIPC cmd;

  cmd.set_mode("MoveJ");
  EXPECT_EQ(cmd.get_mode(), "MoveJ");

  cmd.set_mode("MoveL");
  EXPECT_EQ(cmd.get_mode(), "MoveL");

  cmd.set_mode("CartesianVelocity");
  EXPECT_EQ(cmd.get_mode(), "CartesianVelocity");
}

// ============================================================================
// Test: Set and get mapping
// ============================================================================
TEST_F(IPCCommandQueueTest, SetGetMapping)
{
  arm_controller::TrajectoryCommandIPC cmd;

  cmd.set_mapping("left_arm");
  EXPECT_EQ(cmd.get_mapping(), "left_arm");

  cmd.set_mapping("right_arm");
  EXPECT_EQ(cmd.get_mapping(), "right_arm");

  cmd.set_mapping("dual_arm_config");
  EXPECT_EQ(cmd.get_mapping(), "dual_arm_config");
}

// ============================================================================
// Test: Set and get command_id
// ============================================================================
TEST_F(IPCCommandQueueTest, SetGetCommandId)
{
  arm_controller::TrajectoryCommandIPC cmd;

  cmd.set_command_id("trajectory_001");
  EXPECT_EQ(cmd.get_command_id(), "trajectory_001");

  cmd.set_command_id("velocity_command_42");
  EXPECT_EQ(cmd.get_command_id(), "velocity_command_42");
}

// ============================================================================
// Test: Set and get parameters
// ============================================================================
TEST_F(IPCCommandQueueTest, SetGetParameters)
{
  arm_controller::TrajectoryCommandIPC cmd;

  std::vector<double> params = {1.0, 2.0, 3.0, 4.0, 5.0};
  cmd.set_parameters(params);

  EXPECT_EQ(cmd.param_count, 5);
  auto retrieved = cmd.get_parameters();
  EXPECT_EQ(retrieved.size(), 5);
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_DOUBLE_EQ(retrieved[i], params[i]);
  }
}

// ============================================================================
// Test: Parameter count limits
// ============================================================================
TEST_F(IPCCommandQueueTest, ParameterCountLimit)
{
  arm_controller::TrajectoryCommandIPC cmd;

  // Create parameters exceeding MAX_PARAMS (100)
  std::vector<double> large_params;
  for (int i = 0; i < 150; ++i) {
    large_params.push_back(static_cast<double>(i));
  }

  cmd.set_parameters(large_params);

  // Should be capped at MAX_PARAMS
  EXPECT_EQ(cmd.param_count, arm_controller::TrajectoryCommandIPC::MAX_PARAMS);
  auto retrieved = cmd.get_parameters();
  EXPECT_EQ(retrieved.size(), arm_controller::TrajectoryCommandIPC::MAX_PARAMS);
}

// ============================================================================
// Test: Empty parameters
// ============================================================================
TEST_F(IPCCommandQueueTest, EmptyParameters)
{
  arm_controller::TrajectoryCommandIPC cmd;

  std::vector<double> empty_params;
  cmd.set_parameters(empty_params);

  EXPECT_EQ(cmd.param_count, 0);
  auto retrieved = cmd.get_parameters();
  EXPECT_EQ(retrieved.size(), 0);
}

// ============================================================================
// Test: Mode string truncation
// ============================================================================
TEST_F(IPCCommandQueueTest, ModeStringTruncation)
{
  arm_controller::TrajectoryCommandIPC cmd;

  // Create a string longer than MAX_MODE_LEN
  std::string long_mode(100, 'A');
  cmd.set_mode(long_mode);

  // Should be truncated to MAX_MODE_LEN - 1
  EXPECT_LE(std::strlen(cmd.mode), arm_controller::TrajectoryCommandIPC::MAX_MODE_LEN - 1);
}

// ============================================================================
// Test: Mapping string truncation
// ============================================================================
TEST_F(IPCCommandQueueTest, MappingStringTruncation)
{
  arm_controller::TrajectoryCommandIPC cmd;

  // Create a string longer than MAX_MAPPING_LEN
  std::string long_mapping(100, 'B');
  cmd.set_mapping(long_mapping);

  // Should be truncated to MAX_MAPPING_LEN - 1
  EXPECT_LE(std::strlen(cmd.mapping), arm_controller::TrajectoryCommandIPC::MAX_MAPPING_LEN - 1);
}

// ============================================================================
// Test: Command ID string truncation
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandIdStringTruncation)
{
  arm_controller::TrajectoryCommandIPC cmd;

  // Create a string longer than MAX_COMMAND_ID_LEN
  std::string long_id(200, 'C');
  cmd.set_command_id(long_id);

  // Should be truncated to MAX_COMMAND_ID_LEN - 1
  EXPECT_LE(std::strlen(cmd.command_id), arm_controller::TrajectoryCommandIPC::MAX_COMMAND_ID_LEN - 1);
}

// ============================================================================
// Test: Multiple commands with different modes
// ============================================================================
TEST_F(IPCCommandQueueTest, MultipleCommandTypes)
{
  std::vector<arm_controller::TrajectoryCommandIPC> commands;

  std::vector<std::string> modes = {"MoveJ", "MoveL", "MoveC", "JointVelocity", "CartesianVelocity"};
  std::vector<std::string> mappings = {"left_arm", "right_arm", "dual"};

  for (size_t i = 0; i < modes.size(); ++i) {
    arm_controller::TrajectoryCommandIPC cmd;
    cmd.set_mode(modes[i]);
    cmd.set_mapping(mappings[i % mappings.size()]);
    cmd.set_command_id("cmd_" + std::to_string(i));
    commands.push_back(cmd);
  }

  EXPECT_EQ(commands.size(), 5);
  for (size_t i = 0; i < commands.size(); ++i) {
    EXPECT_EQ(commands[i].get_mode(), modes[i]);
    EXPECT_EQ(commands[i].get_mapping(), mappings[i % mappings.size()]);
  }
}

// ============================================================================
// Test: Parameter value preservation
// ============================================================================
TEST_F(IPCCommandQueueTest, ParameterValuePreservation)
{
  arm_controller::TrajectoryCommandIPC cmd;

  std::vector<double> test_params = {1.5, 2.5, 3.5, -1.0, 0.0, 100.5};
  cmd.set_parameters(test_params);

  auto retrieved = cmd.get_parameters();
  EXPECT_EQ(retrieved.size(), test_params.size());

  for (size_t i = 0; i < test_params.size(); ++i) {
    EXPECT_DOUBLE_EQ(retrieved[i], test_params[i]);
  }
}

// ============================================================================
// Test: Maximum joint parameters (16 joints)
// ============================================================================
TEST_F(IPCCommandQueueTest, MaxJointParameters)
{
  arm_controller::TrajectoryCommandIPC cmd;

  std::vector<double> joint_params;
  for (int i = 0; i < 16; ++i) {
    joint_params.push_back(static_cast<double>(i) * 0.1);
  }

  cmd.set_parameters(joint_params);

  EXPECT_EQ(cmd.param_count, 16);
  auto retrieved = cmd.get_parameters();
  EXPECT_EQ(retrieved.size(), 16);
}

// ============================================================================
// Test: CommandQueueIPC singleton initialization
// ============================================================================
TEST_F(IPCCommandQueueTest, SingletonInitialization)
{
  auto& queue1 = arm_controller::CommandQueueIPC::getInstance();
  auto& queue2 = arm_controller::CommandQueueIPC::getInstance();

  // Should return the same instance
  EXPECT_EQ(&queue1, &queue2);
}

// ============================================================================
// Test: IPC initialization and opening
// ============================================================================
TEST_F(IPCCommandQueueTest, InitializeAndOpen)
{
  auto& queue = arm_controller::CommandQueueIPC::getInstance();

  // Initialize creates new shared memory
  bool init_result = queue.initialize();
  EXPECT_TRUE(init_result);

  // Open should connect to existing shared memory
  bool open_result = queue.open();
  EXPECT_TRUE(open_result);
}

// ============================================================================
// Additional IPC Types Tests (improve ipc coverage from 61.8% -> 75%)
// ============================================================================

TEST_F(IPCCommandQueueTest, ShmHeaderValidation) {
  arm_controller::ipc::ShmHeader header;
  EXPECT_TRUE(header.isValid());
  EXPECT_EQ(header.version, arm_controller::ipc::IPC_VERSION);
  EXPECT_EQ(header.magic, 0xDEADBEEF);
}

TEST_F(IPCCommandQueueTest, ShmHeaderInvalidMagic) {
  arm_controller::ipc::ShmHeader header;
  header.magic = 0xDEADC0DE;  // Invalid magic
  EXPECT_FALSE(header.isValid());
}

TEST_F(IPCCommandQueueTest, TrajectoryCommandInitialization) {
  arm_controller::ipc::TrajectoryCommand cmd;
  EXPECT_EQ(cmd.seq, 0);
  EXPECT_EQ(cmd.param_count, 0);
  EXPECT_EQ(cmd.producer_id, 0);
}

TEST_F(IPCCommandQueueTest, TrajectoryCommandSetMode) {
  arm_controller::ipc::TrajectoryCommand cmd;
  cmd.set_mode("MoveJ");
  EXPECT_EQ(cmd.get_mode(), "MoveJ");
}

TEST_F(IPCCommandQueueTest, TrajectoryCommandSetMapping) {
  arm_controller::ipc::TrajectoryCommand cmd;
  cmd.set_mapping("left_arm");
  EXPECT_EQ(cmd.get_mapping(), "left_arm");
}

TEST_F(IPCCommandQueueTest, TrajectoryCommandSetCommandId) {
  arm_controller::ipc::TrajectoryCommand cmd;
  cmd.set_command_id("cmd_001");
  EXPECT_EQ(cmd.get_command_id(), "cmd_001");
}

TEST_F(IPCCommandQueueTest, TrajectoryCommandSetParameters) {
  arm_controller::ipc::TrajectoryCommand cmd;
  std::vector<double> params = {1.0, 2.0, 3.0};
  cmd.set_parameters(params);
  EXPECT_EQ(cmd.param_count, 3);
  auto retrieved = cmd.get_parameters();
  EXPECT_EQ(retrieved.size(), 3);
  EXPECT_DOUBLE_EQ(retrieved[0], 1.0);
}

TEST_F(IPCCommandQueueTest, TrajectoryCommandMaxParametersLimit) {
  arm_controller::ipc::TrajectoryCommand cmd;
  std::vector<double> params;
  for (size_t i = 0; i < 20; ++i) {
    params.push_back((double)i);
  }
  cmd.set_parameters(params);
  EXPECT_LE(cmd.param_count, (int32_t)arm_controller::ipc::MAX_JOINTS);
}

TEST_F(IPCCommandQueueTest, HeartbeatInitialization) {
  arm_controller::ipc::Heartbeat hb;
  EXPECT_EQ(hb.seq, 0);
  EXPECT_EQ(hb.status, 0);
}

TEST_F(IPCCommandQueueTest, TrajectoryCommandStringTruncation) {
  arm_controller::ipc::TrajectoryCommand cmd;
  std::string long_mode(100, 'a');
  cmd.set_mode(long_mode);
  EXPECT_LE(strlen(cmd.mode), arm_controller::ipc::TrajectoryCommand::MAX_MODE_LEN - 1);
}

TEST_F(IPCCommandQueueTest, TrajectoryCommandValidation) {
  arm_controller::ipc::TrajectoryCommand cmd;
  cmd.param_count = 3;
  EXPECT_TRUE(cmd.isValid());
}

// ============================================================================
// Test: CommandValidator - Valid mapping
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandValidatorValidMapping) {
  using namespace arm_controller::ipc;

  auto result = CommandValidator::validateMapping("left_arm");
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.error_message, "");
}

// ============================================================================
// Test: CommandValidator - Empty mapping
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandValidatorEmptyMapping) {
  using namespace arm_controller::ipc;

  auto result = CommandValidator::validateMapping("");
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.error_message.empty());
}

// ============================================================================
// Test: CommandValidator - Valid joint positions
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandValidatorValidJointPositions) {
  using namespace arm_controller::ipc;

  std::vector<double> positions = {0.0, 1.57, -1.57, 0.5, -0.5, 3.14};
  auto result = CommandValidator::validateJointPositions(positions);
  EXPECT_TRUE(result.valid);
}

// ============================================================================
// Test: CommandValidator - Empty joint positions
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandValidatorEmptyJointPositions) {
  using namespace arm_controller::ipc;

  std::vector<double> positions = {};
  auto result = CommandValidator::validateJointPositions(positions);
  EXPECT_FALSE(result.valid);
}

// ============================================================================
// Test: CommandValidator - Valid joint velocities
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandValidatorValidJointVelocities) {
  using namespace arm_controller::ipc;

  std::vector<double> velocities = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  auto result = CommandValidator::validateJointVelocities(velocities);
  EXPECT_TRUE(result.valid);
}

// ============================================================================
// Test: CommandValidator - Valid cartesian velocities
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandValidatorValidCartesianVelocities) {
  using namespace arm_controller::ipc;

  std::vector<double> velocities = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  auto result = CommandValidator::validateCartesianVelocities(velocities);
  EXPECT_TRUE(result.valid);
}

// ============================================================================
// Test: CommandValidator - Valid pose
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandValidatorValidPose) {
  using namespace arm_controller::ipc;

  auto result = CommandValidator::validatePose(
    0.5, 0.5, 0.5,      // x, y, z
    0.0, 0.0, 0.0, 1.0  // qx, qy, qz, qw (identity quaternion)
  );
  EXPECT_TRUE(result.valid);
}

// ============================================================================
// Test: CommandValidator - Valid waypoints
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandValidatorValidWaypoints) {
  using namespace arm_controller::ipc;

  // Waypoints must be multiples of 7 (x,y,z,qx,qy,qz,qw)
  std::vector<double> waypoints = {0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 1.0};  // One pose
  auto result = CommandValidator::validateWaypoints(waypoints);
  EXPECT_TRUE(result.valid);
}

// ============================================================================
// Test: CommandBuilder - Basic construction
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandBuilderConstruction) {
  using namespace arm_controller::ipc;

  CommandBuilder builder;
  auto cmd = builder.build();

  EXPECT_NE(cmd.command_id, nullptr);
  EXPECT_GT(strlen(cmd.command_id), 0);
}

// ============================================================================
// Test: CommandBuilder - With mode
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandBuilderWithMode) {
  using namespace arm_controller::ipc;

  CommandBuilder builder;
  auto cmd = builder.withMode("MoveJ").build();

  EXPECT_EQ(cmd.get_mode(), "MoveJ");
}

// ============================================================================
// Test: CommandBuilder - With mapping
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandBuilderWithMapping) {
  using namespace arm_controller::ipc;

  CommandBuilder builder;
  auto cmd = builder.withMapping("left_arm").build();

  EXPECT_EQ(cmd.get_mapping(), "left_arm");
}

// ============================================================================
// Test: CommandBuilder - With joint positions
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandBuilderWithJointPositions) {
  using namespace arm_controller::ipc;

  std::vector<double> positions = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  CommandBuilder builder;
  auto cmd = builder.withJointPositions(positions).build();

  EXPECT_EQ(cmd.param_count, 6);
  auto params = cmd.get_parameters();
  EXPECT_EQ(params.size(), 6);
}

// ============================================================================
// Test: CommandBuilder - With joint velocities
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandBuilderWithJointVelocities) {
  using namespace arm_controller::ipc;

  std::vector<double> velocities = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  CommandBuilder builder;
  auto cmd = builder.withJointVelocities(velocities).build();

  EXPECT_EQ(cmd.param_count, 6);
}

// ============================================================================
// Test: CommandBuilder - With pose
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandBuilderWithPose) {
  using namespace arm_controller::ipc;

  CommandBuilder builder;
  auto cmd = builder.withPose(
    0.5, 0.5, 0.5,      // x, y, z
    0.0, 0.0, 0.0, 1.0  // qx, qy, qz, qw
  ).build();

  EXPECT_EQ(cmd.param_count, 7);
}

// ============================================================================
// Test: CommandBuilder - With waypoints
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandBuilderWithWaypoints) {
  using namespace arm_controller::ipc;

  std::vector<double> waypoints = {0.0, 1.0, 2.0, 3.0};
  CommandBuilder builder;
  auto cmd = builder.withWaypoints(waypoints).build();

  EXPECT_EQ(cmd.param_count, 4);
}

// ============================================================================
// Test: CommandBuilder - Chaining multiple methods
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandBuilderChaining) {
  using namespace arm_controller::ipc;

  std::vector<double> positions = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  CommandBuilder builder;
  auto cmd = builder
    .withMode("MoveJ")
    .withMapping("left_arm")
    .withJointPositions(positions)
    .build();

  EXPECT_EQ(cmd.get_mode(), "MoveJ");
  EXPECT_EQ(cmd.get_mapping(), "left_arm");
  EXPECT_EQ(cmd.param_count, 6);
}

// ============================================================================
// Test: CommandBuilder - With producer ID
// ============================================================================
TEST_F(IPCCommandQueueTest, CommandBuilderWithProducerId) {
  using namespace arm_controller::ipc;

  CommandBuilder builder(42);
  auto cmd = builder.withMode("MoveL").build();

  // Command should be created with given producer ID context
  EXPECT_EQ(cmd.get_mode(), "MoveL");
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
