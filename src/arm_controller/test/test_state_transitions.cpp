// Copyright 2024 Universal Arm Controller Contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <cstring>
#include "arm_controller/ipc/controller_state_manager.hpp"

namespace arm_controller::ipc
{

class StateTransitionTest : public ::testing::Test
{
protected:
  ControllerStateManager manager_{"left_arm"};
};

// Test 1: Initial state should be SystemStart
TEST_F(StateTransitionTest, InitializeToSystemStart) {
  manager_.initializeCurrentMode("SystemStart");

  EXPECT_EQ(manager_.getCurrentMode(), "SystemStart");
  EXPECT_EQ(manager_.getTargetMode(), "SystemStart");
  EXPECT_EQ(manager_.getExecutionState(), ExecutionState::IDLE);
  EXPECT_FALSE(manager_.isInHookState());
}

// Test 2: SystemStart -> MoveJ should NOT require hook state
// (direct transition, no stopping needed)
TEST_F(StateTransitionTest, SystemStartToMoveJ_NoHook) {
  manager_.initializeCurrentMode("SystemStart");

  bool result = manager_.transitionToMode("MoveJ");

  EXPECT_TRUE(result);
  EXPECT_EQ(manager_.getCurrentMode(), "MoveJ");
  EXPECT_EQ(manager_.getTargetMode(), "MoveJ");
  EXPECT_FALSE(manager_.isInHookState());
  EXPECT_EQ(manager_.getExecutionState(), ExecutionState::IDLE);
}

// Test 3: MoveJ -> MoveL should require hook state
// (both are motion modes, need to stop before transitioning)
TEST_F(StateTransitionTest, MoveJToMoveL_WithHook) {
  manager_.initializeCurrentMode("SystemStart");
  manager_.transitionToMode("MoveJ");

  // Transition to MoveL should enter hook state
  bool result = manager_.transitionToMode("MoveL");

  EXPECT_TRUE(result);
  EXPECT_EQ(manager_.getCurrentMode(), "MoveJ");    // Still in MoveJ
  EXPECT_EQ(manager_.getTargetMode(), "MoveL");     // Target is MoveL
  EXPECT_TRUE(manager_.isInHookState());             // In hook state
  EXPECT_EQ(manager_.getExecutionState(), ExecutionState::IDLE);
}

// Test 4: While in hook state, target mode can be changed
TEST_F(StateTransitionTest, UpdateTargetInHookState) {
  manager_.initializeCurrentMode("SystemStart");
  manager_.transitionToMode("MoveJ");
  manager_.transitionToMode("MoveL");    // Enter hook state

  // While in hook, transition to MoveC should update target
  bool result = manager_.transitionToMode("MoveC");

  EXPECT_TRUE(result);
  EXPECT_EQ(manager_.getCurrentMode(), "MoveJ");
  EXPECT_EQ(manager_.getTargetMode(), "MoveC");
  EXPECT_TRUE(manager_.isInHookState());
}

// Test 5: Exit hook state when executor reports mode change
TEST_F(StateTransitionTest, ExitHookStateOnExecutorUpdate) {
  manager_.initializeCurrentMode("SystemStart");
  manager_.transitionToMode("MoveJ");
  manager_.transitionToMode("MoveL");    // Enter hook state

  // Simulate executor returning from Holding back to MoveL
  ExecutorControllerState executor_state;
  std::strncpy(executor_state.current_mode, "MoveL", sizeof(executor_state.current_mode) - 1);
  executor_state.execution_state = (int32_t)ExecutionState::IDLE;

  manager_.updateFromExecutor(executor_state);

  EXPECT_FALSE(manager_.isInHookState());
  EXPECT_EQ(manager_.getCurrentMode(), "MoveL");
  EXPECT_EQ(manager_.getExecutionState(), ExecutionState::IDLE);
}

// Test 6: Same mode transition should be no-op
TEST_F(StateTransitionTest, SameModeTransition) {
  manager_.initializeCurrentMode("SystemStart");
  manager_.transitionToMode("MoveJ");

  bool result = manager_.transitionToMode("MoveJ");

  EXPECT_TRUE(result);
  EXPECT_EQ(manager_.getCurrentMode(), "MoveJ");
  EXPECT_FALSE(manager_.isInHookState());
}

// Test 7: Setting execution state should update state
TEST_F(StateTransitionTest, SetExecutionState) {
  manager_.initializeCurrentMode("SystemStart");

  manager_.setExecutionState(ExecutionState::PENDING);
  EXPECT_EQ(manager_.getExecutionState(), ExecutionState::PENDING);

  manager_.setExecutionState(ExecutionState::EXECUTING);
  EXPECT_EQ(manager_.getExecutionState(), ExecutionState::EXECUTING);

  manager_.setExecutionState(ExecutionState::SUCCESS);
  EXPECT_EQ(manager_.getExecutionState(), ExecutionState::SUCCESS);
}

// Test 8: Full workflow - SystemStart -> MoveJ -> MoveL with hook -> back to MoveL
TEST_F(StateTransitionTest, FullWorkflow) {
  // Initialize at SystemStart
  manager_.initializeCurrentMode("SystemStart");
  EXPECT_EQ(manager_.getCurrentMode(), "SystemStart");

  // Transition to MoveJ (no hook needed)
  manager_.transitionToMode("MoveJ");
  EXPECT_EQ(manager_.getCurrentMode(), "MoveJ");
  EXPECT_FALSE(manager_.isInHookState());

  // Set execution state to executing
  manager_.setExecutionState(ExecutionState::EXECUTING);
  EXPECT_EQ(manager_.getExecutionState(), ExecutionState::EXECUTING);

  // Try to transition to MoveL (should enter hook)
  manager_.transitionToMode("MoveL");
  EXPECT_EQ(manager_.getCurrentMode(), "MoveJ");    // Still in MoveJ
  EXPECT_EQ(manager_.getTargetMode(), "MoveL");     // Target is MoveL
  EXPECT_TRUE(manager_.isInHookState());             // In hook state

  // Simulate executor stopping and transitioning to MoveL
  ExecutorControllerState executor_state;
  std::strncpy(executor_state.current_mode, "MoveL", sizeof(executor_state.current_mode) - 1);
  executor_state.execution_state = (int32_t)ExecutionState::IDLE;

  manager_.updateFromExecutor(executor_state);

  // Should exit hook and be in MoveL now
  EXPECT_FALSE(manager_.isInHookState());
  EXPECT_EQ(manager_.getCurrentMode(), "MoveL");
  EXPECT_EQ(manager_.getExecutionState(), ExecutionState::IDLE);
}

}  // namespace arm_controller::ipc
