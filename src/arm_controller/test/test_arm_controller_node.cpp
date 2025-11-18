#include <gtest/gtest.h>
#include "controller_node.hpp" 
#include "arm_controller/controller_base/mode_controller_base.hpp"

// Mock class for ModeControllerBase
// class MockModeControllerBase : public ModeControllerBase {
// public:
//     MOCK_METHOD(void, start, (), (override));
//     MOCK_METHOD(void, stop, (), (override));
// };

// // Test fixture for ControllerNode
// class ControllerNodeTest : public ::testing::Test {
// protected:
//     ControllerNode controller_node;

//     void SetUp() override {
//         // Initialize with some mock controllers
//         MockModeControllerBase* mock_controller1 = new MockModeControllerBase();
//         MockModeControllerBase* mock_controller2 = new MockModeControllerBase();

//         controller_node.controller_map_[ModeControllerBase::ControllerMode::MODE_1] = std::shared_ptr<ModeControllerBase>(mock_controller1);
//         controller_node.controller_map_[ModeControllerBase::ControllerMode::MODE_2] = std::shared_ptr<ModeControllerBase>(mock_controller2);

//         controller_node.work_to_controller_map_[ModeControllerBase::WorkMode::WORK_MODE_1] = ModeControllerBase::ControllerMode::MODE_1;
//         controller_node.work_to_controller_map_[ModeControllerBase::WorkMode::WORK_MODE_2] = ModeControllerBase::ControllerMode::MODE_2;
//     }
// };

// // Test case 1: Start a valid controller mode
// TEST_F(ControllerNodeTest, StartValidControllerMode) {
//     EXPECT_CALL(*controller_node.controller_map_[ModeControllerBase::ControllerMode::MODE_1], start()).Times(1);

//     bool result = controller_node.start_working_controller(static_cast<uint8_t>(ModeControllerBase::WorkMode::WORK_MODE_1));
//     EXPECT_TRUE(result);
// }

// // Test case 2: Start an invalid controller mode
// TEST_F(ControllerNodeTest, StartInvalidControllerMode) {
//     bool result = controller_node.start_working_controller(99); // Assuming 99 is an invalid mode
//     EXPECT_FALSE(result);
// }

// // Test case 3: Start a controller mode that is already active
// TEST_F(ControllerNodeTest, StartAlreadyActiveControllerMode) {
//     controller_node.current_mode_ = static_cast<uint8_t>(ModeControllerBase::WorkMode::WORK_MODE_1);

//     bool result = controller_node.start_working_controller(static_cast<uint8_t>(ModeControllerBase::WorkMode::WORK_MODE_1));
//     EXPECT_TRUE(result);
// }

// // Test case 4: Stop a controller mode that is active
// TEST_F(ControllerNodeTest, StopActiveControllerMode) {
//     controller_node.current_mode_ = static_cast<uint8_t>(ModeControllerBase::WorkMode::WORK_MODE_1);
//     EXPECT_CALL(*controller_node.controller_map_[ModeControllerBase::ControllerMode::MODE_1], stop()).Times(1);

//     controller_node.stop_working_controller();
// }

// // Test case 5: Stop a controller mode that is not active
// TEST_F(ControllerNodeTest, StopInactiveControllerMode) {
//     controller_node.current_mode_ = static_cast<uint8_t>(ModeControllerBase::WorkMode::WORK_MODE_2);

//     controller_node.stop_working_controller();
// }

// // Test case 6: Start a controller mode with a null controller pointer
// TEST_F(ControllerNodeTest, StartControllerModeWithNullPointer) {
//     controller_node.controller_map_[ModeControllerBase::ControllerMode::MODE_3] = nullptr;
//     controller_node.work_to_controller_map_[ModeControllerBase::WorkMode::WORK_MODE_3] = ModeControllerBase::ControllerMode::MODE_3;

//     bool result = controller_node.start_working_controller(static_cast<uint8_t>(ModeControllerBase::WorkMode::WORK_MODE_3));
//     EXPECT_FALSE(result);
// }
