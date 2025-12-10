#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "arm_controller/controller_base/velocity_controller_base.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

// Mock implementation for testing velocity controller subscriptions
class MockVelocityController : public VelocityControllerImpl<sensor_msgs::msg::JointState>
{
public:
  MockVelocityController(const rclcpp::Node::SharedPtr & node)
  : VelocityControllerImpl<sensor_msgs::msg::JointState>("MockVelocity", node) {}

  void velocity_callback(const sensor_msgs::msg::JointState::SharedPtr) override {}
  void start(const std::string &) override {}
  bool stop(const std::string &) override {return true;}

  auto & get_subscriptions() {return subscriptions_;}
  using VelocityControllerImpl<sensor_msgs::msg::JointState>::cleanup_subscriptions;
};

class VelocityControllerSubscriptionsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node = std::make_shared<rclcpp::Node>("test_velocity_node");
  }

  void TearDown() override
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

// ============================================================================
// Test: Velocity controller creates subscriptions
// ============================================================================
TEST_F(VelocityControllerSubscriptionsTest, CreatesSubscriptions) {
  auto controller = std::make_shared<MockVelocityController>(node);
  node->declare_parameter("controllers.MockVelocity.input_topic", "/velocity/{mapping}");

  EXPECT_TRUE(controller->get_subscriptions().empty());

  controller->init_subscriptions("arm");
  EXPECT_EQ(controller->get_subscriptions().size(), 1);
  EXPECT_TRUE(controller->get_subscriptions().count("arm") > 0);
}

// ============================================================================
// Test: Multiple velocity controller subscriptions
// ============================================================================
TEST_F(VelocityControllerSubscriptionsTest, MultipleSubscriptions) {
  auto controller = std::make_shared<MockVelocityController>(node);
  node->declare_parameter("controllers.MockVelocity.input_topic", "/vel_cmd/{mapping}");

  std::vector<std::string> mappings = {"left", "right", "center"};
  for (const auto & m : mappings) {
    controller->init_subscriptions(m);
  }

  EXPECT_EQ(controller->get_subscriptions().size(), 3);
  for (const auto & m : mappings) {
    EXPECT_TRUE(controller->get_subscriptions().count(m) > 0);
  }
}

// ============================================================================
// Test: Velocity controller cleanup
// ============================================================================
TEST_F(VelocityControllerSubscriptionsTest, CleanupSubscriptions) {
  auto controller = std::make_shared<MockVelocityController>(node);
  node->declare_parameter("controllers.MockVelocity.input_topic", "/vel/{mapping}");

  controller->init_subscriptions("arm");
  EXPECT_EQ(controller->get_subscriptions().size(), 1);

  controller->cleanup_subscriptions("arm");
  EXPECT_TRUE(controller->get_subscriptions().empty());
}

// ============================================================================
// Test: Velocity controller partial cleanup
// ============================================================================
TEST_F(VelocityControllerSubscriptionsTest, PartialCleanup) {
  auto controller = std::make_shared<MockVelocityController>(node);
  node->declare_parameter("controllers.MockVelocity.input_topic", "/vel/{mapping}");

  controller->init_subscriptions("base");
  controller->init_subscriptions("arm");
  controller->init_subscriptions("gripper");

  EXPECT_EQ(controller->get_subscriptions().size(), 3);

  controller->cleanup_subscriptions("arm");

  EXPECT_EQ(controller->get_subscriptions().size(), 2);
  EXPECT_TRUE(controller->get_subscriptions().count("base") > 0);
  EXPECT_TRUE(controller->get_subscriptions().count("gripper") > 0);
  EXPECT_TRUE(controller->get_subscriptions().count("arm") == 0);
}

// ============================================================================
// Test: Velocity controller missing parameter
// ============================================================================
TEST_F(VelocityControllerSubscriptionsTest, MissingInputTopicParameter) {
  auto controller = std::make_shared<MockVelocityController>(node);
  // Don't declare parameter

  EXPECT_NO_THROW(controller->init_subscriptions("arm"));
  EXPECT_TRUE(controller->get_subscriptions().empty());
}

// ============================================================================
// Test: Velocity controller complex mapping names
// ============================================================================
TEST_F(VelocityControllerSubscriptionsTest, ComplexMappingNames) {
  auto controller = std::make_shared<MockVelocityController>(node);
  node->declare_parameter("controllers.MockVelocity.input_topic", "/api/{mapping}/vel");

  std::vector<std::string> mappings = {
    "joint_1",
    "motor_left_shoulder",
    "axis_xyz"
  };

  for (const auto & m : mappings) {
    controller->init_subscriptions(m);
  }

  EXPECT_EQ(controller->get_subscriptions().size(), 3);
}

// ============================================================================
// Test: Velocity controller topic name with suffix after mapping
// ============================================================================
TEST_F(VelocityControllerSubscriptionsTest, MappingWithSuffixAndPrefix) {
  auto controller = std::make_shared<MockVelocityController>(node);
  node->declare_parameter(
    "controllers.MockVelocity.input_topic",
    "/vel_cmd_{mapping}_request");

  controller->init_subscriptions("joint");
  EXPECT_TRUE(controller->get_subscriptions().count("joint") > 0);
}

// ============================================================================
// Test: Velocity controller cleanup idempotency
// ============================================================================
TEST_F(VelocityControllerSubscriptionsTest, CleanupIdempotency) {
  auto controller = std::make_shared<MockVelocityController>(node);
  node->declare_parameter("controllers.MockVelocity.input_topic", "/vel/{mapping}");

  controller->init_subscriptions("arm");
  controller->cleanup_subscriptions("arm");

  // Multiple cleanups should not crash
  EXPECT_NO_THROW(controller->cleanup_subscriptions("arm"));
  EXPECT_NO_THROW(controller->cleanup_subscriptions("arm"));
  EXPECT_TRUE(controller->get_subscriptions().empty());
}

// ============================================================================
// Test: Velocity controller reinit after cleanup
// ============================================================================
TEST_F(VelocityControllerSubscriptionsTest, ReinitAfterCleanup) {
  auto controller = std::make_shared<MockVelocityController>(node);
  node->declare_parameter("controllers.MockVelocity.input_topic", "/vel/{mapping}");

  controller->init_subscriptions("motor");
  EXPECT_EQ(controller->get_subscriptions().size(), 1);

  controller->cleanup_subscriptions("motor");
  EXPECT_TRUE(controller->get_subscriptions().empty());

  controller->init_subscriptions("motor");
  EXPECT_EQ(controller->get_subscriptions().size(), 1);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
