#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "arm_controller/controller_base/trajectory_controller_base.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

// Mock implementation for testing subscription lifecycle
class MockTrajectoryController : public TrajectoryControllerImpl<sensor_msgs::msg::JointState>
{
public:
  MockTrajectoryController(const rclcpp::Node::SharedPtr & node)
  : TrajectoryControllerImpl<sensor_msgs::msg::JointState>("MockTrajectory", node) {}

  void plan_and_execute(const std::string &, const sensor_msgs::msg::JointState::SharedPtr) override
  {
  }
  void start(const std::string &) override {}
  bool stop(const std::string &) override {return true;}

  auto & get_subscriptions() {return subscriptions_;}
  using TrajectoryControllerImpl<sensor_msgs::msg::JointState>::cleanup_subscriptions;
};

class TopicSubscriptionLifecycleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node = std::make_shared<rclcpp::Node>("test_subscription_node");
  }

  void TearDown() override
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

// ============================================================================
// Test: Subscription creation and cleanup lifecycle
// ============================================================================
TEST_F(TopicSubscriptionLifecycleTest, SubscriptionCreationAndCleanup) {
  auto controller = std::make_shared<MockTrajectoryController>(node);
  node->declare_parameter("controllers.MockTrajectory.input_topic", "/test_topic/{mapping}");

  EXPECT_TRUE(controller->get_subscriptions().empty());

  // Create subscription
  controller->init_subscriptions("test_arm");
  EXPECT_EQ(controller->get_subscriptions().size(), 1);
  EXPECT_TRUE(controller->get_subscriptions().count("test_arm") > 0);

  // Clean up
  controller->cleanup_subscriptions("test_arm");
  EXPECT_TRUE(controller->get_subscriptions().empty());
}

// ============================================================================
// Test: Multiple independent subscriptions
// ============================================================================
TEST_F(TopicSubscriptionLifecycleTest, MultipleIndependentSubscriptions) {
  auto controller = std::make_shared<MockTrajectoryController>(node);
  node->declare_parameter("controllers.MockTrajectory.input_topic", "/test/{mapping}");

  // Create subscriptions for different mappings
  std::vector<std::string> mappings = {"left_arm", "right_arm", "base"};
  for (const auto & mapping : mappings) {
    controller->init_subscriptions(mapping);
  }

  EXPECT_EQ(controller->get_subscriptions().size(), 3);

  // Verify all exist
  for (const auto & mapping : mappings) {
    EXPECT_TRUE(controller->get_subscriptions().count(mapping) > 0);
  }

  // Clean up one
  controller->cleanup_subscriptions("left_arm");
  EXPECT_EQ(controller->get_subscriptions().size(), 2);
  EXPECT_TRUE(controller->get_subscriptions().count("left_arm") == 0);
  EXPECT_TRUE(controller->get_subscriptions().count("right_arm") > 0);
  EXPECT_TRUE(controller->get_subscriptions().count("base") > 0);
}

// ============================================================================
// Test: Mapping placeholder substitution in topic names
// ============================================================================
TEST_F(TopicSubscriptionLifecycleTest, MappingPlaceholderReplacement) {
  auto controller = std::make_shared<MockTrajectoryController>(node);
  node->declare_parameter(
    "controllers.MockTrajectory.input_topic",
    "/controller_api/command/{mapping}/request");

  controller->init_subscriptions("single_arm");
  controller->init_subscriptions("dual_arm");

  EXPECT_EQ(controller->get_subscriptions().size(), 2);
  EXPECT_TRUE(controller->get_subscriptions().count("single_arm") > 0);
  EXPECT_TRUE(controller->get_subscriptions().count("dual_arm") > 0);
}

// ============================================================================
// Test: Reinitialization after cleanup
// ============================================================================
TEST_F(TopicSubscriptionLifecycleTest, ReinitializationAfterCleanup) {
  auto controller = std::make_shared<MockTrajectoryController>(node);
  node->declare_parameter("controllers.MockTrajectory.input_topic", "/cmd/{mapping}");

  // First cycle
  controller->init_subscriptions("arm");
  EXPECT_TRUE(controller->get_subscriptions().count("arm") > 0);

  // Clean up
  controller->cleanup_subscriptions("arm");
  EXPECT_TRUE(controller->get_subscriptions().empty());

  // Reinitialize - should work
  controller->init_subscriptions("arm");
  EXPECT_TRUE(controller->get_subscriptions().count("arm") > 0);
}

// ============================================================================
// Test: Cleanup non-existent subscription doesn't crash
// ============================================================================
TEST_F(TopicSubscriptionLifecycleTest, CleanupNonExistentSubscription) {
  auto controller = std::make_shared<MockTrajectoryController>(node);
  node->declare_parameter("controllers.MockTrajectory.input_topic", "/cmd/{mapping}");

  // Should not throw
  EXPECT_NO_THROW(controller->cleanup_subscriptions("non_existent"));
  EXPECT_TRUE(controller->get_subscriptions().empty());

  // Create one and try to cleanup different mapping
  controller->init_subscriptions("arm");
  EXPECT_NO_THROW(controller->cleanup_subscriptions("different_arm"));
  EXPECT_EQ(controller->get_subscriptions().size(), 1);    // arm still exists
}

// ============================================================================
// Test: Empty mapping handling
// ============================================================================
TEST_F(TopicSubscriptionLifecycleTest, EmptyMappingHandling) {
  auto controller = std::make_shared<MockTrajectoryController>(node);
  node->declare_parameter("controllers.MockTrajectory.input_topic", "/cmd/{mapping}");

  controller->init_subscriptions("");

  // Empty mapping should not create subscription
  EXPECT_TRUE(controller->get_subscriptions().empty());
}

// ============================================================================
// Test: Missing parameter handling
// ============================================================================
TEST_F(TopicSubscriptionLifecycleTest, MissingParameterHandling) {
  auto controller = std::make_shared<MockTrajectoryController>(node);
  // Don't declare parameter

  EXPECT_NO_THROW(controller->init_subscriptions("arm"));
  EXPECT_TRUE(controller->get_subscriptions().empty());
}

// ============================================================================
// Test: Resubscription to same mapping
// ============================================================================
TEST_F(TopicSubscriptionLifecycleTest, ResubscriptionToSameMapping) {
  auto controller = std::make_shared<MockTrajectoryController>(node);
  node->declare_parameter("controllers.MockTrajectory.input_topic", "/cmd/{mapping}");

  controller->init_subscriptions("arm");
  auto first_sub = controller->get_subscriptions()["arm"];

  // Subscribe again to same mapping
  controller->init_subscriptions("arm");
  auto second_sub = controller->get_subscriptions()["arm"];

  // Should create a new subscription (pointers different)
  EXPECT_NE(first_sub, second_sub);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
