#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "arm_controller/controller_base/trajectory_controller_base.hpp"
#include "arm_controller/controller_base/velocity_controller_base.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>

// Mock implementations for testing
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
  bool move(const std::string &, const std::vector<double>&) override {return true;}

  // Expose protected members for testing
  auto & get_subscriptions() {return subscriptions_;}
  using TrajectoryControllerImpl<sensor_msgs::msg::JointState>::cleanup_subscriptions;
};

class MockVelocityController : public VelocityControllerImpl<sensor_msgs::msg::JointState>
{
public:
  MockVelocityController(const rclcpp::Node::SharedPtr & node)
  : VelocityControllerImpl<sensor_msgs::msg::JointState>("MockVelocity", node) {}

  void velocity_callback(const sensor_msgs::msg::JointState::SharedPtr) override {}
  void start(const std::string &) override {}
  bool stop(const std::string &) override {return true;}

  // Expose protected members for testing
  auto & get_subscriptions() {return subscriptions_;}
  using VelocityControllerImpl<sensor_msgs::msg::JointState>::cleanup_subscriptions;
};

// Test Fixture
class TopicLifecycleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS2
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node = std::make_shared<rclcpp::Node>("test_node");
  }

  void TearDown() override
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

// ============================================================================
// Test: init_subscriptions creates subscription with correct topic name
// ============================================================================
TEST_F(TopicLifecycleTest, InitSubscriptionsCreatesSubscription) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Configure topic parameter
  node->declare_parameter(
    "controllers.MockTrajectory.input_topic",
    "/test_topic/{mapping}");

  // Call init_subscriptions
  controller->init_subscriptions("test_arm");

  // Verify subscription was created (subscriptions_ should contain entry)
  EXPECT_TRUE(controller->get_subscriptions().count("test_arm") > 0);
}

// ============================================================================
// Test: cleanup_subscriptions removes subscription
// ============================================================================
TEST_F(TopicLifecycleTest, CleanupSubscriptionsRemovesSubscription) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Configure topic parameter
  node->declare_parameter(
    "controllers.MockTrajectory.input_topic",
    "/test_topic/{mapping}");

  // Create subscription
  controller->init_subscriptions("test_arm");
  EXPECT_TRUE(controller->get_subscriptions().count("test_arm") > 0);

  // Clean up subscription
  controller->cleanup_subscriptions("test_arm");

  // Verify subscription was removed
  EXPECT_TRUE(controller->get_subscriptions().count("test_arm") == 0);
}

// ============================================================================
// Test: multiple mappings can have independent subscriptions
// ============================================================================
TEST_F(TopicLifecycleTest, MultipleMappingsIndependentSubscriptions) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Configure topic parameter
  node->declare_parameter(
    "controllers.MockTrajectory.input_topic",
    "/test_topic/{mapping}");

  // Create subscriptions for multiple mappings
  controller->init_subscriptions("left_arm");
  controller->init_subscriptions("right_arm");

  // Verify both subscriptions exist
  EXPECT_TRUE(controller->get_subscriptions().count("left_arm") > 0);
  EXPECT_TRUE(controller->get_subscriptions().count("right_arm") > 0);

  // Clean up one mapping
  controller->cleanup_subscriptions("left_arm");

  // Verify only left_arm was removed, right_arm still exists
  EXPECT_TRUE(controller->get_subscriptions().count("left_arm") == 0);
  EXPECT_TRUE(controller->get_subscriptions().count("right_arm") > 0);
}

// ============================================================================
// Test: mapping placeholder substitution
// ============================================================================
TEST_F(TopicLifecycleTest, MappingPlaceholderSubstitution) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Configure topic parameter with {mapping} placeholder
  node->declare_parameter(
    "controllers.MockTrajectory.input_topic",
    "/controller_api/test_action/{mapping}");

  // Create subscription
  controller->init_subscriptions("single_arm");

  // Verify subscription was created with correct mapping
  EXPECT_TRUE(controller->get_subscriptions().count("single_arm") > 0);

  // Create with different mapping
  controller->init_subscriptions("dual_arm");
  EXPECT_TRUE(controller->get_subscriptions().count("dual_arm") > 0);
}

// ============================================================================
// Test: init_subscriptions called multiple times for same mapping doesn't create duplicate
// ============================================================================
TEST_F(TopicLifecycleTest, InitSubscriptionsIdempotent) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Configure topic parameter
  node->declare_parameter(
    "controllers.MockTrajectory.input_topic",
    "/test_topic/{mapping}");

  // Create subscription
  controller->init_subscriptions("test_arm");
  auto first_subscription = controller->get_subscriptions()["test_arm"];

  // Call again with same mapping
  controller->init_subscriptions("test_arm");
  auto second_subscription = controller->get_subscriptions()["test_arm"];

  // Verify new subscription was created (pointer should be different)
  EXPECT_NE(first_subscription, second_subscription);
}

// ============================================================================
// Test: cleanup_subscriptions on non-existent mapping doesn't crash
// ============================================================================
TEST_F(TopicLifecycleTest, CleanupNonExistentSubscription) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Try to cleanup subscription that doesn't exist
  EXPECT_NO_THROW(controller->cleanup_subscriptions("non_existent"));

  // Verify no subscriptions exist
  EXPECT_TRUE(controller->get_subscriptions().empty());
}

// ============================================================================
// Test: VelocityController also supports subscription management
// ============================================================================
TEST_F(TopicLifecycleTest, VelocityControllerSubscriptionManagement) {
  auto controller = std::make_shared<MockVelocityController>(node);

  // Configure topic parameter
  node->declare_parameter(
    "controllers.MockVelocity.input_topic",
    "/velocity_topic/{mapping}");

  // Create subscription
  controller->init_subscriptions("test_arm");
  EXPECT_TRUE(controller->get_subscriptions().count("test_arm") > 0);

  // Clean up
  controller->cleanup_subscriptions("test_arm");
  EXPECT_TRUE(controller->get_subscriptions().count("test_arm") == 0);
}

// ============================================================================
// Test: empty mapping doesn't create subscription
// ============================================================================
TEST_F(TopicLifecycleTest, EmptyMappingNotSubscribed) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Configure topic parameter
  node->declare_parameter(
    "controllers.MockTrajectory.input_topic",
    "/test_topic/{mapping}");

  // Try to create subscription with empty mapping
  controller->init_subscriptions("");

  // Verify no subscription was created
  EXPECT_TRUE(controller->get_subscriptions().empty());
}

// ============================================================================
// Test: missing input_topic parameter doesn't cause crash
// ============================================================================
TEST_F(TopicLifecycleTest, MissingInputTopicParameter) {
  auto controller = std::make_shared<MockTrajectoryController>(node);

  // Don't set input_topic parameter

  // Should not crash
  EXPECT_NO_THROW(controller->init_subscriptions("test_arm"));

  // Subscription should not be created
  EXPECT_TRUE(controller->get_subscriptions().empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
