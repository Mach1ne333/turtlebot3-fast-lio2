// Copyright 2026 lhd
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <gtest/gtest.h>
#include <thread>
#include <pluginlib/class_loader.hpp>
#include <nav2_core/exceptions.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include "fast_planner_nav2/controller.hpp"

namespace
{
class TestCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  TestCostmap()
  : Costmap2DROS("fp_test_costmap")
  {
    global_frame_ = "odom";
    robot_base_frame_ = "base_link";
    use_radius_ = true;
    robot_radius_ = 0.105;
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom", false, false);
    layered_costmap_->resizeMap(60, 60, 0.05, -1.5, -1.5);
    setRobotFootprint(nav2_costmap_2d::makeFootprintFromRadius(robot_radius_));
  }
};
class GoalChecker : public nav2_core::GoalChecker
{
public:
  bool slow = false;
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(
    const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override {return false;}
  bool getTolerances(geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist &) override
  {
    if (slow) {std::this_thread::sleep_for(std::chrono::milliseconds(90));}
    pose.position.x = pose.position.y = 0.1;
    pose.orientation.z = std::sin(0.05);
    pose.orientation.w = std::cos(0.05);
    return true;
  }
};
nav_msgs::msg::Path path(double x, double yaw = 0)
{
  nav_msgs::msg::Path output;
  output.header.frame_id = "odom";
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "odom";
  p.pose.orientation.w = 1;
  output.poses.push_back(p);
  p.pose.position.x = x;
  p.pose.orientation.z = std::sin(yaw * 0.5);
  p.pose.orientation.w = std::cos(yaw * 0.5);
  output.poses.push_back(p);
  return output;
}
TEST(Controller, PluginLifecycleFailureAndNewTask)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("fp_controller_test");
    auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf->setUsingDedicatedThread(true);
    auto map = std::make_shared<TestCostmap>();
    pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
    auto controller = loader.createSharedInstance("fast_planner_nav2::FastPlannerController");
    ASSERT_NE(controller, nullptr);
    controller->configure(node, "FollowPath", tf, map);
    EXPECT_THROW(controller->setPlan(path(0.8)), nav2_core::PlannerException);
    controller->activate();
    GoalChecker checker;
    auto pose = path(0).poses.front();
    geometry_msgs::msg::Twist velocity;
    controller->setPlan(path(0, 1));
    auto command = controller->computeVelocityCommands(pose, velocity, &checker);
    EXPECT_EQ(command.twist.linear.x, 0);
    EXPECT_GT(command.twist.angular.z, 0);
    controller->setPlan(path(0.8));
    command = controller->computeVelocityCommands(pose, velocity, &checker);
    EXPECT_GT(command.twist.linear.x, 0);
    EXPECT_EQ(command.twist.linear.y, 0);
    EXPECT_EQ(command.header.frame_id, "base_link");
    // Exceptions, not normal zero commands, tell Humble that control failed.
    checker.slow = true;
    controller->setPlan(path(0.8));
    EXPECT_THROW(
      controller->computeVelocityCommands(
        pose, velocity,
        &checker), nav2_core::PlannerException);
    checker.slow = false;
    controller->setPlan(path(0, -1));
    command = controller->computeVelocityCommands(pose, velocity, &checker);
    EXPECT_LT(command.twist.angular.z, 0);
    auto missing_tf = path(0.8);
    missing_tf.header.frame_id = "missing_frame";
    for (auto & p : missing_tf.poses) {
      p.header.frame_id = "missing_frame";
    }
    controller->setPlan(missing_tf);
    EXPECT_THROW(
      controller->computeVelocityCommands(
        pose, velocity,
        &checker), nav2_core::PlannerException);
    EXPECT_THROW(controller->setPlan(nav_msgs::msg::Path{}), nav2_core::PlannerException);
    EXPECT_THROW(
      controller->computeVelocityCommands(
        pose, velocity,
        &checker), nav2_core::PlannerException);
    // A paused or reset simulation clock must not produce dt-derived motion.
    auto * clock = node->get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock, 0), RCL_RET_OK);
    controller->setPlan(path(0, 1));
    EXPECT_NO_THROW(controller->computeVelocityCommands(pose, velocity, &checker));
    EXPECT_THROW(
      controller->computeVelocityCommands(
        pose, velocity,
        &checker), nav2_core::PlannerException);
    ASSERT_EQ(rcl_set_ros_time_override(clock, 100000000), RCL_RET_OK);
    EXPECT_NO_THROW(controller->computeVelocityCommands(pose, velocity, &checker));
    ASSERT_EQ(rcl_set_ros_time_override(clock, 50000000), RCL_RET_OK);
    EXPECT_THROW(
      controller->computeVelocityCommands(
        pose, velocity,
        &checker), nav2_core::PlannerException);
    ASSERT_EQ(rcl_disable_ros_time_override(clock), RCL_RET_OK);
    controller->deactivate();
    EXPECT_THROW(
      controller->computeVelocityCommands(
        pose, velocity,
        &checker), nav2_core::PlannerException);
    controller->activate();
    EXPECT_THROW(
      controller->computeVelocityCommands(
        pose, velocity,
        &checker), nav2_core::PlannerException);
    controller->cleanup();
    EXPECT_THROW(controller->activate(), nav2_core::PlannerException);
  }
  rclcpp::shutdown();
}
}  // namespace
