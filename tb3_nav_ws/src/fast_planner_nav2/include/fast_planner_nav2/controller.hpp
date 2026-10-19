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
#ifndef FAST_PLANNER_NAV2__CONTROLLER_HPP_
#define FAST_PLANNER_NAV2__CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <nav2_core/controller.hpp>
#include "fast_planner_nav2/control.hpp"

namespace fast_planner_nav2
{
class FastPlannerController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  Grid snapshot() const;
  void reset();
  std::mutex mutex_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::string name_;
  Settings settings_;
  nav_msgs::msg::Path plan_;
  size_t progress_ = 0;
  bool active_ = false, aligning_ = false, has_last_time_ = false;
  double period_ = 0.1, speed_limit_ = 0.22;
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
};
}  // namespace fast_planner_nav2
#endif  // FAST_PLANNER_NAV2__CONTROLLER_HPP_
