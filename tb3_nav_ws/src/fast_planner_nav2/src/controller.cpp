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
#include "fast_planner_nav2/controller.hpp"
#include <tf2/utils.h>
#include <algorithm>
#include <chrono>
#include <nav2_core/exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "grad_replanner/PlannerInterface.h"

namespace fast_planner_nav2
{
namespace
{
using Clock = std::chrono::steady_clock;
void validatePose(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  for (double v : {p.position.x, p.position.y, p.position.z, q.x, q.y, q.z, q.w}) {
    if (!std::isfinite(v)) {throw nav2_core::PlannerException("Non-finite pose");}
  }
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (std::abs(norm - 1.0) > 0.01) {throw nav2_core::PlannerException("Invalid pose quaternion");}
}
Pose unpack(const geometry_msgs::msg::Pose & p)
{
  validatePose(p);
  return {p.position.x, p.position.y, tf2::getYaw(p.orientation)};
}
}  // namespace

void FastPlannerController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  std::lock_guard<std::mutex> lock(mutex_);
  node_ = parent.lock();
  if (!node_ || !tf || !costmap || !costmap->getCostmap()) {
    throw nav2_core::PlannerException("Missing controller dependencies");
  }
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ = std::move(costmap);
  auto parameter = [this](const std::string & key, auto & value) {
      const auto full = name_ + "." + key;
      if (!node_->has_parameter(full)) {node_->declare_parameter(full, value);}
      node_->get_parameter(full, value);
    };
  parameter("max_v", settings_.max_v);
  parameter("max_w", settings_.max_w);
  parameter("accel_v", settings_.accel_v);
  parameter("decel_v", settings_.decel_v);
  parameter("accel_w", settings_.accel_w);
  parameter("decel_w", settings_.decel_w);
  parameter("lookahead", settings_.lookahead);
  parameter("align_enter", settings_.align_enter);
  parameter("align_exit", settings_.align_exit);
  parameter("safety_margin", settings_.safety_margin);
  parameter("reaction_time", settings_.reaction_time);
  parameter("local_length", settings_.local_length);
  parameter("match_distance", settings_.match_distance);
  parameter("sample_spacing", settings_.sample_spacing);
  parameter("budget", settings_.budget);
  parameter("final_check_reserve", settings_.final_check_reserve);
  parameter("tf_timeout", settings_.tf_timeout);
  parameter("unknown_is_obstacle", settings_.unknown_is_obstacle);
  parameter("max_points", settings_.max_points);
  parameter("max_map_cells", settings_.max_map_cells);
  parameter("max_path_points", settings_.max_path_points);
  double frequency = 10.0;
  node_->get_parameter_or("controller_frequency", frequency, frequency);
  period_ = 1.0 / frequency;
  settings_.validate(period_);
  speed_limit_ = settings_.max_v;
  active_ = false;
  reset();
}
void FastPlannerController::reset()
{
  plan_ = nav_msgs::msg::Path{};
  progress_ = 0;
  aligning_ = false;
  last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  has_last_time_ = false;
}
void FastPlannerController::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!node_ || !costmap_) {throw nav2_core::PlannerException("Unconfigured controller");}
  active_ = true;
}
void FastPlannerController::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  reset();
}
void FastPlannerController::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  reset();
  costmap_.reset();
  tf_.reset();
  node_.reset();
}
void FastPlannerController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  reset();
  if (!active_ || path.poses.empty() || path.header.frame_id.empty() ||
    path.poses.size() > static_cast<size_t>(settings_.max_path_points))
  {
    throw nav2_core::PlannerException("Inactive controller or invalid path");
  }
  for (const auto & p : path.poses) {
    validatePose(p.pose);
    if (!p.header.frame_id.empty() && p.header.frame_id != path.header.frame_id) {
      throw nav2_core::PlannerException("Mixed path coordinate frames");
    }
  }
  plan_ = path;
}
void FastPlannerController::setSpeedLimit(const double & limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  speed_limit_ = limitedSpeed(limit, percentage, settings_.max_v);
}

Grid FastPlannerController::snapshot() const
{
  auto * map = costmap_->getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
  if (!costmap_->isCurrent()) {throw nav2_core::PlannerException("Costmap is not current");}
  Grid grid;
  grid.width = map->getSizeInCellsX();
  grid.height = map->getSizeInCellsY();
  grid.resolution = map->getResolution();
  grid.origin_x = map->getOriginX();
  grid.origin_y = map->getOriginY();
  const size_t size = static_cast<size_t>(grid.width) * grid.height;
  if (!size || size > static_cast<size_t>(settings_.max_map_cells) ||
    !std::isfinite(grid.resolution) || grid.resolution <= 0 ||
    !std::isfinite(grid.origin_x) || !std::isfinite(grid.origin_y))
  {
    throw nav2_core::PlannerException("Invalid or oversized local map");
  }
  // Use the circumscribed circle of the padded footprint, including for rotations.
  for (const auto & point : costmap_->getRobotFootprint()) {
    const double radius = std::hypot(point.x, point.y);
    if (!std::isfinite(radius)) {throw nav2_core::PlannerException("Invalid footprint");}
    grid.radius = std::max(grid.radius, radius);
  }
  if (costmap_->getUseRadius()) {grid.radius = std::max(grid.radius, costmap_->getRobotRadius());}
  if (grid.radius <= 0) {throw nav2_core::PlannerException("Missing robot footprint");}
  grid.unknown_is_obstacle = settings_.unknown_is_obstacle;
  grid.cells.assign(map->getCharMap(), map->getCharMap() + size);
  return grid;
}

geometry_msgs::msg::TwistStamped FastPlannerController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & input, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * checker)
{
  const auto start = Clock::now();  // Includes time waiting for controller-state lock.
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || plan_.poses.empty() || !checker) {
    throw nav2_core::PlannerException("Inactive controller, empty path or missing goal checker");
  }
  const auto deadline = start + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(settings_.budget));
  auto withinBudget = [&]() {
      if (Clock::now() >= deadline) {
        throw nav2_core::PlannerException("Control calculation timed out");
      }
    };
  try {
    const auto now = node_->now();
    double dt = has_last_time_ ? (now - last_time_).seconds() : period_;
    last_time_ = now;
    has_last_time_ = true;
    if (dt <= 0 || dt > 2 * period_) {
      aligning_ = false;
      throw nav2_core::PlannerException("Control time jumped or stopped");
    }
    for (double v : {velocity.linear.x, velocity.linear.y, velocity.linear.z,
        velocity.angular.x, velocity.angular.y, velocity.angular.z})
    {
      if (!std::isfinite(v)) {throw nav2_core::PlannerException("Non-finite measured velocity");}
    }
    validatePose(input.pose);
    const std::string frame = costmap_->getGlobalFrameID();
    geometry_msgs::msg::PoseStamped robot = input;
    if (input.header.frame_id != frame) {
      robot = tf_->transform(input, frame, tf2::durationFromSec(settings_.tf_timeout));
    }
    const Pose pose = unpack(robot.pose);
    geometry_msgs::msg::TransformStamped transform;
    const bool needs_transform = plan_.header.frame_id != frame;
    if (needs_transform) {
      transform = tf_->lookupTransform(
        frame, plan_.header.frame_id, now,
        rclcpp::Duration::from_seconds(settings_.tf_timeout));
    }
    std::vector<Point> path;
    Pose goal;
    for (size_t i = 0; i < plan_.poses.size(); ++i) {
      auto p = plan_.poses[i];
      if (needs_transform) {tf2::doTransform(p, p, transform);}
      const Pose converted = unpack(p.pose);
      const Point point(converted.x, converted.y);
      if (path.empty() || (point - path.back()).norm() > 1e-7) {path.push_back(point);}
      if (i + 1 == plan_.poses.size()) {goal = converted;}
    }
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    if (!checker->getTolerances(pose_tolerance, velocity_tolerance)) {
      throw nav2_core::PlannerException("Goal checker has no usable tolerances");
    }
    const double xy = pose_tolerance.position.x;
    const double yaw = std::abs(tf2::getYaw(pose_tolerance.orientation));
    if (!std::isfinite(xy) || xy <= 0 || !std::isfinite(yaw) || yaw <= 0) {
      throw nav2_core::PlannerException("Unsupported goal checker tolerances");
    }
    withinBudget();
    const auto grid = snapshot();
    std::vector<Point> reference;
    double remaining = 0;
    if (std::hypot(goal.x - pose.x, goal.y - pose.y) > xy) {
      const auto local = localReference(path, {pose.x, pose.y}, grid, settings_, progress_);
      remaining = local.remaining;
      Planner::PlannerInterface planner;
      const double r = grid.resolution;
      // Add a border to tolerate the rolling ESDF's origin snap. Final checks still
      // use exactly the original costmap bounds, not the expanded ESDF bounds.
      planner.initEsdfMap(
        (grid.width + 4) * r, (grid.height + 4) * r, 4 * r, r,
        {grid.origin_x - 2 * r, grid.origin_y - 2 * r, 0.2 - 2 * r}, 0);
      planner.setCurrentPose({static_cast<float>(pose.x), static_cast<float>(pose.y), 0.2f, 0});
      std::vector<Planner::PathPoint> points;
      for (const auto & p : local.points) {
        points.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), 0.2f, 0});
      }
      planner.setPathPoint(points);
      std::vector<Planner::ObstacleInfo> obstacles;
      for (const auto & p : grid.obstacles()) {
        obstacles.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), 0.2f});
      }
      planner.setObstacles(obstacles);
      Planner::GeometricOptions options;
      options.max_points = settings_.max_points;
      options.max_obstacles = settings_.max_map_cells;
      options.sample_spacing = r * 0.5;
      // Includes cell extent, ESDF re-quantization and interpolation error.
      options.clearance = grid.radius + settings_.safety_margin + 2 * std::sqrt(2.0) * r;
      options.obstacle_distance = options.clearance + 0.1;
      const auto planning_deadline = deadline - std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(settings_.final_check_reserve));
      if (planner.makeGeometricPlan(options, planning_deadline) != Planner::PlanStatus::Success) {
        throw nav2_core::PlannerException("Geometric planning failed: " + planner.lastError());
      }
      planner.getLocalPlanTrajResults(points);
      reference.emplace_back(pose.x, pose.y);
      for (const auto & p : points) {
        reference.emplace_back(p.x, p.y);
      }
      if (!grid.pathFree(reference, settings_.safety_margin)) {
        throw nav2_core::PlannerException("Reference path/connection collides");
      }
    }
    withinBudget();
    const Velocity measured{velocity.linear.x, velocity.angular.z};
    const auto command = track(
      pose, measured, reference, goal, remaining,
      xy, yaw, dt, speed_limit_, settings_, aligning_);
    const auto latest = snapshot();
    if ((!reference.empty() && !latest.pathFree(reference, settings_.safety_margin)) ||
      !commandSafe(latest, pose, measured, command, settings_))
    {
      throw nav2_core::PlannerException("Final motion or stopping region is unsafe");
    }
    withinBudget();
    geometry_msgs::msg::TwistStamped output;
    output.header.frame_id = costmap_->getBaseFrameID();
    output.header.stamp = now;
    output.twist.linear.x = command.v;
    output.twist.angular.z = command.w;
    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    RCLCPP_DEBUG(node_->get_logger(), "Fast Planner control %.3f ms", elapsed_ms);
    return output;
  } catch (const std::exception & e) {
    aligning_ = false;
    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Fast Planner failed (%.3f ms): %s", elapsed_ms, e.what());
    throw nav2_core::PlannerException(e.what());
  }
}
}  // namespace fast_planner_nav2
PLUGINLIB_EXPORT_CLASS(fast_planner_nav2::FastPlannerController, nav2_core::Controller)
