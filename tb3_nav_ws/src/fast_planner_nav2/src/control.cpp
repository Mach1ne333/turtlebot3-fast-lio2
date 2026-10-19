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
#include "fast_planner_nav2/control.hpp"
#include <algorithm>
#include <stdexcept>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_filters/filter_values.hpp>

namespace fast_planner_nav2
{
namespace
{
bool occupied(uint8_t cost, bool unknown)
{
  return cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
         (unknown && cost == nav2_costmap_2d::NO_INFORMATION);
}
double approach(double current, double target, double acceleration, double deceleration, double dt)
{
  // Sign reversals first decelerate to zero; do not spend the same dt twice.
  if (current * target < 0) {target = 0;}
  const double rate = std::abs(target) > std::abs(current) ? acceleration : deceleration;
  return current + std::clamp(target - current, -rate * dt, rate * dt);
}
}  // namespace

double angle(double value) {return std::atan2(std::sin(value), std::cos(value));}

void Settings::validate(double period) const
{
  for (double v : {max_v, max_w, accel_v, decel_v, accel_w, decel_w, lookahead,
      align_enter, align_exit, reaction_time, local_length, match_distance,
      sample_spacing, budget, final_check_reserve, tf_timeout, period})
  {
    if (!std::isfinite(v) || v <= 0) {
      throw std::invalid_argument("Nonpositive controller setting");
    }
  }
  if (!std::isfinite(safety_margin) || safety_margin < 0 || align_exit >= align_enter ||
    align_enter >= 3.141592653589793 || budget >= period || final_check_reserve >= budget ||
    reaction_time < period + budget + 0.05 || max_points < 5 || max_points > 256 ||
    max_map_cells < 1 || max_map_cells > 200000 || max_path_points < 5 ||
    max_path_points > 100000 || max_v > 0.22 || max_w > 1.0)
  {
    throw std::invalid_argument("Inconsistent Burger controller limits/budget");
  }
}

bool Grid::contains(Point p, double padding) const
{
  return p.allFinite() && p.x() >= origin_x + padding && p.y() >= origin_y + padding &&
         p.x() < origin_x + width * resolution - padding &&
         p.y() < origin_y + height * resolution - padding;
}
bool Grid::circleFree(Point p, double r) const
{
  if (!std::isfinite(r) || r < 0 || resolution <= 0 ||
    cells.size() != static_cast<size_t>(width) * height || !contains(p, r)) {return false;}
  const int x0 = std::max(0, static_cast<int>(std::floor((p.x() - r - origin_x) / resolution)));
  const int y0 = std::max(0, static_cast<int>(std::floor((p.y() - r - origin_y) / resolution)));
  const int x1 = std::min(
    static_cast<int>(width) - 1,
    static_cast<int>(std::floor((p.x() + r - origin_x) / resolution)));
  const int y1 = std::min(
    static_cast<int>(height) - 1,
    static_cast<int>(std::floor((p.y() + r - origin_y) / resolution)));
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      if (!occupied(cells[y * width + x], unknown_is_obstacle)) {continue;}
      const double nearest_x = std::clamp(
        p.x(), origin_x + x * resolution,
        origin_x + (x + 1) * resolution);
      const double nearest_y = std::clamp(
        p.y(), origin_y + y * resolution,
        origin_y + (y + 1) * resolution);
      if ((p - Point(nearest_x, nearest_y)).squaredNorm() <= r * r) {return false;}
    }
  }
  return true;
}
bool Grid::pathFree(const std::vector<Point> & path, double margin) const
{
  if (path.empty() || resolution <= 0) {return false;}
  const double step = resolution * 0.5;
  // Additional half-step covers all positions between spatial samples.
  const double r = radius + margin + step * 0.5;
  if (!circleFree(path.front(), r)) {return false;}
  for (size_t i = 1; i < path.size(); ++i) {
    const double length = (path[i] - path[i - 1]).norm();
    if (!std::isfinite(length) || length > (width + height) * resolution) {return false;}
    const size_t count = std::max<size_t>(1, std::ceil(length / step));
    for (size_t j = 1; j <= count; ++j) {
      if (!circleFree(
          path[i - 1] + (path[i] - path[i - 1]) * (static_cast<double>(j) / count),
          r))
      {
        return false;
      }
    }
  }
  return true;
}
std::vector<Point> Grid::obstacles() const
{
  std::vector<Point> result;
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      if (occupied(cells[y * width + x], unknown_is_obstacle)) {
        result.emplace_back(origin_x + (x + 0.5) * resolution, origin_y + (y + 0.5) * resolution);
      }
    }
  }
  return result;
}

LocalReference localReference(
  const std::vector<Point> & path, Point robot,
  const Grid & grid, const Settings & s, size_t & progress)
{
  if (path.size() < 2 || path.size() > static_cast<size_t>(s.max_path_points)) {
    throw std::runtime_error("Empty or degenerate global path");
  }
  progress = std::min(progress, path.size() - 2);
  size_t best = progress;
  Point projected = path[progress];
  double best_distance = (projected - robot).norm(), searched = 0;
  // Strictly local, forward, arc-length-limited matching; ties retain earlier segments.
  for (size_t i = progress; i + 1 < path.size(); ++i) {
    const Point delta = path[i + 1] - path[i];
    const double length = delta.norm();
    if (length < 1e-9) {continue;}
    const double allowed = std::min(length, s.match_distance - searched);
    const double along = std::clamp((robot - path[i]).dot(delta) / length, 0.0, allowed);
    const Point candidate = path[i] + delta * (along / length);
    const double distance = (candidate - robot).norm();
    if (distance < best_distance) {best = i; projected = candidate; best_distance = distance;}
    searched += length;
    if (searched >= s.match_distance) {break;}
  }
  if (best_distance > s.match_distance) {throw std::runtime_error("Robot too far from path");}
  progress = best;
  LocalReference result;
  result.remaining = best_distance + (path[best + 1] - projected).norm();
  for (size_t i = best + 1; i + 1 < path.size(); ++i) {
    result.remaining += (path[i + 1] - path[i]).norm();
  }
  std::vector<Point> seed{robot};
  auto append = [&](Point next) {
      if ((next - seed.back()).norm() > 1e-7) {seed.push_back(next);}
    };
  const double padding = grid.radius + s.safety_margin + grid.resolution;
  if (!grid.contains(robot, padding)) {throw std::runtime_error("Robot outside checkable map");}
  double accumulated = 0;
  bool finished = false;
  for (size_t i = best; i < path.size() && !finished; ++i) {
    Point target = i == best ? projected : path[i];
    const Point from = seed.back();
    const double length = (target - from).norm();
    if (length < 1e-7) {continue;}
    const size_t n = std::max<size_t>(1, std::ceil(length / s.sample_spacing));
    if (n > 100000) {throw std::runtime_error("Unbounded path segment");}
    for (size_t j = 1; j <= n; ++j) {
      Point p = from + (target - from) * (static_cast<double>(j) / n);
      const double distance = (p - seed.back()).norm();
      if (!grid.contains(p, padding)) {finished = true; break;}
      if (accumulated + distance >= s.local_length) {
        append(seed.back() + (p - seed.back()) * ((s.local_length - accumulated) / distance));
        accumulated = s.local_length;
        finished = true;
        break;
      }
      append(p);
      accumulated += distance;
    }
  }
  if (seed.size() < 2 || accumulated < 1e-5) {throw std::runtime_error("Zero-length local path");}
  std::vector<double> arc(seed.size(), 0);
  for (size_t i = 1; i < seed.size(); ++i) {
    arc[i] = arc[i - 1] + (seed[i] - seed[i - 1]).norm();
  }
  const size_t count = std::min<size_t>(
    s.max_points,
    std::max<size_t>(5, std::ceil(arc.back() / s.sample_spacing) + 1));
  size_t segment = 1;
  for (size_t i = 0; i < count; ++i) {
    const double target = arc.back() * i / (count - 1);
    while (segment + 1 < arc.size() && arc[segment] < target) {++segment;}
    const double fraction = (target - arc[segment - 1]) / (arc[segment] - arc[segment - 1]);
    result.points.push_back(seed[segment - 1] + fraction * (seed[segment] - seed[segment - 1]));
  }
  return result;
}

double limitedSpeed(double limit, bool percentage, double base)
{
  if (!std::isfinite(limit) || limit < 0) {throw std::invalid_argument("Invalid speed limit");}
  // Humble NO_SPEED_LIMIT == 0.0 (this is not a stop command).
  if (limit == nav2_costmap_2d::NO_SPEED_LIMIT) {return base;}
  return std::min(base, percentage ? base * limit / 100.0 : limit);
}
Velocity track(
  const Pose & p, Velocity measured, const std::vector<Point> & reference,
  const Pose & goal, double remaining, double xy, double yaw, double dt, double speed_limit,
  const Settings & s, bool & aligning)
{
  if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(measured.v) || !std::isfinite(measured.w)) {
    throw std::runtime_error("Invalid control interval/velocity");
  }
  Velocity target;
  const bool at_goal = std::hypot(goal.x - p.x, goal.y - p.y) <= xy;
  double error = angle(goal.yaw - p.yaw);
  if (!at_goal) {
    if (reference.size() < 2) {throw std::runtime_error("Missing geometric reference");}
    Point look = reference.back();
    double distance = 0;
    for (size_t i = 1; i < reference.size(); ++i) {
      const double segment = (reference[i] - reference[i - 1]).norm();
      if (distance + segment >= s.lookahead && segment > 0) {
        look = reference[i - 1] + (reference[i] - reference[i - 1]) *
          ((s.lookahead - distance) / segment);
        break;
      }
      distance += segment;
    }
    const double dx = look.x() - p.x, dy = look.y() - p.y;
    error = angle(std::atan2(dy, dx) - p.yaw);
    aligning = std::abs(error) > (aligning ? s.align_exit : s.align_enter);
    if (!aligning) {
      const double lateral = -std::sin(p.yaw) * dx + std::cos(p.yaw) * dy;
      const double curvature = 2 * lateral / std::max(1e-8, dx * dx + dy * dy);
      target.v = std::min(
        {speed_limit, s.max_v,
          s.max_w / std::max(1e-6, std::abs(curvature)),
          std::sqrt(2 * s.decel_v * std::max(0.0, remaining - xy * 0.5))});
      target.w = target.v * curvature;
    }
  } else {
    aligning = false;
  }
  if (at_goal || aligning) {
    // Stop translation before turning in place. Goal checker owns completion.
    if (std::abs(measured.v) < 0.01 && std::abs(error) > (at_goal ? yaw : s.align_exit)) {
      target.w = std::copysign(
        std::min(
          {s.max_w, 2 * std::abs(error),
            std::sqrt(2 * s.decel_w * std::abs(error))}), error);
    }
  }
  return {
    std::clamp(approach(measured.v, target.v, s.accel_v, s.decel_v, dt), 0.0, speed_limit),
    std::clamp(approach(measured.w, target.w, s.accel_w, s.decel_w, dt), -s.max_w, s.max_w)};
}

bool commandSafe(
  const Grid & grid, const Pose & initial, Velocity measured,
  Velocity command, const Settings & s)
{
  for (double v :
    {initial.x, initial.y, initial.yaw, measured.v, measured.w, command.v, command.w})
  {
    if (!std::isfinite(v)) {return false;}
  }
  const double max_v = std::max(std::abs(measured.v), std::abs(command.v));
  const double max_w = std::max(std::abs(measured.w), std::abs(command.w));
  if (max_v > s.max_v + 0.05 || max_w > s.max_w + 0.2) {return false;}
  const double stop_time = std::max(max_v / s.decel_v, max_w / s.decel_w);
  if (stop_time + s.reaction_time > 5.0) {return false;}
  // Conservative circular reachable envelope covers arbitrary heading and output delay
  // during the reaction window, including smoother transitions from measured velocity.
  // Use the configured bound as well: odometry can lag a previous faster command.
  // No temporal sampling can miss an obstacle inside this envelope.
  const double speed_bound = std::max(max_v, s.max_v);
  const double reach = speed_bound * s.reaction_time +
    speed_bound * speed_bound / (2 * s.decel_v);
  if (!grid.circleFree({initial.x, initial.y}, grid.radius + s.safety_margin + reach)) {
    return false;
  }
  // Also sample the nominal differential-drive transition and braking trajectory.
  const double dt = std::min(
    0.02, grid.resolution * 0.5 /
    std::max(1e-6, max_v + grid.radius * max_w));
  Pose pose = initial;
  Velocity velocity = measured;
  double time = 0;
  while (time < s.reaction_time + stop_time + dt) {
    const bool braking = time >= s.reaction_time;
    velocity.v = approach(velocity.v, braking ? 0 : command.v, s.accel_v, s.decel_v, dt);
    velocity.w = approach(velocity.w, braking ? 0 : command.w, s.accel_w, s.decel_w, dt);
    const double middle_yaw = pose.yaw + velocity.w * dt * 0.5;
    pose.x += velocity.v * std::cos(middle_yaw) * dt;
    pose.y += velocity.v * std::sin(middle_yaw) * dt;
    pose.yaw = angle(pose.yaw + velocity.w * dt);
    if (!grid.circleFree(
        {pose.x, pose.y},
        grid.radius + s.safety_margin + grid.resolution * 0.25))
    {
      return false;
    }
    time += dt;
  }
  return true;
}
}  // namespace fast_planner_nav2
