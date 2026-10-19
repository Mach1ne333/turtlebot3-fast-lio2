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
#ifndef FAST_PLANNER_NAV2__CONTROL_HPP_
#define FAST_PLANNER_NAV2__CONTROL_HPP_

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace fast_planner_nav2
{
using Point = Eigen::Vector2d;
struct Pose { double x = 0, y = 0, yaw = 0; };
struct Velocity { double v = 0, w = 0; };
struct Settings
{
  double max_v = 0.22, max_w = 1.0;
  double accel_v = 2.5, decel_v = 2.5, accel_w = 3.2, decel_w = 3.2;
  double lookahead = 0.3, align_enter = 0.7, align_exit = 0.2;
  double safety_margin = 0.02, reaction_time = 0.25;
  double local_length = 1.0, match_distance = 0.5, sample_spacing = 0.05;
  double budget = 0.08, final_check_reserve = 0.015, tf_timeout = 0.01;
  bool unknown_is_obstacle = true;
  int max_points = 32, max_map_cells = 20000, max_path_points = 10000;
  void validate(double controller_period) const;
};
struct Grid
{
  unsigned int width = 0, height = 0;
  double resolution = 0, origin_x = 0, origin_y = 0, radius = 0;
  bool unknown_is_obstacle = true;
  std::vector<uint8_t> cells;
  bool contains(Point point, double padding = 0) const;
  bool circleFree(Point point, double radius) const;
  bool pathFree(const std::vector<Point> & path, double margin) const;
  std::vector<Point> obstacles() const;
};
struct LocalReference
{
  std::vector<Point> points;
  double remaining = 0;
};
LocalReference localReference(
  const std::vector<Point> & path, Point robot,
  const Grid & grid, const Settings & settings, size_t & progress);
Velocity track(
  const Pose & pose, Velocity measured, const std::vector<Point> & reference,
  const Pose & goal, double remaining, double xy_tolerance, double yaw_tolerance,
  double dt, double speed_limit, const Settings & settings, bool & aligning);
bool commandSafe(
  const Grid & grid, const Pose & pose, Velocity measured,
  Velocity command, const Settings & settings);
double limitedSpeed(double limit, bool percentage, double base);
double angle(double value);
}  // namespace fast_planner_nav2
#endif  // FAST_PLANNER_NAV2__CONTROL_HPP_
