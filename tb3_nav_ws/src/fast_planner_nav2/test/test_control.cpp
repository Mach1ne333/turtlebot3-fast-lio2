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
#include "fast_planner_nav2/control.hpp"
#include <nav2_costmap_2d/cost_values.hpp>

namespace fast_planner_nav2
{
Grid emptyGrid()
{
  Grid grid;
  grid.width = grid.height = 60;
  grid.resolution = 0.05;
  grid.origin_x = grid.origin_y = -1.5;
  grid.radius = 0.105;
  grid.cells.resize(3600, 0);
  return grid;
}
TEST(LocalReference, ShortPathProgressAndTruncation)
{
  Settings s;
  auto grid = emptyGrid();
  size_t progress = 0;
  auto short_path = localReference({{0, 0}, {0.01, 0}}, {0, 0}, grid, s, progress);
  EXPECT_GE(short_path.points.size(), 5u);
  auto local = localReference({{0, 0}, {0.5, 0}, {2, 0}, {5, 0}}, {0, 0}, grid, s, progress);
  EXPECT_NEAR(local.points.back().x(), s.local_length, 1e-6);
  EXPECT_NEAR(local.remaining, 5, 1e-6);
  progress = 0;
  localReference({{0, 0}, {1, 0}, {1, 1}, {0, 0}, {-1, 0}}, {0.1, 0}, grid, s, progress);
  EXPECT_EQ(progress, 0u);
  EXPECT_THROW(localReference({{0, 0}, {0, 0}}, {0, 0}, grid, s, progress), std::runtime_error);
}
TEST(Grid, CellExtentUnknownAndBoundary)
{
  auto grid = emptyGrid();
  grid.cells[30 * 60 + 34] = nav2_costmap_2d::LETHAL_OBSTACLE;
  auto obstacles = grid.obstacles();
  ASSERT_EQ(obstacles.size(), 1u);
  EXPECT_NEAR(obstacles.front().x(), 0.225, 1e-8);
  EXPECT_FALSE(grid.circleFree({0.19, 0.025}, 0.02));
  EXPECT_FALSE(grid.circleFree({1.49, 0}, 0.1));
  grid.cells[30 * 60 + 34] = nav2_costmap_2d::NO_INFORMATION;
  EXPECT_FALSE(grid.circleFree({0.19, 0.025}, 0.02));
  grid.unknown_is_obstacle = false;
  EXPECT_TRUE(grid.circleFree({0.19, 0.025}, 0.02));
}
TEST(Tracking, AlignmentGoalYawAndLimits)
{
  Settings s;
  bool aligning = false;
  auto command = track(
    {0, 0, 0}, {}, {{0, 0}, {0.5, 0.1}}, {1, 0, 0}, 1,
    0.1, 0.1, 0.1, 0.1, s, aligning);
  EXPECT_GE(command.v, 0);
  EXPECT_LE(command.v, 0.1);
  EXPECT_GT(command.w, 0);
  EXPECT_LE(command.w, s.accel_w * 0.1);
  command = track(
    {0, 0, 0}, {}, {{0, 0}, {0, 1}}, {0, 1, 0}, 1,
    0.1, 0.1, 0.1, 0.22, s, aligning);
  EXPECT_TRUE(aligning);
  EXPECT_EQ(command.v, 0);
  EXPECT_GT(command.w, 0);
  command = track(
    {0, 0, 0}, {}, {}, {0, 0, 1.0}, 0,
    0.1, 0.1, 0.1, 0.22, s, aligning);
  EXPECT_EQ(command.v, 0);
  EXPECT_GT(command.w, 0);
  command = track(
    {0, 0, 1}, {}, {}, {0, 0, 1}, 0,
    0.1, 0.1, 0.1, 0.22, s, aligning);
  EXPECT_EQ(command.v, 0);
  EXPECT_EQ(command.w, 0);
  EXPECT_DOUBLE_EQ(limitedSpeed(50, true, 0.22), 0.11);
  EXPECT_DOUBLE_EQ(limitedSpeed(0.1, false, 0.22), 0.1);
  EXPECT_DOUBLE_EQ(limitedSpeed(0, false, 0.22), 0.22);
  EXPECT_THROW(limitedSpeed(-1, false, 0.22), std::invalid_argument);
  EXPECT_THROW(track({}, {}, {}, {}, 0, 0.1, 0.1, 0, 0.22, s, aligning), std::runtime_error);
}
TEST(MotionSafety, SafeReferenceDoesNotAuthorizeUnsafeCommand)
{
  auto grid = emptyGrid();
  Settings s;
  // This cell is not on the upward reference, but a forward motion reaches it.
  grid.cells[30 * 60 + 33] = nav2_costmap_2d::LETHAL_OBSTACLE;
  EXPECT_TRUE(grid.pathFree({{0, 0}, {0, 0.5}}, s.safety_margin));
  EXPECT_FALSE(commandSafe(grid, {}, {0.22, 0}, {0.22, 0}, s));
  grid = emptyGrid();
  EXPECT_TRUE(commandSafe(grid, {}, {}, {0, 0.5}, s));
  EXPECT_FALSE(commandSafe(grid, {1.36, 0, 0}, {0.22, 0}, {0.22, 0}, s));
  grid.cells[30 * 60 + 30] = nav2_costmap_2d::LETHAL_OBSTACLE;
  EXPECT_FALSE(commandSafe(grid, {}, {}, {0, 0.5}, s));
}
TEST(Settings, BudgetAndReactionContract)
{
  Settings s;
  EXPECT_NO_THROW(s.validate(0.1));
  s.budget = 0.2;
  EXPECT_THROW(s.validate(0.1), std::invalid_argument);
  s.budget = 0.08;
  s.reaction_time = 0.1;
  EXPECT_THROW(s.validate(0.1), std::invalid_argument);
}
}  // namespace fast_planner_nav2
