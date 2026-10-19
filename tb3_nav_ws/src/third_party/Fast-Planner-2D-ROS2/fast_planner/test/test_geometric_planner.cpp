// Copyright (c) 2026 lhd
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <gtest/gtest.h>
#include <limits>
#include "grad_replanner/PlannerInterface.h"

struct OptimizerRegressionAccess
{
  static double evaluate(
    GradBandOptimizer & opt, const std::vector<double> & x,
    std::vector<double> & gradient)
  {
    opt.variable_num = 3 * (opt.end_id - opt.start_id);
    opt.min_cost = std::numeric_limits<double>::max();
    opt.time_start = quadrotor_msgs::Time::now();
    return GradBandOptimizer::costFuncMinCurvaturePara(x, gradient, &opt);
  }
};

namespace
{
using Planner::PlanStatus;
using Clock = Planner::PlannerInterface::Clock;
auto deadline() {return Clock::now() + std::chrono::seconds(2);}
std::vector<Planner::PathPoint> straight()
{
  std::vector<Planner::PathPoint> path;
  for (int i = 0; i < 12; ++i) {
    path.push_back({0.08f * i, 0, 0.2f, 0});
  }
  return path;
}
void initialize(Planner::PlannerInterface & planner)
{
  planner.initParam(2, 1.5);
  planner.initEsdfMap(3, 3, 0.2, 0.05, {-1.5, -1.5, 0.1}, 0);
  planner.setCurrentPose({0, 0, 0.2f, 0});
  planner.setPathPoint(straight());
  planner.setObstacles({});
}
TEST(GeometricPlanner, SuccessThenInvalidAndCollisionNeverReturnStaleResults)
{
  Planner::PlannerInterface planner;
  initialize(planner);
  Planner::GeometricOptions options;
  ASSERT_EQ(
    planner.makeGeometricPlan(options, deadline()),
    PlanStatus::Success) << planner.lastError();
  std::vector<Planner::PathPoint> result;
  planner.getLocalPlanTrajResults(result);
  ASSERT_GT(result.size(), 2u);
  for (const auto & p : result) {
    EXPECT_TRUE(std::isfinite(p.x) && std::isfinite(p.y));
    EXPECT_FLOAT_EQ(p.v, 0);
  }
  auto path = straight();
  path[4].x = std::numeric_limits<float>::infinity();
  planner.setPathPoint(path);
  EXPECT_EQ(planner.makeGeometricPlan(options, deadline()), PlanStatus::InvalidInput);
  planner.getLocalPlanTrajResults(result);
  EXPECT_TRUE(result.empty());
  planner.setPathPoint(straight());
  planner.setObstacles({{0, 0, 0.2f}});  // Fixed start cannot escape this obstacle.
  EXPECT_EQ(planner.makeGeometricPlan(options, deadline()), PlanStatus::Collision);
  planner.getLocalPlanTrajResults(result);
  EXPECT_TRUE(result.empty());
  planner.setObstacles({});
  EXPECT_EQ(planner.makeGeometricPlan(options, deadline()), PlanStatus::Success);
}
TEST(GeometricPlanner, DegenerateInputAndBudget)
{
  Planner::PlannerInterface planner;
  initialize(planner);
  Planner::GeometricOptions options;
  EXPECT_EQ(planner.makeGeometricPlan(options, Clock::now()), PlanStatus::Timeout);
  options.max_points = 5;
  EXPECT_EQ(planner.makeGeometricPlan(options, deadline()), PlanStatus::InvalidInput);
  options.max_points = 64;
  planner.setPathPoint(std::vector<Planner::PathPoint>(10));
  EXPECT_EQ(planner.makeGeometricPlan(options, deadline()), PlanStatus::InvalidInput);
  planner.setPathPoint({});
  EXPECT_EQ(planner.makeGeometricPlan(options, deadline()), PlanStatus::InvalidInput);
  planner.setPathPoint(straight());
  EXPECT_EQ(planner.makeGeometricPlan(options, deadline()), PlanStatus::Success);
}
TEST(Esdf, BoundsAndFullVerticalProjection)
{
  SDFMap map;
  EXPECT_THROW(map.initMap(1, 1, 1, 0, {0, 0, 0}, 0), std::invalid_argument);
  EXPECT_THROW(map.initMap(1e100, 1, 1, 0.1, {0, 0, 0}, 0), std::invalid_argument);
  map.initMap(1, 1, 0.4, 0.1, {0, 0, 0}, 0);
  map.addLaserPoints({1, 0.5, 0.2}, 1);  // Upper boundary is excluded.
  EXPECT_EQ(map.getOccupancy(Eigen::Vector3d(1, 0.5, 0.2)), -1);
  map.addLaserPoints({0.55, 0.55, 0.2}, 1);
  map.startUpdateMapInfo({0.5, 0.5, 0.2});
  Eigen::Vector3d gradient;
  const double lower = map.getDistWithGradTrilinear({0.75, 0.55, 0.15}, gradient);
  const double upper = map.getDistWithGradTrilinear({0.75, 0.55, 0.25}, gradient);
  EXPECT_NEAR(lower, upper, 1e-10);
  EXPECT_NEAR(gradient.z(), 0, 1e-10);
}
TEST(Optimizer, GradientAndPerInstanceParameters)
{
  auto map = std::make_shared<SDFMap>();
  map->initMap(6, 6, 0.4, 0.1, {-3, -3, 0}, 0);
  map->startUpdateMapInfo({0, 0, 0.2});
  Eigen::MatrixXd points(8, 3);
  for (int i = 0; i < 8; ++i) {
    points.row(i) << 0.2 * i - 0.7, 0.03 * i * i - 0.5, 0.2;
  }
  std::vector<double> x;
  for (int i = 3; i < 5; ++i) {
    for (int j = 0; j < 3; ++j) {
      x.push_back(points(i, j));
    }
  }
  auto evaluate = [&](double v, double a, double interval) {
      GradBandOptimizer optimizer;
      optimizer.setParameterAuto(v, a);
      optimizer.setDistanceField(map);
      optimizer.setBSplineInterval(interval);
      optimizer.setControlPoints(points);
      std::vector<double> gradient, scratch;
      const double cost = OptimizerRegressionAccess::evaluate(optimizer, x, gradient);
      for (size_t i = 0; i < x.size(); ++i) {
        auto plus = x, minus = x;
        plus[i] += 1e-6;
        minus[i] -= 1e-6;
        const double numerical = (OptimizerRegressionAccess::evaluate(optimizer, plus, scratch) -
          OptimizerRegressionAccess::evaluate(optimizer, minus, scratch)) / 2e-6;
        EXPECT_NEAR(gradient[i], numerical, 1e-6);
      }
      EXPECT_EQ(optimizer.optimizeTrajFixedEnd(0), GradBandOptimizer::Result::Timeout);
      return cost;
    };
  const double first = evaluate(1, 3, 1);
  const double second = evaluate(0.1, 0.1, 0.2);
  EXPECT_GT(std::abs(second - first), 1e-5);
  EXPECT_NEAR(evaluate(1, 3, 1), first, 1e-12);
  GradBandOptimizer invalid;
  invalid.setParameterAuto(1, 1);
  invalid.setBSplineInterval(0.1);
  invalid.setControlPoints(points);
  EXPECT_THROW(invalid.optimizeTrajFixedEnd(0.05), std::invalid_argument);
}
}  // namespace
