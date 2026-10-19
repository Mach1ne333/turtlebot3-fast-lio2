#include "grad_replanner/PlannerInterface.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Planner
{
void PlannerInterface::initParam(double velocity, double acceleration)
{
  if (!std::isfinite(velocity) || velocity <= 0 ||
      !std::isfinite(acceleration) || acceleration <= 0) {
    throw std::invalid_argument("Invalid internal spline scales");
  }
  internal_velocity_ = velocity;
  internal_acceleration_ = acceleration;
}

void PlannerInterface::initEsdfMap(double x, double y, double z, double resolution,
  Eigen::Vector3d origin, double inflate)
{
  map_ = std::make_shared<SDFMap>();
  map_->initMap(x, y, z, resolution, origin, inflate);
}

void PlannerInterface::setCurrentPose(PathPoint pose) { pose_ = {pose.x, pose.y, 0.2}; }
void PlannerInterface::setPathPoint(const std::vector<PathPoint> & path)
{
  result_.clear();
  path_ = path;
}
void PlannerInterface::setObstacles(const std::vector<ObstacleInfo> & obstacles)
{
  result_.clear();
  obstacles_ = obstacles;  // ESDF work occurs inside the budgeted planning call.
}
void PlannerInterface::getLocalPlanTrajResults(std::vector<PathPoint> & output) const
{
  output = result_;
}
void PlannerInterface::getLocalEsdfMap(std::vector<ObstacleInfo> & output)
{
  output.clear();
  if (!map_) return;
  std::vector<Eigen::Vector3d> points;
  map_->getObstacleWorldPoints(points);
  for (const auto & p : points) {
    output.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()),
      static_cast<float>(p.z())});
  }
}

bool PlannerInterface::makePlan()
{
  return makeGeometricPlan(GeometricOptions{}, Clock::now() + std::chrono::milliseconds(80)) ==
    PlanStatus::Success;
}

PlanStatus PlannerInterface::makeGeometricPlan(
  const GeometricOptions & options, Clock::time_point deadline)
{
  result_.clear();
  error_.clear();
  auto fail = [this](PlanStatus status, const std::string & message) {
      result_.clear();
      error_ = message;
      return status;
    };
  if (Clock::now() >= deadline) return fail(PlanStatus::Timeout, "Planning budget exhausted");
  if (!map_ || !pose_.allFinite() || path_.size() < 5 || path_.size() > options.max_points ||
      obstacles_.size() > options.max_obstacles || options.retries < 1 || options.retries > 5 ||
      options.max_points > 256 || options.max_samples < 2 || options.max_samples > 16384 ||
      !std::isfinite(options.clearance) || options.clearance <= 0 ||
      !std::isfinite(options.obstacle_distance) || options.obstacle_distance < options.clearance ||
      !std::isfinite(options.sample_spacing) || options.sample_spacing <= 0 ||
      !std::isfinite(options.parameter_interval) || options.parameter_interval <= 0) {
    return fail(PlanStatus::InvalidInput, "Invalid geometric planning input/options");
  }
  std::vector<Eigen::Vector3d> points;
  double length = 0;
  for (const auto & p : path_) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      return fail(PlanStatus::InvalidInput, "Non-finite path");
    }
    Eigen::Vector3d next(p.x, p.y, 0.2);
    if (!points.empty()) {
      const double distance = (next - points.back()).norm();
      if (distance < 1e-7) return fail(PlanStatus::InvalidInput, "Duplicate path points");
      length += distance;
    }
    points.push_back(next);
  }
  if (length < 1e-5) return fail(PlanStatus::InvalidInput, "Zero-length path");
  for (const auto & p : obstacles_) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      return fail(PlanStatus::InvalidInput, "Non-finite obstacle");
    }
  }

  try {
    map_->updateMapRolling(pose_);
    for (const auto & p : obstacles_) map_->addLaserPoints({p.x, p.y, 0.2}, 1);
    map_->startUpdateMapInfo(pose_);
    if (Clock::now() >= deadline) return fail(PlanStatus::Timeout, "ESDF budget exhausted");
    Eigen::MatrixXd controls;
    std::vector<Eigen::Vector3d> derivatives(4, Eigen::Vector3d::Zero());
    NonUniformBspline::BsplineParameterize(
      options.parameter_interval, points, derivatives, controls);
    if (controls.rows() <= 6 || controls.cols() != 3 || !controls.allFinite()) {
      return fail(PlanStatus::InvalidInput, "Invalid spline parameterization");
    }
    // Enforce the planar seed exactly; a vertically extruded ESDF has zero Z gradient.
    controls.col(2).setConstant(0.2);
    GradBandOptimizer optimizer;
    optimizer.setParameterAuto(internal_velocity_, internal_acceleration_);
    optimizer.setObstacleDistance(options.obstacle_distance);
    optimizer.setDistanceField(map_);
    optimizer.setBSplineInterval(options.parameter_interval);
    optimizer.resetLambda2();
    for (int attempt = 0; attempt < options.retries; ++attempt) {
      optimizer.setControlPoints(controls);
      const double remaining = std::chrono::duration<double>(deadline - Clock::now()).count();
      if (remaining <= 0 || optimizer.optimizeTrajFixedEnd(remaining) ==
          GradBandOptimizer::Result::Timeout) {
        return fail(PlanStatus::Timeout, "Optimization budget exhausted");
      }
      controls = optimizer.getControlPoints();
      if (!controls.allFinite() || (controls.col(2).array() - 0.2).abs().maxCoeff() > 1e-6) {
        return fail(PlanStatus::OptimizationFailed, "Non-finite or non-planar spline");
      }
      NonUniformBspline spline(controls, 3, options.parameter_interval);
      const auto derivative = spline.getDerivative().getControlPoint();
      double max_derivative = 0;
      for (int row = 0; row < derivative.rows(); ++row) {
        max_derivative = std::max(max_derivative, derivative.row(row).norm());
      }
      double begin, end;
      spline.getRegion(begin, end);
      const double count = std::ceil((end - begin) * max_derivative / options.sample_spacing);
      if (!std::isfinite(count) || count > options.max_samples - 1 || end <= begin) {
        return fail(PlanStatus::OptimizationFailed, "Spline sampling exceeds bounds");
      }
      const size_t segments = std::max<size_t>(1, static_cast<size_t>(count));
      std::vector<PathPoint> candidate;
      bool safe = true;
      for (size_t i = 0; i <= segments; ++i) {
        if (Clock::now() >= deadline) return fail(PlanStatus::Timeout, "Sampling budget exhausted");
        const Eigen::Vector3d p = spline.evaluateDeBoor(begin + (end - begin) * i / segments);
        Eigen::Vector3d gradient;
        const double distance = map_->getDistWithGradTrilinear(p, gradient);
        if (!p.allFinite() || !std::isfinite(distance)) {
          return fail(PlanStatus::OptimizationFailed, "Non-finite spline result");
        }
        // Distance is 1-Lipschitz; half the sampling step covers the gaps.
        if (distance < options.clearance + options.sample_spacing * 0.5) safe = false;
        candidate.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()),
          static_cast<float>(p.z()), 0.0f});
      }
      if (safe) {
        if (Clock::now() >= deadline) return fail(PlanStatus::Timeout, "Result expired");
        result_ = std::move(candidate);
        return PlanStatus::Success;
      }
      optimizer.renewLambda2(0.05);
    }
    return fail(PlanStatus::Collision, "No collision-free geometric result");
  } catch (const std::exception & e) {
    return fail(PlanStatus::OptimizationFailed, e.what());
  }
}
}  // namespace Planner
