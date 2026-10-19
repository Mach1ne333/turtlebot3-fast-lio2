/**
 * @brief Trajectory Optimize Base Bspline
 * @author juchunyu@qq.com
 * @copyright Copyright (c) 2026, Institute of Robotics Planning and Control (irpc).
 * Nav2 integration: explicit geometric result contract, not a timed robot trajectory.
 */
#ifndef _PLANNER_INTERFACE_H_
#define _PLANNER_INTERFACE_H_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "grad_replanner/grad_band_optimizer.h"
#include "grad_replanner/non_uniform_bspline.h"

namespace Planner
{
struct PathPoint { float x = 0, y = 0, z = 0, v = 0; };
struct ObstacleInfo { float x = 0, y = 0, z = 0; };

enum class PlanStatus { Success, InvalidInput, OptimizationFailed, Collision, Timeout };
struct GeometricOptions
{
  double clearance = 0.15;
  double obstacle_distance = 0.35;
  double sample_spacing = 0.025;
  double parameter_interval = 0.1;  // Internal spline parameter, NOT execution time.
  size_t max_points = 64;
  size_t max_obstacles = 10000;
  size_t max_samples = 4096;
  int retries = 3;
};

class PlannerInterface
{
public:
  using Clock = std::chrono::steady_clock;
  void initParam(double internal_velocity_scale, double internal_acceleration_scale);
  void initEsdfMap(double x, double y, double z, double resolution,
    Eigen::Vector3d origin, double inflate);
  void setCurrentPose(PathPoint pose);
  void setPathPoint(const std::vector<PathPoint> & path);
  void setObstacles(const std::vector<ObstacleInfo> & obstacles);
  PlanStatus makeGeometricPlan(const GeometricOptions & options, Clock::time_point deadline);
  // Kept for the visual demo. Results are geometric; v is always zero.
  bool makePlan();
  void getLocalPlanTrajResults(std::vector<PathPoint> & output) const;
  void getLocalEsdfMap(std::vector<ObstacleInfo> & output);
  const std::string & lastError() const { return error_; }

private:
  std::shared_ptr<SDFMap> map_;
  double internal_velocity_ = 2.0, internal_acceleration_ = 1.5;
  Eigen::Vector3d pose_ = Eigen::Vector3d(0, 0, 0.2);
  std::vector<PathPoint> path_, result_;
  std::vector<ObstacleInfo> obstacles_;
  std::string error_;
};
}  // namespace Planner
#endif
