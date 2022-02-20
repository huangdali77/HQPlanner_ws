#include "hqplanner/tasks/cubic_st_speed/cubic_speed_profile_cost.h"

#include <ros/ros.h>

#include <limits>

namespace hqplanner {
namespace tasks {
namespace {
constexpr auto kInfCost = std::numeric_limits<double>::infinity();
constexpr double kEpsilon = 1e-6;
}  // namespace

using hqplanner::PathObstacle;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::CubicStSpeedConfig;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::math::CubicSplineClamped;
using hqplanner::speed::SpeedLimit;
using hqplanner::speed::STPoint;

CubicSpeedProfileCost::CubicSpeedProfileCost(
    const CubicStSpeedConfig &config,
    const std::vector<const PathObstacle *> &obstacles,
    const SpeedLimit &speed_limit, const TrajectoryPoint &init_point)
    : config_(config),
      obstacles_(obstacles),
      speed_limit_(speed_limit),
      init_point_(init_point) {}

double CubicSpeedProfileCost::Calculate(const CubicSplineClamped &curve,
                                        const double end_time,
                                        const double curr_min_cost) const {
  ROS_INFO("Calculate obs size:%d", static_cast<int>(obstacles_.size()));
  double cost = 0.0;
  constexpr double kDeltaT = 0.2;
  ROS_INFO("endtime:%f", end_time);
  for (double t = kDeltaT; t < end_time + kEpsilon; t += kDeltaT) {
    ROS_INFO("endtime:%f, t:%f", end_time, t);
    if (cost > curr_min_cost) {
      return cost;
    }
    cost += CalculatePointCost(curve, t);
  }
  return cost;
}

double CubicSpeedProfileCost::CalculatePointCost(
    const CubicSplineClamped &curve, const double t) const {
  ROS_INFO("================");
  const double s = curve.GetSplinePointValue(t);
  ROS_INFO("================");
  const double v = curve.GetSplinePointFirstDerivativeValue(t);
  const double a = curve.GetSplinePointSecondDerivativeValue(t);
  const double da = curve.GetSplinePointThirdDerivativeValue(t);

  if (s < 0.0) {
    ROS_INFO("s < 0.0");
    return kInfCost;
  }

  const double speed_limit = speed_limit_.GetSpeedLimitByS(s);
  if (v < 0.0 || v > speed_limit * (1.0 + config_.speed_limit_buffer)) {
    ROS_INFO("v =%f", v);
    return kInfCost;
  }
  // if (a > config_.max_accel || a < config_.min_decel) {
  //   ROS_INFO("a =%f", a);
  //   return kInfCost;
  // }

  double cost = 0.0;
  for (const auto *obstacle : obstacles_) {
    ROS_INFO("for");
    ROS_INFO("obs size:%d", static_cast<int>(obstacles_.size()));
    auto boundary = obstacle->st_boundary();
    // const double kIgnoreDistance = 100.0;
    // if (boundary.min_s() > kIgnoreDistance) {
    //   continue;
    // }
    if (t < boundary.min_t() || t > boundary.max_t()) {
      continue;
    }

    if (obstacle->IsBlockingObstacle() &&
        boundary.IsPointInBoundary(STPoint(s, t))) {
      return kInfCost;
    }

    double s_upper = 0.0;
    double s_lower = 0.0;
    boundary.GetBoundarySRange(t, &s_upper, &s_lower);
    if (s < s_lower) {
      const double len = v * ConfigParam::instance()->FLAGS_follow_time_buffer;
      if (s + len < s_lower) {
        continue;
      } else {
        cost += config_.obstacle_weight * std::pow((len - s_lower + s), 2);
      }
    } else if (s > s_upper) {
      const double kSafeDistance = 15.0;  // or calculated from velocity
      if (s > s_upper + kSafeDistance) {
        continue;
      } else {
        cost += config_.obstacle_weight *
                std::pow((kSafeDistance + s_upper - s), 2);
      }
    } else {
      if (!obstacle->IsBlockingObstacle()) {
        cost += config_.unblocking_obstacle_cost;
      }
    }
  }
  cost += config_.speed_weight * std::pow((v - speed_limit), 2);
  cost += config_.accelerate_weight * std::pow(a, 2);
  cost += config_.jerk_weight * std::pow(da, 2);

  return cost;
}

}  // namespace tasks
}  // namespace hqplanner
