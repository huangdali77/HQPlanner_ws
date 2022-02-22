#include "hqplanner/tasks/poly_st_speed/speed_profile_cost.h"

#include <limits>

namespace hqplanner {
namespace tasks {
namespace {
constexpr auto kInfCost = std::numeric_limits<double>::infinity();
constexpr double kEpsilon = 1e-6;
}  // namespace

using hqplanner::PathObstacle;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::PerceptionObstacle;
using hqplanner::forproto::PolyStSpeedConfig;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::math::QuarticPolynomialCurve1dPro;
using hqplanner::speed::SpeedLimit;
using hqplanner::speed::STPoint;
SpeedProfileCost::SpeedProfileCost(
    const PolyStSpeedConfig &config,
    const std::vector<const PathObstacle *> &obstacles,
    const SpeedLimit &speed_limit, const TrajectoryPoint &init_point)
    : config_(config),
      obstacles_(obstacles),
      speed_limit_(speed_limit),
      init_point_(init_point) {}

double SpeedProfileCost::Calculate(const QuarticPolynomialCurve1dPro &curve,
                                   const double end_time,
                                   const double curr_min_cost) const {
  double cost = 0.0;
  constexpr double kDeltaT = 0.5;
  for (double t = kDeltaT; t < end_time + kEpsilon; t += kDeltaT) {
    if (cost > curr_min_cost) {
      return cost;
    }
    cost += CalculatePointCost(curve, t);
  }
  return cost;
}

double SpeedProfileCost::CalculatePointCost(
    const QuarticPolynomialCurve1dPro &curve, const double t) const {
  const double s = curve.Evaluate(0, t);
  const double v = curve.Evaluate(1, t);
  const double a = curve.Evaluate(2, t);
  const double da = curve.Evaluate(3, t);

  if (s < 0.0) {
    return kInfCost;
  }

  const double speed_limit = speed_limit_.GetSpeedLimitByS(s);
  if (v < 0.0 || v > speed_limit * (1.0 + config_.speed_limit_buffer)) {
    return kInfCost;
  }
  // if (a > config_.preferred_accel || a < config_.preferred_decel) {
  //   return kInfCost;
  // }

  double cost = 0.0;
  for (const auto *obstacle : obstacles_) {
    if (obstacle->obstacle()->Perception().type ==
        PerceptionObstacle::Type::UNKNOWN_UNMOVABLE) {
      continue;
    }

    auto boundary = obstacle->st_boundary();
    const double kIgnoreDistance = 100.0;
    if (boundary.min_s() > kIgnoreDistance) {
      continue;
    }
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
