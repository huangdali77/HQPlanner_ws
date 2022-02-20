#ifndef HQPLANNER_TASKS_POLY_ST_SPEED_SPEED_PROFILE_COST_H_
#define HQPLANNER_TASKS_POLY_ST_SPEED_SPEED_PROFILE_COST_H_

#include <vector>

#include "hqplanner/common/path_decision.h"
#include "hqplanner/common/path_obstacle.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/poly_st_speed_config.h"
#include "hqplanner/math/curve1d/quartic_polynomial_curve1d.h"
#include "hqplanner/math/curve1d/quartic_polynomial_curve1d_pro.h"
#include "hqplanner/speed/speed_limit.h"
namespace hqplanner {
namespace tasks {

class SpeedProfileCost {
 public:
  explicit SpeedProfileCost(
      const hqplanner::forproto::PolyStSpeedConfig &config,
      const std::vector<const hqplanner::PathObstacle *> &obstacles,
      const hqplanner::speed::SpeedLimit &speed_limit,
      const hqplanner::forproto::TrajectoryPoint &init_point);

  double Calculate(const hqplanner::math::QuarticPolynomialCurve1dPro &curve,
                   const double end_time, const double curr_min_cost) const;

 private:
  double CalculatePointCost(
      const hqplanner::math::QuarticPolynomialCurve1dPro &curve,
      const double t) const;

  const hqplanner::forproto::PolyStSpeedConfig config_;
  const std::vector<const hqplanner::PathObstacle *> &obstacles_;
  const hqplanner::speed::SpeedLimit &speed_limit_;
  const hqplanner::forproto::TrajectoryPoint &init_point_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif
