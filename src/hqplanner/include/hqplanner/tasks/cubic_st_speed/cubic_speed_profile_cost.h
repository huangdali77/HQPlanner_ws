#ifndef HQPLANNER_TASKS_CUBIC_ST_SPEED_CUBIC_SPEED_PROFILE_COST_H_
#define HQPLANNER_TASKS_CUBIC_ST_SPEED_CUBIC_SPEED_PROFILE_COST_H_

#include <vector>

#include "hqplanner/common/path_decision.h"
#include "hqplanner/common/path_obstacle.h"
#include "hqplanner/for_proto/cubic_st_speed_config.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/poly_st_speed_config.h"
#include "hqplanner/math/cubic_spline_clamped.h"
#include "hqplanner/speed/speed_limit.h"

namespace hqplanner {
namespace tasks {

class CubicSpeedProfileCost {
 public:
  explicit CubicSpeedProfileCost(
      const hqplanner::forproto::CubicStSpeedConfig &config,
      const std::vector<const hqplanner::PathObstacle *> &obstacles,
      const hqplanner::speed::SpeedLimit &speed_limit,
      const hqplanner::forproto::TrajectoryPoint &init_point);

  double Calculate(const hqplanner::math::CubicSplineClamped &curve,
                   const double end_time, const double curr_min_cost) const;

 private:
  double CalculatePointCost(const hqplanner::math::CubicSplineClamped &curve,
                            const double t) const;

  const hqplanner::forproto::CubicStSpeedConfig config_;
  const std::vector<const hqplanner::PathObstacle *> &obstacles_;
  const hqplanner::speed::SpeedLimit &speed_limit_;
  const hqplanner::forproto::TrajectoryPoint &init_point_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif
