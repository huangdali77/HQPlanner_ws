#ifndef HQPLANNER_TASKS_DP_POLY_PATH_TRAJECTORY_COST_H_
#define HQPLANNER_TASKS_DP_POLY_PATH_TRAJECTORY_COST_H_

#include <vector>

#include "hqplanner/common/obstacle.h"
#include "hqplanner/common/path_decision.h"
#include "hqplanner/for_proto/dp_poly_path_config.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/vehicle_config.h"
#include "hqplanner/math/box2d.h"
#include "hqplanner/math/curve1d/quintic_polynomial_curve1d.h"
#include "hqplanner/reference_line/reference_line.h"
#include "hqplanner/speed/speed_data.h"
#include "hqplanner/tasks/dp_poly_path/comparable_cost.h"
namespace hqplanner {
namespace tasks {

class TrajectoryCost {
 public:
  TrajectoryCost() = default;
  explicit TrajectoryCost(
      const hqplanner::forproto::DpPolyPathConfig &config,
      const hqplanner::ReferenceLine &reference_line,
      const bool is_change_lane_path,
      const std::vector<const hqplanner::PathObstacle *> &obstacles,
      const hqplanner::forproto::VehicleParam &vehicle_param,
      const hqplanner::speed::SpeedData &heuristic_speed_data,
      const hqplanner::forproto::SLPoint &init_sl_point);
  ComparableCost Calculate(
      const hqplanner::math::QuinticPolynomialCurve1d &curve,
      const float start_s, const float end_s, const uint32_t curr_level,
      const uint32_t total_level);

 private:
  ComparableCost CalculatePathCost(
      const hqplanner::math::QuinticPolynomialCurve1d &curve,
      const float start_s, const float end_s, const uint32_t curr_level,
      const uint32_t total_level);
  ComparableCost CalculateStaticObstacleCost(
      const hqplanner::math::QuinticPolynomialCurve1d &curve,
      const float start_s, const float end_s);
  ComparableCost CalculateDynamicObstacleCost(
      const hqplanner::math::QuinticPolynomialCurve1d &curve,
      const float start_s, const float end_s) const;
  ComparableCost GetCostBetweenObsBoxes(
      const hqplanner::math::Box2d &ego_box,
      const hqplanner::math::Box2d &obstacle_box) const;

  //   FRIEND_TEST(AllTrajectoryTests, GetCostFromObsSL);
  ComparableCost GetCostFromObsSL(
      const float adc_s, const float adc_l,
      const hqplanner::forproto::SLBoundary &obs_sl_boundary);

  hqplanner::math::Box2d GetBoxFromSLPoint(
      const hqplanner::forproto::SLPoint &sl, const float dl) const;

  const hqplanner::forproto::DpPolyPathConfig config_;
  const hqplanner::ReferenceLine *reference_line_ = nullptr;
  bool is_change_lane_path_ = false;
  const hqplanner::forproto::VehicleParam vehicle_param_;
  hqplanner::speed::SpeedData heuristic_speed_data_;
  const hqplanner::forproto::SLPoint init_sl_point_;
  uint32_t num_of_time_stamps_ = 0;
  std::vector<std::vector<hqplanner::math::Box2d>> dynamic_obstacle_boxes_;
  std::vector<float> obstacle_probabilities_;

  std::vector<hqplanner::forproto::SLBoundary> static_obstacle_sl_boundaries_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif
