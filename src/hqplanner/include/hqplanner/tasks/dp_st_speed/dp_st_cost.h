#ifndef HQPLANNER_TASKS_DP_ST_SPEED_DP_ST_COST_H_
#define HQPLANNER_TASKS_DP_ST_SPEED_DP_ST_COST_H_

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hqplanner/common/path_obstacle.h"
#include "hqplanner/for_proto/dp_st_speed_config.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/speed/st_boundary.h"
#include "hqplanner/speed/st_point.h"
#include "hqplanner/tasks/dp_st_speed/st_graph_point.h"

namespace hqplanner {
namespace tasks {

class DpStCost {
 public:
  explicit DpStCost(
      const hqplanner::forproto::DpStSpeedConfig& dp_st_speed_config,
      const std::vector<const PathObstacle*>& obstacles,
      const hqplanner::forproto::TrajectoryPoint& init_point);

  float GetObstacleCost(const StGraphPoint& point);

  float GetReferenceCost(
      const hqplanner::speed::STPoint& point,
      const hqplanner::speed::STPoint& reference_point) const;

  float GetSpeedCost(const hqplanner::speed::STPoint& first,
                     const hqplanner::speed::STPoint& second,
                     const float speed_limit) const;

  float GetAccelCostByTwoPoints(const float pre_speed,
                                const hqplanner::speed::STPoint& first,
                                const hqplanner::speed::STPoint& second);
  float GetAccelCostByThreePoints(const hqplanner::speed::STPoint& first,
                                  const hqplanner::speed::STPoint& second,
                                  const hqplanner::speed::STPoint& third);

  float GetJerkCostByTwoPoints(const float pre_speed, const float pre_acc,
                               const hqplanner::speed::STPoint& pre_point,
                               const hqplanner::speed::STPoint& curr_point);
  float GetJerkCostByThreePoints(const float first_speed,
                                 const hqplanner::speed::STPoint& first_point,
                                 const hqplanner::speed::STPoint& second_point,
                                 const hqplanner::speed::STPoint& third_point);

  float GetJerkCostByFourPoints(const hqplanner::speed::STPoint& first,
                                const hqplanner::speed::STPoint& second,
                                const hqplanner::speed::STPoint& third,
                                const hqplanner::speed::STPoint& fourth);

 private:
  float GetAccelCost(const float accel);
  float JerkCost(const float jerk);

  void AddToKeepClearRange(
      const std::vector<const hqplanner::PathObstacle*>& obstacles);
  static void SortAndMergeRange(
      std::vector<std::pair<float, float>>* keep_clear_range_);
  bool InKeepClearRange(float s) const;

  const hqplanner::forproto::DpStSpeedConfig& config_;
  const std::vector<const hqplanner::PathObstacle*>& obstacles_;
  const hqplanner::forproto::TrajectoryPoint& init_point_;

  float unit_t_ = 0.0;

  std::unordered_map<std::string, int> boundary_map_;
  std::vector<std::vector<std::pair<float, float>>> boundary_cost_;

  std::vector<std::pair<float, float>> keep_clear_range_;

  std::array<float, 200> accel_cost_;
  std::array<float, 400> jerk_cost_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_DP_ST_SPEED_DP_ST_COST_H_
