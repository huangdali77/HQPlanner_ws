#ifndef HQPLANNER_TASKS_DP_ST_SPEED_DP_ST_SPEED_OPTIMIZER_H_
#define HQPLANNER_TASKS_DP_ST_SPEED_DP_ST_SPEED_OPTIMIZER_H_

#include <string>

#include "hqplanner/for_proto/dp_st_speed_config.h"
#include "hqplanner/for_proto/st_boundary_config.h"
#include "hqplanner/tasks/dp_st_speed/dp_st_speed_optimizer.h"
#include "hqplanner/tasks/speed_optimizer.h"
#include "hqplanner/tasks/st_graph/speed_limit_decider.h"
#include "hqplanner/tasks/st_graph/st_boundary_mapper.h"

namespace hqplanner {
namespace tasks {

/**
 * @class DpStSpeedOptimizer
 * @brief DpStSpeedOptimizer does ST graph speed planning with dynamic
 * programming algorithm.
 */
class DpStSpeedOptimizer : public SpeedOptimizer {
 public:
  DpStSpeedOptimizer();

  bool Init(const PlanningConfig& config) override;

 private:
  bool Process(const hqplanner::forproto::SLBoundary& adc_sl_boundary,
               const hqplanner::path::PathData& path_data,
               const hqplanner::forproto::TrajectoryPoint& init_point,
               const hqplanner::ReferenceLine& reference_line,
               const hqplanner::speed::SpeedData& reference_speed_data,
               hqplanner::PathDecision* const path_decision,
               hqplanner::speed::SpeedData* const speed_data) override;

  bool SearchStGraph(const StBoundaryMapper& boundary_mapper,
                     const SpeedLimitDecider& speed_limit_decider,
                     const hqplanner::path::PathData& path_data,
                     hqplanner::speed::SpeedData* speed_data,
                     hqplanner::PathDecision* path_decision) const;

 private:
  hqplanner::forproto::TrajectoryPoint init_point_;
  const hqplanner::ReferenceLine* reference_line_ = nullptr;
  hqplanner::forproto::SLBoundary adc_sl_boundary_;
  hqplanner::forproto::DpStSpeedConfig dp_st_speed_config_;
  hqplanner::forproto::StBoundaryConfig st_boundary_config_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_DP_ST_SPEED_OPTIMIZER_H_
