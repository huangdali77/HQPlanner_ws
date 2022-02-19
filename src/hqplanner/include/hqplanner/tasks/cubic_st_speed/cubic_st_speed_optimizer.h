#ifndef HQPLANNER_TASKS_POLY_ST_CUBIC_ST_SPEED_OPTIMIZER_H_
#define HQPLANNER_TASKS_POLY_ST_CUBIC_ST_SPEED_OPTIMIZER_H_

#include <string>

#include "hqplanner/common/path_decision.h"
#include "hqplanner/common/path_obstacle.h"
#include "hqplanner/for_proto/cubic_st_speed_config.h"
#include "hqplanner/for_proto/st_boundary_config.h"
#include "hqplanner/tasks/speed_optimizer.h"
#include "hqplanner/tasks/task.h"
namespace hqplanner {
namespace tasks {

class CubicStSpeedOptimizer : public SpeedOptimizer {
 public:
  CubicStSpeedOptimizer();
  //   ~CubicStSpeedOptimizer() = default;

  bool Init(const hqplanner::forproto::PlanningConfig& config) override;

 private:
  bool Process(const hqplanner::forproto::SLBoundary& adc_sl_boundary,
               const hqplanner::path::PathData& path_data,
               const hqplanner::forproto::TrajectoryPoint& init_point,
               const hqplanner::ReferenceLine& reference_line,
               const hqplanner::speed::SpeedData& reference_speed_data,
               hqplanner::PathDecision* const path_decision,
               hqplanner::speed::SpeedData* const speed_data) override;

 private:
  hqplanner::forproto::CubicStSpeedConfig cubic_st_speed_config_;
  hqplanner::forproto::StBoundaryConfig st_boundary_config_;
  hqplanner::forproto::SLBoundary adc_sl_boundary_;
  hqplanner::forproto::TrajectoryPoint init_point_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif
