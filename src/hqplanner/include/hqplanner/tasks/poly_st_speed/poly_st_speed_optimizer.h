#ifndef HQPLANNER_TASKS_POLY_ST_POLY_ST_SPEED_OPTIMIZER_H_
#define HQPLANNER_TASKS_POLY_ST_POLY_ST_SPEED_OPTIMIZER_H_

#include "hqplanner/for_proto/planning_config.h"
#include "hqplanner/for_proto/poly_st_speed_config.h"
#include "hqplanner/for_proto/st_boundary_config.h"
#include "hqplanner/for_proto/vehicle_config.h"
#include "hqplanner/tasks/speed_optimizer.h"
#include "hqplanner/tasks/st_graph/st_boundary_mapper.h"

namespace hqplanner {
namespace tasks {

class PolyStSpeedOptimizer : public SpeedOptimizer {
 public:
  PolyStSpeedOptimizer();

  bool Init(const hqplanner::forproto::PlanningConfig& config) override;

 private:
  bool Process(const hqplanner::forproto::SLBoundary& adc_sl_boundary,
               const hqplanner::path::PathData& path_data,
               const hqplanner::forproto::TrajectoryPoint& init_point,
               const hqplanner::ReferenceLine& reference_line,
               const hqplanner::speed::SpeedData& reference_speed_data,
               hqplanner::PathDecision* const path_decision,
               hqplanner::speed::SpeedData* const speed_data) override;

  hqplanner::forproto::PolyStSpeedConfig poly_st_speed_config_;
  hqplanner::forproto::StBoundaryConfig st_boundary_config_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif
