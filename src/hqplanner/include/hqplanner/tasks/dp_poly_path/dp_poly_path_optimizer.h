#ifndef HQPLANNER_TASKS_DP_POLY_PATH_OPTIMIZER_H_
#define HQPLANNER_TASKS_DP_POLY_PATH_OPTIMIZER_H_

#include <string>

#include "hqplanner/for_proto/dp_poly_path_config.h"
#include "hqplanner/for_proto/planning_config.h"
#include "hqplanner/path/path_data.h"
#include "hqplanner/tasks/path_optimizer.h"
namespace hqplanner {
namespace tasks {

/**
 * @class DpPolyPathOptimizer
 * @brief DpPolyPathOptimizer does path planning with dynamic programming
 * algorithm.
 */
class DpPolyPathOptimizer : public PathOptimizer {
 public:
  DpPolyPathOptimizer();

  bool Init(const hqplanner::forproto::PlanningConfig &config) override;

 private:
  bool Process(const hqplanner::speed::SpeedData &speed_data,
               const hqplanner::ReferenceLine &reference_line,
               const hqplanner::forproto::TrajectoryPoint &init_point,
               hqplanner::path::PathData *const path_data) override;

 private:
  hqplanner::forproto::DpPolyPathConfig config_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif
