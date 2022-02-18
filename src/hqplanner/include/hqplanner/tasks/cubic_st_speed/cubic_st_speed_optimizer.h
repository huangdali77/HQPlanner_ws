#ifndef HQPLANNER_TASKS_POLY_ST_CUBIC_ST_SPEED_OPTIMIZER_H_
#define HQPLANNER_TASKS_POLY_ST_CUBIC_ST_SPEED_OPTIMIZER_H_

#include <string>

#include "hqplanner/common/path_decision.h"
#include "hqplanner/common/path_obstacle.h"
#include "hqplanner/for_proto/dp_st_speed_config.h"
#include "hqplanner/for_proto/st_boundary_config.h"
#include "hqplanner/tasks/task.h"

namespace hqplanner {
namespace tasks {

class CubicStSpeedOptimizer : public Task {
 public:
  CubicStSpeedOptimizer();
  ~CubicStSpeedOptimizer() = default;

  bool Init(const hqplanner::forproto::PlanningConfig& config) override;

  bool Execute(hqplanner::Frame* frame,
               hqplanner::ReferenceLineInfo* reference_line_info) override;
};

}  // namespace tasks
}  // namespace hqplanner

#endif
