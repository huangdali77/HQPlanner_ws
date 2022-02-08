#ifndef HQPLANNER_TASKS_SPEED_OPTIMIZER_H_
#define HQPLANNER_TASKS_SPEED_OPTIMIZER_H_

#include <string>
#include <vector>

#include "hqplanner/speed/speed_data.h"
#include "hqplanner/tasks/st_graph/st_graph_data.h"
#include "hqplanner/tasks/task.h"

namespace hqplanner {
namespace tasks {

class SpeedOptimizer : public Task {
 public:
  explicit SpeedOptimizer(const std::string& name);
  virtual ~SpeedOptimizer() = default;
  bool Execute(Frame* frame, ReferenceLineInfo* reference_line_info) override;

 protected:
  virtual bool Process(const hqplanner::forproto::SLBoundary& adc_sl_boundary,
                       const hqplanner::path::PathData& path_data,
                       const hqplanner::forproto::TrajectoryPoint& init_point,
                       const hqplanner::ReferenceLine& reference_line,
                       const hqplanner::speed::SpeedData& reference_speed_data,
                       hqplanner::PathDecision* const path_decision,
                       hqplanner::speed::SpeedData* const speed_data) = 0;
};

}  // namespace tasks
}  // namespace hqplanner

#endif