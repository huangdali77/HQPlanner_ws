#ifndef HQPLANNER_TASKS_PATH_OPTIMIZER_H_
#define HQPLANNER_TASKS_PATH_OPTIMIZER_H_

#include <string>

#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/reference_line/reference_line.h"
#include "hqplanner/speed/speed_data.h"
#include "hqplanner/tasks/task.h"
namespace hqplanner {
namespace tasks {

class PathOptimizer : public Task {
 public:
  explicit PathOptimizer(const std::string &name);
  virtual ~PathOptimizer() = default;
  bool Execute(hqplanner::Frame *frame,
               hqplanner::ReferenceLineInfo *reference_line_info) override;

 protected:
  virtual bool Process(const hqplanner::speed::SpeedData &speed_data,
                       const hqplanner::ReferenceLine &reference_line,
                       const hqplanner::forproto::TrajectoryPoint &init_point,
                       hqplanner::path::PathData *const path_data) = 0;
};

}  // namespace tasks
}  // namespace hqplanner

#endif
