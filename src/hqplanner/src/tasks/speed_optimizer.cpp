#include "hqplanner/tasks/speed_optimizer.h"

#include "hqplanner/speed/speed_limit.h"

namespace hqplanner {
namespace tasks {

SpeedOptimizer::SpeedOptimizer(const std::string& name) : Task(name) {}

bool SpeedOptimizer::Execute(Frame* frame,
                             ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);

  auto ret = Process(
      reference_line_info->AdcSlBoundary(), reference_line_info->path_data(),
      frame->PlanningStartPoint(), reference_line_info->reference_line(),
      *reference_line_info->mutable_speed_data(),
      reference_line_info->path_decision(),
      reference_line_info->mutable_speed_data());

  //   RecordDebugInfo(reference_line_info->speed_data());
  return ret;
}

}  // namespace tasks
}  // namespace hqplanner
