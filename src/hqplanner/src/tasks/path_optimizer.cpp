#include "hqplanner/tasks/path_optimizer.h"

#include "ros/ros.h"
namespace hqplanner {
namespace tasks {

PathOptimizer::PathOptimizer(const std::string& name) : Task(name) {}

bool PathOptimizer::Execute(Frame* frame,
                            ReferenceLineInfo* const reference_line_info) {
  Task::Execute(frame, reference_line_info);
  auto ret = Process(
      reference_line_info->speed_data(), reference_line_info->reference_line(),
      frame->PlanningStartPoint(), reference_line_info->mutable_path_data());

  if (!ret) {
    ROS_INFO("not drivabale reference line");
    reference_line_info->SetDrivable(false);
  }
  return ret;
}

}  // namespace tasks
}  // namespace hqplanner
