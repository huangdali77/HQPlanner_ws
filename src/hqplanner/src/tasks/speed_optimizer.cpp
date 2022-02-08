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

// void SpeedOptimizer::RecordDebugInfo(const SpeedData& speed_data) {
//   auto* debug = reference_line_info_->mutable_debug();
//   auto ptr_speed_plan = debug->mutable_planning_data()->add_speed_plan();
//   ptr_speed_plan->set_name(Name());
//   ptr_speed_plan->mutable_speed_point()->CopyFrom(
//       {speed_data.speed_vector().begin(), speed_data.speed_vector().end()});
// }

// void SpeedOptimizer::RecordSTGraphDebug(const StGraphData& st_graph_data,
//                                         STGraphDebug* st_graph_debug) const {
//   if (!FLAGS_enable_record_debug || !st_graph_debug) {
//     ADEBUG << "Skip record debug info";
//     return;
//   }

//   st_graph_debug->set_name(Name());
//   for (const auto& boundary : st_graph_data.st_boundaries()) {
//     auto boundary_debug = st_graph_debug->add_boundary();
//     boundary_debug->set_name(boundary->id());
//     switch (boundary->boundary_type()) {
//       case StBoundary::BoundaryType::FOLLOW:
//         boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_FOLLOW);
//         break;
//       case StBoundary::BoundaryType::OVERTAKE:
//         boundary_debug->set_type(
//             StGraphBoundaryDebug::ST_BOUNDARY_TYPE_OVERTAKE);
//         break;
//       case StBoundary::BoundaryType::STOP:
//         boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_STOP);
//         break;
//       case StBoundary::BoundaryType::UNKNOWN:
//         boundary_debug->set_type(
//             StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);
//         break;
//       case StBoundary::BoundaryType::YIELD:
//         boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_YIELD);
//         break;
//       case StBoundary::BoundaryType::KEEP_CLEAR:
//         boundary_debug->set_type(
//             StGraphBoundaryDebug::ST_BOUNDARY_TYPE_KEEP_CLEAR);
//         break;
//     }

//     for (const auto& point : boundary->points()) {
//       auto point_debug = boundary_debug->add_point();
//       point_debug->set_t(point.x());
//       point_debug->set_s(point.y());
//     }
//   }

//   for (const auto& point : st_graph_data.speed_limit().speed_limit_points())
//   {
//     common::SpeedPoint speed_point;
//     speed_point.set_s(point.first);
//     speed_point.set_v(point.second);
//     st_graph_debug->add_speed_limit()->CopyFrom(speed_point);
//   }

//   const auto& speed_data = reference_line_info_->speed_data();
//   st_graph_debug->mutable_speed_profile()->CopyFrom(
//       {speed_data.speed_vector().begin(), speed_data.speed_vector().end()});
// }

}  // namespace tasks
}  // namespace hqplanner
