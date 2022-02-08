#ifndef HQPLANNER_TASKS_PATH_DECIDER_PATH_DECIDER_H_
#define HQPLANNER_TASKS_PATH_DECIDER_PATH_DECIDER_H_

#include <limits>
#include <string>

#include "hqplanner/common/path_decision.h"
#include "hqplanner/for_proto/decision.h"
#include "hqplanner/tasks/task.h"
namespace hqplanner {
namespace tasks {

class PathDecider : public Task {
 public:
  PathDecider();
  ~PathDecider() = default;

  bool Execute(Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:
  bool Process(const hqplanner::path::PathData &path_data,
               PathDecision *const path_decision);

  bool MakeObjectDecision(const hqplanner::path::PathData &path_data,
                          PathDecision *const path_decision);

  bool MakeStaticObstacleDecision(const hqplanner::path::PathData &path_data,
                                  PathDecision *const path_decision);

  forproto::ObjectStop GenerateObjectStopDecision(
      const PathObstacle &path_obstacle) const;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_PATH_DECIDER_PATH_DECIDER_H_
