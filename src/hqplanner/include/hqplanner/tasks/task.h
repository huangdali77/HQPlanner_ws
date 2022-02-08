#ifndef HQPLANNER_TASKS_TASK_H_
#define HQPLANNER_TASKS_TASK_H_

#include <string>

#include "hqplanner/common/frame.h"
#include "hqplanner/for_proto/planning_config.h"
#include "hqplanner/reference_line/reference_line_info.h"

namespace hqplanner {
namespace tasks {

class Task {
 public:
  explicit Task(const std::string& name);
  virtual ~Task() = default;
  virtual const std::string& Name() const;

  virtual bool Execute(hqplanner::Frame* frame,
                       hqplanner::ReferenceLineInfo* reference_line_info);

  virtual bool Init(const hqplanner::forproto::PlanningConfig& config);

 protected:
  bool is_init_ = false;
  hqplanner::Frame* frame_ = nullptr;
  hqplanner::ReferenceLineInfo* reference_line_info_ = nullptr;

 private:
  const std::string name_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif