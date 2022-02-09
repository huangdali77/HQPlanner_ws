#include "hqplanner/tasks/task.h"

namespace hqplanner {
namespace tasks {
using hqplanner::Frame;
using hqplanner::ReferenceLineInfo;
using hqplanner::forproto::PlanningConfig;
Task::Task(const std::string& name) : name_(name) {}

const std::string& Task::Name() const { return name_; }

bool Task::Init(const PlanningConfig&) { return true; }

bool Task::Execute(Frame* frame, ReferenceLineInfo* reference_line_info) {
  frame_ = frame;
  reference_line_info_ = reference_line_info;
  return true;
}
}  // namespace tasks
}  // namespace hqplanner