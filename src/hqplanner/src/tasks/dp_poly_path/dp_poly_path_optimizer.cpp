#include "hqplanner/tasks/dp_poly_path/dp_poly_path_optimizer.h"

#include <assert.h>

#include <string>
#include <utility>
#include <vector>

#include "hqplanner/for_proto/planning_config.h"
#include "hqplanner/tasks/dp_poly_path/dp_road_graph.h"
#include "ros/ros.h"

namespace hqplanner {
namespace tasks {
using hqplanner::ReferenceLine;
using hqplanner::forproto::PlanningConfig;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::path::PathData;
using hqplanner::speed::SpeedData;
DpPolyPathOptimizer::DpPolyPathOptimizer()
    : PathOptimizer("DpPolyPathOptimizer") {}

bool DpPolyPathOptimizer::Init(const PlanningConfig &config) {
  config_ = config.em_planner_config.dp_poly_path_config;
  is_init_ = true;
  return true;
}

bool DpPolyPathOptimizer::Process(const SpeedData &speed_data,
                                  const ReferenceLine &reference_line,
                                  const TrajectoryPoint &init_point,
                                  PathData *const path_data) {
  if (!is_init_) {
    ROS_INFO("Please call Init() before Process().");
    return false;
  }
  assert(path_data != nullptr);
  // speed_data 为heuristic_speed_data
  DPRoadGraph dp_road_graph(config_, *reference_line_info_, speed_data);

  if (!dp_road_graph.FindPathTunnel(
          init_point,
          reference_line_info_->path_decision()->path_obstacle_items(),
          path_data)) {
    ROS_INFO("Failed to find tunnel in road graph");
    return false;
  }

  return true;
}

}  // namespace tasks
}  // namespace hqplanner
