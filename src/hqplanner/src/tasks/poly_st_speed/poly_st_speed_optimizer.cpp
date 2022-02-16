
#include "hqplanner/tasks/poly_st_speed/poly_st_speed_optimizer.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/vehicle_config_helper.h"
#include "hqplanner/for_proto/vehicle_state_provider.h"
#include "hqplanner/tasks/poly_st_speed/poly_st_graph.h"
#include "hqplanner/tasks/st_graph/speed_limit_decider.h"
#include "hqplanner/tasks/st_graph/st_graph_data.h"

namespace hqplanner {
namespace tasks {

using hqplanner::PathDecision;
using hqplanner::ReferenceLine;
using hqplanner::forproto::PlanningConfig;
using hqplanner::forproto::SLBoundary;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::path::PathData;
using hqplanner::speed::SpeedData;
using hqplanner::speed::SpeedLimit;
PolyStSpeedOptimizer::PolyStSpeedOptimizer()
    : SpeedOptimizer("PolyStSpeedOptimizer") {}

bool PolyStSpeedOptimizer::Init(const PlanningConfig& config) {
  if (is_init_) {
    // AERROR << "Duplicated Init.";
    return false;
  }
  poly_st_speed_config_ = config.em_planner_config.poly_st_speed_config;
  st_boundary_config_ = poly_st_speed_config_.st_boundary_config;
  is_init_ = true;
  return true;
}

bool PolyStSpeedOptimizer::Process(const SLBoundary& adc_sl_boundary,
                                   const PathData& path_data,
                                   const TrajectoryPoint& init_point,
                                   const ReferenceLine& reference_line,
                                   const SpeedData& reference_speed_data,
                                   PathDecision* const path_decision,
                                   SpeedData* const speed_data) {
  if (reference_line_info_->ReachedDestination()) {
    ROS_INFO("Please call Init() before process PolyStSpeedOptimizer.");
    assert(0);
    return true;
  }
  if (!is_init_) {
    // AERROR << "Please call Init() before Process.";
    return false;
  }

  if (path_data.discretized_path().NumOfPoints() == 0) {
    ROS_INFO("Empty path data");
    assert(0);
    return false;
  }

  StBoundaryMapper boundary_mapper(
      adc_sl_boundary, st_boundary_config_, reference_line, path_data,
      poly_st_speed_config_.total_path_length, poly_st_speed_config_.total_time,
      reference_line_info_->IsChangeLanePath());

  //   for (const auto* path_obstacle : path_decision->path_obstacles().Items())
  //   {
  //     DCHECK(path_obstacle->HasLongitudinalDecision());
  //   }
  // step 1 get boundaries
  path_decision->EraseStBoundaries();
  if (boundary_mapper.CreateStBoundary(path_decision) == false) {
    ROS_INFO("Mapping obstacle for dp st speed optimizer failed.");
    return false;
  }

  for (const auto* obstacle : path_decision->path_obstacle_items()) {
    auto id = obstacle->Id();
    auto* mutable_obstacle = path_decision->Find(id);

    if (!obstacle->st_boundary().IsEmpty()) {
      mutable_obstacle->SetBlockingObstacle(true);
    }
    // else {
    //   path_decision->SetStBoundary(
    //       id, path_decision->Find(id)->reference_line_st_boundary());
    // }
  }

  SpeedLimitDecider speed_limit_decider(adc_sl_boundary, st_boundary_config_,
                                        reference_line, path_data);
  SpeedLimit speed_limits;
  if (!speed_limit_decider.GetSpeedLimits(path_decision->path_obstacles(),
                                          &speed_limits)) {
    return false;
  }

  // step 2 perform graph search
  // make a poly_st_graph and perform search here.
  PolyStGraph poly_st_graph(poly_st_speed_config_, reference_line_info_,
                            speed_limits);
  auto ret = poly_st_graph.FindStTunnel(
      init_point, reference_line_info_->path_decision()->path_obstacle_items(),
      speed_data);
  if (!ret) {
    return false;
  }
  return true;
}

}  // namespace tasks
}  // namespace hqplanner
