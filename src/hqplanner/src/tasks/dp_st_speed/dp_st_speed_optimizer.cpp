#include "hqplanner/tasks/dp_st_speed/dp_st_speed_optimizer.h"

#include <ros/ros.h>

#include <algorithm>
#include <vector>

#include "hqplanner/for_proto/vehicle_config_helper.h"
#include "hqplanner/for_proto/vehicle_state_provider.h"
#include "hqplanner/tasks/dp_st_speed/dp_st_graph.h"
#include "hqplanner/tasks/st_graph/st_graph_data.h"

namespace hqplanner {
namespace tasks {
using hqplanner::PathDecision;
using hqplanner::ReferenceLine;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::PlanningConfig;
using hqplanner::forproto::SLBoundary;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::forproto::VehicleConfigHelper;
using hqplanner::path::PathData;
using hqplanner::speed::SpeedData;
using hqplanner::speed::SpeedLimit;
using hqplanner::speed::StBoundary;

DpStSpeedOptimizer::DpStSpeedOptimizer()
    : SpeedOptimizer("DpStSpeedOptimizer") {}

bool DpStSpeedOptimizer::Init(const PlanningConfig& config) {
  dp_st_speed_config_ = config.em_planner_config.dp_st_speed_config;
  st_boundary_config_ = dp_st_speed_config_.st_boundary_config;
  is_init_ = true;
  return true;
}

bool DpStSpeedOptimizer::SearchStGraph(
    const StBoundaryMapper& boundary_mapper,
    const SpeedLimitDecider& speed_limit_decider, const PathData& path_data,
    SpeedData* speed_data, PathDecision* path_decision) const {
  std::vector<const StBoundary*> boundaries;

  for (auto* obstacle : path_decision->path_obstacle_items()) {
    auto id = obstacle->Id();
    if (!obstacle->st_boundary().IsEmpty()) {
      if (obstacle->st_boundary().boundary_type() ==
          StBoundary::BoundaryType::KEEP_CLEAR) {
        path_decision->Find(id)->SetBlockingObstacle(false);
      } else {
        path_decision->Find(id)->SetBlockingObstacle(true);
      }
      boundaries.push_back(&obstacle->st_boundary());
    } else if (ConfigParam::instance()->FLAGS_enable_side_vehicle_st_boundary &&
               (adc_sl_boundary_.start_l > 2.0 ||
                adc_sl_boundary_.end_l < -2.0)) {
      if (path_decision->Find(id)->reference_line_st_boundary().IsEmpty()) {
        continue;
      }
      //   ADEBUG << "obstacle " << id << " is NOT blocking.";
      auto st_boundary_copy =
          path_decision->Find(id)->reference_line_st_boundary();
      auto st_boundary = st_boundary_copy.CutOffByT(3.5);
      if (!st_boundary.IsEmpty()) {
        auto decision = obstacle->LongitudinalDecision();
        if (decision.has_yield()) {
          st_boundary.SetBoundaryType(StBoundary::BoundaryType::YIELD);
        } else if (decision.has_overtake()) {
          st_boundary.SetBoundaryType(StBoundary::BoundaryType::OVERTAKE);
        } else if (decision.has_follow()) {
          st_boundary.SetBoundaryType(StBoundary::BoundaryType::FOLLOW);
        } else if (decision.has_stop()) {
          st_boundary.SetBoundaryType(StBoundary::BoundaryType::STOP);
        }
        st_boundary.SetId(st_boundary_copy.id());
        st_boundary.SetCharacteristicLength(
            st_boundary_copy.characteristic_length());

        path_decision->SetStBoundary(id, st_boundary);
        boundaries.push_back(&obstacle->st_boundary());
      }
    }
  }

  // step 2 perform graph search
  SpeedLimit speed_limit;
  if (!speed_limit_decider.GetSpeedLimits(path_decision->path_obstacles(),
                                          &speed_limit)) {
    ROS_INFO("Getting speed limits for dp st speed optimizer failed!");
    return false;
  }

  const float path_length = path_data.discretized_path().Length();
  StGraphData st_graph_data(boundaries, init_point_, speed_limit, path_length);

  DpStGraph st_graph(
      st_graph_data, dp_st_speed_config_,
      reference_line_info_->path_decision()->path_obstacle_items(), init_point_,
      adc_sl_boundary_);

  if (!st_graph.Search(speed_data)) {
    ROS_INFO("failed to search graph with dynamic programming.");
    assert(0);
    return false;
  }

  return true;
}

bool DpStSpeedOptimizer::Process(const SLBoundary& adc_sl_boundary,
                                 const PathData& path_data,
                                 const TrajectoryPoint& init_point,
                                 const ReferenceLine& reference_line,
                                 const SpeedData& reference_speed_data,
                                 PathDecision* const path_decision,
                                 SpeedData* const speed_data) {
  if (!is_init_) {
    ROS_INFO("Please call Init() before process DpStSpeedOptimizer.");
    assert(0);
    return false;
  }
  init_point_ = init_point;
  adc_sl_boundary_ = adc_sl_boundary;
  reference_line_ = &reference_line;

  if (path_data.discretized_path().NumOfPoints() == 0) {
    ROS_INFO("Empty path data");
    assert(0);
    return false;
  }

  StBoundaryMapper boundary_mapper(
      adc_sl_boundary, st_boundary_config_, *reference_line_, path_data,
      dp_st_speed_config_.total_path_length, dp_st_speed_config_.total_time,
      reference_line_info_->IsChangeLanePath());

  path_decision->EraseStBoundaries();
  if (!(boundary_mapper.CreateStBoundary(path_decision))) {
    ROS_INFO("Mapping obstacle for dp st speed optimizer failed.");
    return false;
  }

  SpeedLimitDecider speed_limit_decider(adc_sl_boundary, st_boundary_config_,
                                        *reference_line_, path_data);

  if (!SearchStGraph(boundary_mapper, speed_limit_decider, path_data,
                     speed_data, path_decision)) {
    ROS_INFO("Failed to search graph with dynamic programming.");

    return false;
  }
  return true;
}

}  // namespace tasks
}  // namespace hqplanner
