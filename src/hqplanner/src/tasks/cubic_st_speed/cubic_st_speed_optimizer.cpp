#include "hqplanner/tasks/cubic_st_speed/cubic_st_speed_optimizer.h"

#include <ros/ros.h>

#include <algorithm>
#include <vector>

#include "hqplanner/tasks/cubic_st_speed/cubic_speed_profile_cost.h"
#include "hqplanner/tasks/poly_st_speed/speed_profile_cost.h"
#include "hqplanner/tasks/st_graph/speed_limit_decider.h"
#include "hqplanner/tasks/st_graph/st_boundary_mapper.h"
namespace hqplanner {
namespace tasks {

namespace {
constexpr double kEpsilon = 1e-6;
}

using hqplanner::PathDecision;
using hqplanner::ReferenceLine;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::PlanningConfig;
using hqplanner::forproto::SLBoundary;
using hqplanner::forproto::SpeedPoint;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::forproto::VehicleConfigHelper;
using hqplanner::math::CubicSplineClamped;
using hqplanner::path::PathData;
using hqplanner::speed::SpeedData;
using hqplanner::speed::SpeedLimit;
using hqplanner::speed::StBoundary;
using hqplanner::tasks::CubicSpeedProfileCost;
using hqplanner::tasks::SpeedProfileCost;
CubicStSpeedOptimizer::CubicStSpeedOptimizer()
    : SpeedOptimizer("CubicStSpeedOptimizer") {}

bool CubicStSpeedOptimizer::Init(const PlanningConfig& config) {
  cubic_st_speed_config_ = config.em_planner_config.cubic_st_speed_config;
  st_boundary_config_ = cubic_st_speed_config_.st_boundary_config;
  is_init_ = true;
  return true;
}

bool CubicStSpeedOptimizer::Process(const SLBoundary& adc_sl_boundary,
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

  SpeedLimitDecider speed_limit_decider(adc_sl_boundary, st_boundary_config_,
                                        reference_line, path_data);
  SpeedLimit speed_limits;
  if (!speed_limit_decider.GetSpeedLimits(path_decision->path_obstacles(),
                                          &speed_limits)) {
    return false;
  }

  // 获得dp_st_speed_optimizer计算的速度信息
  std::vector<SpeedPoint> reference_dp_speed_points =
      speed_data->speed_vector();
  std::vector<double> speed_anchor_point_t;
  std::vector<double> speed_anchor_point_s;
  for (const auto& speed_point : reference_dp_speed_points) {
    speed_anchor_point_t.emplace_back(speed_point.t);
    speed_anchor_point_s.emplace_back(speed_point.s);
  }

  // 计算最小代价的速度曲线

  CubicSplineClamped min_cost_curve;
  const double path_length = reference_dp_speed_points.back().s;
  const double path_endtime = reference_dp_speed_points.back().t;
  const double end_speed_limit = speed_limits.GetSpeedLimitByS(path_length);
  const double ds0 = init_point.v;
  constexpr int num_speed = 10;
  ROS_INFO("end_speed_limit:%f", end_speed_limit);
  // for (int i = 0; i < speed_anchor_point_t.size(); ++i) {
  //   ROS_INFO("t:%f, s:%f", speed_anchor_point_t[i], speed_anchor_point_s[i]);
  // }

  ROS_INFO(
      "path_obstacle_items()size:%d",
      static_cast<int>(
          reference_line_info_->path_decision()->path_obstacle_items().size()));

  // 实例化CubicSpeedProfileCost
  CubicSpeedProfileCost cost(
      cubic_st_speed_config_,
      reference_line_info_->path_decision()->path_obstacle_items(),
      speed_limits, init_point);
  double min_cost = std::numeric_limits<double>::max();

  for (double dsn = 0; dsn < end_speed_limit + kEpsilon;
       dsn += end_speed_limit / num_speed) {
    CubicSplineClamped curve(speed_anchor_point_t, speed_anchor_point_s, ds0,
                             dsn);

    double c = cost.Calculate(curve, path_endtime, min_cost);
    ROS_INFO("after Calculate");
    // if (c == std::numeric_limits<double>::infinity()) {
    //   ROS_INFO("!!!!!!!!!");
    // }
    // 计算规划起点的加速度与init_point.a的代价
    const double curve_start_a = curve.GetSplinePointSecondDerivativeValue(0);

    c += cubic_st_speed_config_.accelerate_diff_from_init_weight *
         std::pow((curve_start_a - init_point.a), 2);

    if (c < min_cost) {
      ROS_INFO("c < min_cost");
      min_cost_curve = curve;
      min_cost = c;
    }
  }

  if (min_cost >= std::numeric_limits<double>::max()) {
    assert(0);
    return false;
  }

  speed_data->Clear();

  const double t_output_resolution =
      cubic_st_speed_config_.trajectory_time_interval;
  double time = 0;
  while (time < path_endtime + kEpsilon) {
    ROS_INFO("==-=-=-==time:%f", time);
    double s = min_cost_curve.GetSplinePointValue(time);
    double v = min_cost_curve.GetSplinePointFirstDerivativeValue(time);
    double a = min_cost_curve.GetSplinePointSecondDerivativeValue(time);
    double da = min_cost_curve.GetSplinePointThirdDerivativeValue(time);
    speed_data->AppendSpeedPoint(s, time, v, a, da);
    time += t_output_resolution;
  }
  return true;
}

}  // namespace tasks
}  // namespace hqplanner