#include "hqplanner/tasks/dp_poly_path/dp_road_graph.h"

#include <assert.h>
#include <ros/ros.h>

#include <algorithm>
#include <utility>

#include "hqplanner/for_proto/vehicle_config_helper.h"
#include "hqplanner/math/cartesian_frenet_conversion.h"
#include "hqplanner/math/curve1d/quintic_polynomial_curve1d.h"
#include "hqplanner/path/frenet_frame_path.h"
#include "hqplanner/util/util.h"
namespace hqplanner {
namespace tasks {

using hqplanner::PathObstacle;
using hqplanner::ReferenceLineInfo;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::DpPolyPathConfig;
using hqplanner::forproto::FrenetFramePoint;
using hqplanner::forproto::ObjectSidePass;
using hqplanner::forproto::ReferencePoint;
using hqplanner::forproto::SLPoint;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::forproto::VehicleConfigHelper;
using hqplanner::math::CartesianFrenetConverter;
using hqplanner::math::QuinticPolynomialCurve1d;
using hqplanner::path::FrenetFramePath;
using hqplanner::path::PathData;
using hqplanner::speed::SpeedData;
using hqplanner::util::MakeSLPoint;

DPRoadGraph::DPRoadGraph(const DpPolyPathConfig &config,
                         const ReferenceLineInfo &reference_line_info,
                         const SpeedData &speed_data)
    : config_(config),
      reference_line_info_(reference_line_info),
      reference_line_(reference_line_info.reference_line()),
      speed_data_(speed_data) {}

bool DPRoadGraph::FindPathTunnel(
    const TrajectoryPoint &init_point,
    const std::vector<const PathObstacle *> &obstacles,
    PathData *const path_data) {
  assert(path_data != nullptr);

  init_point_ = init_point;
  if (!reference_line_.XYToSL(
          {init_point_.path_point.x, init_point_.path_point.y},
          &init_sl_point_)) {
    return false;
  }

  if (!CalculateFrenetPoint(init_point_, &init_frenet_frame_point_)) {
    ROS_INFO("Fail to create init_frenet_frame_point_ at : (%f, %f)",
             init_point_.path_point.x, init_point_.path_point.y);

    return false;
  }

  std::vector<DPRoadGraphNode> min_cost_path;
  if (!GenerateMinCostPath(obstacles, &min_cost_path)) {
    ROS_INFO("Fail to generate graph!");

    return false;
  }
  // 将dp得到的多段路径按特定步长采点FrenetFramePoint存放到frenet_path
  std::vector<FrenetFramePoint> frenet_path;
  float accumulated_s = init_sl_point_.s;
  const float path_resolution = config_.path_resolution;  // 1m

  for (std::size_t i = 1; i < min_cost_path.size(); ++i) {
    const auto &prev_node = min_cost_path[i - 1];
    const auto &cur_node = min_cost_path[i];

    const float path_length = cur_node.sl_point.s - prev_node.sl_point.s;
    float current_s = 0.0;
    const auto &curve = cur_node.min_cost_curve;
    while (current_s + path_resolution / 2.0 < path_length) {
      const float l = curve.Evaluate(0, current_s);
      const float dl = curve.Evaluate(1, current_s);
      const float ddl = curve.Evaluate(2, current_s);
      FrenetFramePoint frenet_frame_point;
      frenet_frame_point.s = accumulated_s + current_s;
      frenet_frame_point.l = l;
      frenet_frame_point.dl = dl;
      frenet_frame_point.ddl = ddl;
      frenet_path.push_back(std::move(frenet_frame_point));
      current_s += path_resolution;
    }
    if (i == min_cost_path.size() - 1) {
      accumulated_s += current_s;
    } else {
      accumulated_s += path_length;
    }
  }
  FrenetFramePath tunnel(frenet_path);
  path_data->SetReferenceLine(&reference_line_);
  //设置path_data中的frenet_path_和discretized_path_
  path_data->SetFrenetPath(tunnel);
  return true;
}

bool DPRoadGraph::GenerateMinCostPath(
    const std::vector<const PathObstacle *> &obstacles,
    std::vector<DPRoadGraphNode> *min_cost_path) {
  assert(min_cost_path != nullptr);

  std::vector<std::vector<SLPoint>> path_waypoints;
  if (!SamplePathWaypoints(init_point_, &path_waypoints) ||
      path_waypoints.size() < 1) {
    ROS_INFO("Fail to sample path waypoints! reference_line_length = %f",
             reference_line_.Length());

    return false;
  }
  path_waypoints.insert(path_waypoints.begin(),
                        std::vector<SLPoint>{init_sl_point_});
  const auto &vehicle_config = VehicleConfigHelper::instance()->GetConfig();

  TrajectoryCost trajectory_cost(
      config_, reference_line_, reference_line_info_.IsChangeLanePath(),
      obstacles, vehicle_config.vehicle_param, speed_data_, init_sl_point_);

  std::list<std::list<DPRoadGraphNode>> graph_nodes;
  graph_nodes.emplace_back();
  graph_nodes.back().emplace_back(init_sl_point_, nullptr, ComparableCost());
  auto &front = graph_nodes.front().front();
  size_t total_level = path_waypoints.size();

  for (std::size_t level = 1; level < path_waypoints.size(); ++level) {
    const auto &prev_dp_nodes = graph_nodes.back();
    const auto &level_points = path_waypoints[level];

    graph_nodes.emplace_back();

    for (size_t i = 0; i < level_points.size(); ++i) {
      const auto &cur_point = level_points[i];

      graph_nodes.back().emplace_back(cur_point, nullptr);
      auto &cur_node = graph_nodes.back().back();
      // if (FLAGS_enable_multi_thread_in_dp_poly_path) {
      //   PlanningThreadPool::instance()->Push(std::bind(
      //       &DPRoadGraph::UpdateNode, this, std::ref(prev_dp_nodes), level,
      //       total_level, &trajectory_cost, &(front), &(cur_node)));

      // } else {
      UpdateNode(prev_dp_nodes, level, total_level, &trajectory_cost, &front,
                 &cur_node);
      // }
    }
    // if (FLAGS_enable_multi_thread_in_dp_poly_path) {
    //   PlanningThreadPool::instance()->Synchronize();
    // }
  }

  // find best path
  DPRoadGraphNode fake_head;
  for (const auto &cur_dp_node : graph_nodes.back()) {
    fake_head.UpdateCost(&cur_dp_node, cur_dp_node.min_cost_curve,
                         cur_dp_node.min_cost);
  }

  const auto *min_cost_node = &fake_head;
  while (min_cost_node->min_cost_prev_node) {
    min_cost_node = min_cost_node->min_cost_prev_node;
    min_cost_path->push_back(*min_cost_node);
  }
  if (min_cost_node != &graph_nodes.front().front()) {
    return false;
  }

  std::reverse(min_cost_path->begin(), min_cost_path->end());

  // for (const auto &node : *min_cost_path) {
  //   ADEBUG << "min_cost_path: " << node.sl_point.ShortDebugString();
  //   planning_debug_->mutable_planning_data()
  //       ->mutable_dp_poly_graph()
  //       ->add_min_cost_point()
  //       ->CopyFrom(node.sl_point);
  // }
  return true;
}

void DPRoadGraph::UpdateNode(const std::list<DPRoadGraphNode> &prev_nodes,
                             const uint32_t level, const uint32_t total_level,
                             TrajectoryCost *trajectory_cost,
                             DPRoadGraphNode *front,
                             DPRoadGraphNode *cur_node) {
  assert(trajectory_cost != nullptr);
  assert(front != nullptr);
  assert(cur_node != nullptr);

  for (const auto &prev_dp_node : prev_nodes) {
    const auto &prev_sl_point = prev_dp_node.sl_point;
    const auto &cur_point = cur_node->sl_point;
    float init_dl = 0.0;
    float init_ddl = 0.0;
    if (level == 1) {
      init_dl = init_frenet_frame_point_.dl;
      init_ddl = init_frenet_frame_point_.ddl;
    }
    QuinticPolynomialCurve1d curve(prev_sl_point.l, init_dl, init_ddl,
                                   cur_point.l, 0.0, 0.0,
                                   cur_point.s - prev_sl_point.s);

    if (!IsValidCurve(curve)) {
      continue;
    }
    const auto cost =
        trajectory_cost->Calculate(curve, prev_sl_point.s, cur_point.s, level,
                                   total_level) +
        prev_dp_node.min_cost;

    cur_node->UpdateCost(&prev_dp_node, curve, cost);
  }

  // try to connect the current point with the first point directly
  // 尝试直接连接当前采样点和规划路径初始点
  if (level >= 2) {
    const float init_dl = init_frenet_frame_point_.dl;
    const float init_ddl = init_frenet_frame_point_.ddl;
    QuinticPolynomialCurve1d curve(init_sl_point_.l, init_dl, init_ddl,
                                   cur_node->sl_point.l, 0.0, 0.0,
                                   cur_node->sl_point.s - init_sl_point_.s);
    if (!IsValidCurve(curve)) {
      return;
    }
    const auto cost = trajectory_cost->Calculate(
        curve, init_sl_point_.s, cur_node->sl_point.s, level, total_level);
    cur_node->UpdateCost(front, curve, cost);
  }
}

bool DPRoadGraph::SamplePathWaypoints(
    const TrajectoryPoint &init_point,
    std::vector<std::vector<SLPoint>> *const points) {
  assert(points != nullptr);

  const float kMinSampleDistance = 40.0;
  // 1、确定规划路径的长度，规划路径的长度为8s或40m
  const float total_length = std::fmin(
      init_sl_point_.s + std::fmax(init_point.v * 8.0, kMinSampleDistance),
      reference_line_.Length());
  const auto &vehicle_config = VehicleConfigHelper::instance()->GetConfig();
  const float half_adc_width = vehicle_config.vehicle_param.width / 2.0;
  // 2、确定路径动态规划的横向采样点数，num_sample_per_level = 7
  const size_t num_sample_per_level =
      ConfigParam::instance()->FLAGS_use_navigation_mode
          ? config_.navigator_sample_num_each_level
          : config_.sample_points_num_each_level;

  // 3、确定路径动态规划的采样步长
  constexpr float kSamplePointLookForwardTime = 4.0;
  // step_length=[8m,...speed x 4s...,15m]
  const float step_length =
      hqplanner::math::Clamp(init_point.v * kSamplePointLookForwardTime,
                             config_.step_length_min, config_.step_length_max);
  const float level_distance =
      (init_point.v > ConfigParam::instance()->FLAGS_max_stop_speed)
          ? step_length
          : step_length / 2.0;
  float accumulated_s = init_sl_point_.s;
  float prev_s = accumulated_s;

  for (std::size_t i = 0; accumulated_s < total_length; ++i) {
    accumulated_s += level_distance;
    if (accumulated_s + level_distance / 2.0 > total_length) {
      accumulated_s = total_length;
    }
    const float s = std::fmin(accumulated_s, total_length);
    constexpr float kMinAllowedSampleStep = 1.0;
    if (std::fabs(s - prev_s) < kMinAllowedSampleStep) {
      continue;
    }
    prev_s = s;

    //设置道路的左右boundary
    double left_width = 0.0;
    double right_width = 0.0;
    reference_line_.GetLaneWidth(s, &left_width, &right_width);

    constexpr float kBoundaryBuff = 0.0;  // 0.20;
    const float eff_right_width = right_width - half_adc_width - kBoundaryBuff;
    const float eff_left_width = left_width - half_adc_width - kBoundaryBuff;

    float kDefaultUnitL = 1.2 / (num_sample_per_level - 1);  // 1.2/6=0.2m
    if (reference_line_info_.IsChangeLanePath() &&
        !reference_line_info_.IsSafeToChangeLane()) {
      kDefaultUnitL = 1.0;
    }
    const float sample_l_range =
        kDefaultUnitL *
        (num_sample_per_level - 1);  // 1.2m或6m(换道时的横向采样范围大)
    float sample_right_boundary = -eff_right_width;
    float sample_left_boundary = eff_left_width;

    const float kLargeDeviationL = 1.75;
    if (reference_line_info_.IsChangeLanePath() ||
        std::fabs(init_sl_point_.l) > kLargeDeviationL) {
      sample_right_boundary = std::fmin(-eff_right_width, init_sl_point_.l);
      sample_left_boundary = std::fmax(eff_left_width, init_sl_point_.l);

      if (init_sl_point_.l > eff_left_width) {
        ROS_INFO("adc out road left boundary ");
        // 测试合适出现这种情况
        // assert(0);
        sample_right_boundary =
            std::fmax(sample_right_boundary, init_sl_point_.l - sample_l_range);
      }
      if (init_sl_point_.l < -eff_right_width) {
        ROS_INFO("adc out road right boundary ");
        ROS_INFO("init_sl_point_.l: %f eff_right_width: %f", init_sl_point_.l,
                 eff_right_width);
        // 测试合适出现这种情况
        // assert(0);
        sample_left_boundary =
            std::fmin(sample_left_boundary, init_sl_point_.l + sample_l_range);
      }
    }

    // =====================NoNeed=================================
    const bool has_sidepass = HasSidepass();

    std::vector<float> sample_l;  //横向采样点的横向坐标
    if (reference_line_info_.IsChangeLanePath() &&
        !reference_line_info_.IsSafeToChangeLane()) {
      sample_l.push_back(reference_line_info_.OffsetToOtherReferenceLine());
    } else if (has_sidepass) {
      ROS_INFO("has sidepass decisoin");
      // currently only left nudge is supported. Need road hard boundary for
      // both sides
      switch (sidepass_.type) {
        case ObjectSidePass::LEFT: {
          sample_l.push_back(eff_left_width + config_.sidepass_distance);
          break;
        }
        case ObjectSidePass::RIGHT: {
          sample_l.push_back(-eff_right_width - config_.sidepass_distance);
          break;
        }
        default:
          break;
      }
    } else {
      hqplanner::util::uniform_slice(sample_right_boundary,
                                     sample_left_boundary,
                                     num_sample_per_level - 1, &sample_l);
    }
    std::vector<SLPoint> level_points;  //横向采样点在frenet坐标系下的坐标
    // planning_internal::SampleLayerDebug sample_layer_debug;
    for (size_t j = 0; j < sample_l.size(); ++j) {
      SLPoint sl = hqplanner::util::MakeSLPoint(s, sample_l[j]);
      // sample_layer_debug.add_sl_point()->CopyFrom(sl);
      level_points.push_back(std::move(sl));
    }
    if (!reference_line_info_.IsChangeLanePath() && has_sidepass) {
      auto sl_zero = hqplanner::util::MakeSLPoint(s, 0.0);
      // sample_layer_debug.add_sl_point()->CopyFrom(sl_zero);
      level_points.push_back(std::move(sl_zero));
    }

    if (!level_points.empty()) {
      // planning_debug_->mutable_planning_data()
      //     ->mutable_dp_poly_graph()
      //     ->add_sample_layer()
      //     ->CopyFrom(sample_layer_debug);
      points->emplace_back(level_points);
    }
  }
  return true;
}

bool DPRoadGraph::CalculateFrenetPoint(
    const TrajectoryPoint &traj_point,
    FrenetFramePoint *const frenet_frame_point) {
  SLPoint sl_point;
  if (!reference_line_.XYToSL(
          {traj_point.path_point.x, traj_point.path_point.y}, &sl_point)) {
    return false;
  }
  frenet_frame_point->s = sl_point.s;
  frenet_frame_point->l = sl_point.l;

  const float theta = traj_point.path_point.theta;
  const float kappa = traj_point.path_point.kappa;
  const float l = frenet_frame_point->l;

  ReferencePoint ref_point;
  ref_point = reference_line_.GetReferencePoint(frenet_frame_point->s);

  const float theta_ref = ref_point.heading;
  const float kappa_ref = ref_point.kappa;
  const float dkappa_ref = ref_point.dkappa;

  const float dl = CartesianFrenetConverter::CalculateLateralDerivative(
      theta_ref, theta, l, kappa_ref);
  const float ddl =
      CartesianFrenetConverter::CalculateSecondOrderLateralDerivative(
          theta_ref, theta, kappa_ref, kappa, dkappa_ref, l);
  frenet_frame_point->dl = dl;
  frenet_frame_point->ddl = ddl;

  return true;
}

bool DPRoadGraph::IsValidCurve(const QuinticPolynomialCurve1d &curve) const {
  constexpr float kMaxLateralDistance = 20.0;
  for (float s = 0.0; s < curve.ParamLength(); s += 2.0) {
    const float l = curve.Evaluate(0, s);
    if (std::fabs(l) > kMaxLateralDistance) {
      return false;
    }
  }
  return true;
}

void DPRoadGraph::GetCurveCost(TrajectoryCost trajectory_cost,
                               const QuinticPolynomialCurve1d &curve,
                               const float start_s, const float end_s,
                               const uint32_t curr_level,
                               const uint32_t total_level,
                               ComparableCost *cost) {
  *cost =
      trajectory_cost.Calculate(curve, start_s, end_s, curr_level, total_level);
}
// ==============NoNeed================================
bool DPRoadGraph::HasSidepass() {
  const auto &path_decision = reference_line_info_.path_decision();
  for (const auto &obstacle : path_decision.path_obstacle_items()) {
    if (obstacle->LateralDecision().has_sidepass()) {
      sidepass_ = obstacle->LateralDecision().sidepass();
      return true;
    }
  }
  return false;
}

}  // namespace tasks
}  // namespace hqplanner
