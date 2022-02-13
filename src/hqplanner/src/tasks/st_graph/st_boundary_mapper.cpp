
#include "hqplanner/tasks/st_graph/st_boundary_mapper.h"

#include <assert.h>
#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>

#include "hqplanner/common/frame.h"
#include "hqplanner/for_proto/decision.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/vehicle_config_helper.h"
#include "hqplanner/for_proto/vehicle_state_provider.h"
#include "hqplanner/math/line_segment2d.h"
#include "hqplanner/math/vec2d.h"
#include "hqplanner/path/discretized_path.h"
#include "hqplanner/util/util.h"

namespace hqplanner {

namespace tasks {
using hqplanner::Obstacle;
using hqplanner::PathDecision;
using hqplanner::PathObstacle;
using hqplanner::forproto::ObjectDecisionType;
using hqplanner::forproto::PathPoint;
using hqplanner::forproto::SLBoundary;
using hqplanner::forproto::SLPoint;
using hqplanner::forproto::StBoundaryConfig;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::forproto::VehicleConfigHelper;
using hqplanner::forproto::VehicleParam;
using hqplanner::math::Box2d;
using hqplanner::math::Vec2d;
using hqplanner::path::DiscretizedPath;
using hqplanner::path::PathData;
using hqplanner::speed::StBoundary;
using hqplanner::speed::STPoint;

namespace {
constexpr double boundary_t_buffer = 0.1;
constexpr double boundary_s_buffer = 1.0;
}  // namespace

StBoundaryMapper::StBoundaryMapper(const SLBoundary& adc_sl_boundary,
                                   const StBoundaryConfig& config,
                                   const ReferenceLine& reference_line,
                                   const PathData& path_data,
                                   const double planning_distance,
                                   const double planning_time,
                                   bool is_change_lane)
    : adc_sl_boundary_(adc_sl_boundary),
      st_boundary_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      vehicle_param_(
          VehicleConfigHelper::instance()->GetConfig().vehicle_param),
      planning_distance_(planning_distance),
      planning_time_(planning_time),
      is_change_lane_(is_change_lane) {}

bool StBoundaryMapper::CreateStBoundary(PathDecision* path_decision) const {
  if (planning_time_ < 0.0) {
    // planning_time_ = 7.0
    ROS_INFO("Fail to get params since planning_time_ < 0.");
    assert(0);
    return false;
  }

  if (path_data_.discretized_path().NumOfPoints() < 2) {
    ROS_INFO("Fail because of too few path points. path points size : % d ",
             path_data_.discretized_path().NumOfPoints());
    assert(0);

    return false;
  }

  PathObstacle* stop_obstacle = nullptr;
  ObjectDecisionType stop_decision;
  double min_stop_s = std::numeric_limits<double>::max();
  const auto& path_obstacles = path_decision->path_obstacle_items();

  for (const auto* const_path_obstacle : path_obstacles) {
    auto* path_obstacle = path_decision->Find(const_path_obstacle->Id());
    // 处理没有决策标签的障碍物
    if (!path_obstacle->HasLongitudinalDecision()) {
      if (!MapWithoutDecision(path_obstacle)) {
        // MapWithoutDecision失败说明无人车的规划路径与障碍物的预测轨迹没有干涉，此时可以对障碍物进行ignore决策
        ObjectDecisionType object_decision;
        object_decision.object_tag = ObjectDecisionType::ObjectTag::IGNORE;
        path_decision->AddLateralDecision("PathDecider/not-has-overlap",
                                          path_obstacle->obstacle()->Id(),
                                          object_decision);
        path_decision->AddLongitudinalDecision("PathDecider/not-has-overlap",
                                               path_obstacle->obstacle()->Id(),
                                               object_decision);
      }

      continue;
    }

    // 处理有决策标签的障碍物（一般是自行车、行人和静止障碍物）
    const auto& decision = path_obstacle->LongitudinalDecision();
    if (decision.has_stop()) {
      const double stop_s = (path_obstacle->PerceptionSLBoundary()).start_s +
                            decision.stop().distance_s;
      // this is a rough estimation based on reference line s, so that a large
      // buffer is used.
      constexpr double stop_buff = 1.0;
      if (stop_s + stop_buff < adc_sl_boundary_.end_s) {
        ROS_INFO("Invalid stop decision");

        return false;
      }
      if (stop_s < min_stop_s) {
        stop_obstacle = path_obstacle;
        min_stop_s = stop_s;
        stop_decision = decision;
      }
    } else if (decision.has_follow() || decision.has_overtake() ||
               decision.has_yield()) {
      if (!MapWithDecision(path_obstacle, decision)) {
        // MapWithoutDecision失败说明无人车的规划路径与障碍物的预测轨迹没有干涉，此时可以对障碍物进行ignore决策
        ObjectDecisionType object_decision;
        object_decision.object_tag = ObjectDecisionType::ObjectTag::IGNORE;
        path_decision->AddLateralDecision("PathDecider/not-has-overlap",
                                          path_obstacle->obstacle()->Id(),
                                          object_decision);
        path_decision->AddLongitudinalDecision("PathDecider/not-has-overlap",
                                               path_obstacle->obstacle()->Id(),
                                               object_decision);
      }
    }
  }

  if (stop_obstacle) {
    bool success = MapStopDecision(stop_obstacle, stop_decision);
    if (!success) {
      return false;
    }
  }
  return true;
}

// Status StBoundaryMapper::CreateStBoundaryWithHistory(
//     const ObjectDecisions& history_decisions,
//     PathDecision* path_decision) const {
//   const auto& path_obstacles = path_decision->path_obstacles();
//   if (planning_time_ < 0.0) {
//     const std::string msg = "Fail to get params since planning_time_ < 0.";
//     AERROR << msg;
//     return Status(ErrorCode::PLANNING_ERROR, msg);
//   }

//   if (path_data_.discretized_path().NumOfPoints() < 2) {
//     AERROR << "Fail to get params because of too few path points. path points
//     "
//               "size: "
//            << path_data_.discretized_path().NumOfPoints() << ".";
//     return Status(ErrorCode::PLANNING_ERROR,
//                   "Fail to get params because of too few path points");
//   }

//   std::unordered_map<std::string, ObjectDecisionType> prev_decision_map;
//   for (const auto& history_decision : history_decisions.decision()) {
//     for (const auto& decision : history_decision.object_decision()) {
//       if (PathObstacle::IsLongitudinalDecision(decision) &&
//           !decision.has_ignore()) {
//         prev_decision_map[history_decision.id()] = decision;
//         break;
//       }
//     }
//   }

//   PathObstacle* stop_obstacle = nullptr;
//   ObjectDecisionType stop_decision;
//   double min_stop_s = std::numeric_limits<double>::max();

//   for (const auto* const_path_obstacle : path_obstacles.Items()) {
//     auto* path_obstacle = path_decision->Find(const_path_obstacle->Id());
//     auto iter = prev_decision_map.find(path_obstacle->Id());
//     ObjectDecisionType decision;
//     if (iter == prev_decision_map.end()) {
//       decision.mutable_ignore();
//     } else {
//       decision = iter->second;
//     }

//     if (!path_obstacle->HasLongitudinalDecision()) {
//       if (!MapWithoutDecision(path_obstacle).ok()) {
//         std::string msg = StrCat("Fail to map obstacle ",
//         path_obstacle->Id(),
//                                  " without decision.");
//         AERROR << msg;
//         return Status(ErrorCode::PLANNING_ERROR, msg);
//       }
//       if (path_obstacle->st_boundary().IsEmpty() || decision.has_ignore()) {
//         continue;
//       }
//     }
//     if (path_obstacle->HasLongitudinalDecision()) {
//       decision = path_obstacle->LongitudinalDecision();
//     }
//     if (decision.has_stop()) {
//       const double stop_s = path_obstacle->PerceptionSLBoundary().start_s() +
//                             decision.stop().distance_s();
//       // this is a rough estimation based on reference line s, so that a
//       large
//       // buffer is used.
//       constexpr double stop_buff = 1.0;
//       if (stop_s + stop_buff < adc_sl_boundary_.end_s()) {
//         AERROR << "Invalid stop decision. not stop at behind of current "
//                   "position. stop_s : "
//                << stop_s << ", and current adc_s is; "
//                << adc_sl_boundary_.end_s();
//         return Status(ErrorCode::PLANNING_ERROR, "invalid decision");
//       }
//       if (stop_s < min_stop_s) {
//         stop_obstacle = path_obstacle;
//         min_stop_s = stop_s;
//         stop_decision = decision;
//       }
//     } else if (decision.has_follow() || decision.has_overtake() ||
//                decision.has_yield()) {
//       if (!MapWithDecision(path_obstacle, decision).ok()) {
//         AERROR << "Fail to map obstacle " << path_obstacle->Id()
//                << " with decision: " << decision.DebugString();
//         return Status(ErrorCode::PLANNING_ERROR,
//                       "Fail to map overtake/yield decision");
//       }
//     } else {
//       AWARN << "No mapping for decision: " << decision.DebugString();
//     }
//   }

//   if (stop_obstacle) {
//     bool success = MapStopDecision(stop_obstacle, stop_decision);
//     if (!success) {
//       std::string msg = "Fail to MapStopDecision.";
//       AERROR << msg;
//       return Status(ErrorCode::PLANNING_ERROR, msg);
//     }
//   }
//   return Status::OK();
// }

bool StBoundaryMapper::MapStopDecision(
    PathObstacle* stop_obstacle,
    const ObjectDecisionType& stop_decision) const {
  assert(stop_decision.has_stop());

  if (stop_obstacle->PerceptionSLBoundary().start_s >
      adc_sl_boundary_.end_s + planning_distance_) {
    return true;
  }

  double st_stop_s = 0.0;
  const double stop_ref_s = stop_obstacle->PerceptionSLBoundary().start_s +
                            stop_decision.stop().distance_s -
                            vehicle_param_.front_edge_to_center;

  if (stop_ref_s > path_data_.frenet_frame_path().points().back().s) {
    st_stop_s = path_data_.discretized_path().EndPoint().s +
                (stop_ref_s - path_data_.frenet_frame_path().points().back().s);
  } else {
    PathPoint stop_point;
    if (!path_data_.GetPathPointWithRefS(stop_ref_s, &stop_point)) {
      ROS_INFO("Fail to get path point from reference s");
      assert(0);
      // AERROR << ". The sl boundary of
      // "
      //           "stop obstacle "
      //        << stop_obstacle->Id()
      //        << " is: " <<
      //        stop_obstacle->PerceptionSLBoundary().DebugString();
      return false;
    }

    st_stop_s = stop_point.s;
  }

  constexpr double kStopEpsilon = 1e-2;
  const double s_min = std::max(0.0, st_stop_s - kStopEpsilon);
  const double s_max =
      std::fmax(s_min, std::fmax(planning_distance_, reference_line_.Length()));

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  point_pairs.emplace_back(STPoint(s_min, 0.0), STPoint(s_max, 0.0));
  point_pairs.emplace_back(
      STPoint(s_min, planning_time_),
      STPoint(s_max + st_boundary_config_.boundary_buffer, planning_time_));
  auto boundary = StBoundary(point_pairs);
  boundary.SetBoundaryType(StBoundary::BoundaryType::STOP);
  boundary.SetCharacteristicLength(st_boundary_config_.boundary_buffer);
  boundary.SetId(stop_obstacle->Id());
  stop_obstacle->SetStBoundary(boundary);
  return true;
}

bool StBoundaryMapper::MapWithoutDecision(PathObstacle* path_obstacle) const {
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  if (!GetOverlapBoundaryPoints(path_data_.discretized_path().path_points(),
                                *(path_obstacle->obstacle()), &upper_points,
                                &lower_points)) {
    return false;
  }
  // 将障碍物的StBoundary存放到PathObstacle中
  auto boundary = StBoundary::GenerateStBoundary(lower_points, upper_points)
                      .ExpandByS(boundary_s_buffer)
                      .ExpandByT(boundary_t_buffer);
  boundary.SetId(path_obstacle->Id());
  const auto& prev_st_boundary = path_obstacle->st_boundary();
  const auto& ref_line_st_boundary =
      path_obstacle->reference_line_st_boundary();
  if (!prev_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(prev_st_boundary.boundary_type());
  } else if (!ref_line_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(ref_line_st_boundary.boundary_type());
  }

  path_obstacle->SetStBoundary(boundary);
  return true;  ///????????????????????
}

bool StBoundaryMapper::GetOverlapBoundaryPoints(
    const std::vector<PathPoint>& path_points, const Obstacle& obstacle,
    std::vector<STPoint>* upper_points,
    std::vector<STPoint>* lower_points) const {
  assert(upper_points != nullptr);
  assert(lower_points != nullptr);
  upper_points->clear();
  lower_points->clear();

  // assert(path_points.size() > 0);

  if (path_points.empty()) {
    return false;
  }

  const auto& trajectory = obstacle.Trajectory();
  if (trajectory.trajectory_point.size() == 0) {
    // 处理静态障碍物
    if (!obstacle.IsStatic()) {
      ROS_INFO("Non-static obstacle[%s] has NO prediction trajectory.",
               obstacle.Id());
    }
    for (const auto& curr_point_on_path : path_points) {
      if (curr_point_on_path.s > planning_distance_) {
        break;
      }
      const Box2d obs_box = obstacle.PerceptionBoundingBox();

      if (CheckOverlap(curr_point_on_path, obs_box,
                       st_boundary_config_.boundary_buffer)) {
        const double backward_distance = -vehicle_param_.front_edge_to_center;
        const double forward_distance = vehicle_param_.length +
                                        vehicle_param_.width +
                                        obs_box.length() + obs_box.width();
        double low_s = std::fmax(0.0, curr_point_on_path.s + backward_distance);
        double high_s = std::fmin(planning_distance_,
                                  curr_point_on_path.s + forward_distance);
        lower_points->emplace_back(low_s, 0.0);
        lower_points->emplace_back(low_s, planning_time_);
        upper_points->emplace_back(high_s, 0.0);
        upper_points->emplace_back(high_s, planning_time_);
        break;
      }
    }
  } else {
    //处理动态障碍物
    const int default_num_point = 50;
    DiscretizedPath discretized_path;  // size为[50,99)
    if (path_points.size() > 2 * default_num_point) {
      const int ratio = path_points.size() / default_num_point;
      std::vector<PathPoint> sampled_path_points;
      for (size_t i = 0; i < path_points.size(); ++i) {
        if (i % ratio == 0) {
          sampled_path_points.push_back(path_points[i]);
        }
      }
      discretized_path.set_path_points(sampled_path_points);
    } else {
      discretized_path.set_path_points(path_points);
    }
    // 遍历障碍物的预测轨迹
    for (int i = 0; i < trajectory.trajectory_point.size(); ++i) {
      const auto& trajectory_point = (trajectory.trajectory_point)[i];
      const Box2d obs_box = obstacle.GetBoundingBox(trajectory_point);

      double trajectory_point_time = trajectory_point.relative_time;
      constexpr double kNegtiveTimeThreshold = -1.0;
      if (trajectory_point_time < kNegtiveTimeThreshold) {
        continue;
      }

      // 以front_edge_to_center为步长遍历adc的路径
      const double step_length = vehicle_param_.front_edge_to_center;
      for (double path_s = 0.0; path_s < discretized_path.Length();
           path_s += step_length) {
        const auto curr_adc_path_point =
            discretized_path.Evaluate(path_s + discretized_path.StartPoint().s);
        if (CheckOverlap(curr_adc_path_point, obs_box,
                         st_boundary_config_.boundary_buffer)) {
          // found overlap, start searching with higher resolution
          const double backward_distance = -step_length;
          const double forward_distance = vehicle_param_.length +
                                          vehicle_param_.width +
                                          obs_box.length() + obs_box.width();
          const double default_min_step = 0.1;  // in meters
          const double fine_tuning_step_length = std::fmin(
              default_min_step, discretized_path.Length() / default_num_point);

          bool find_low = false;
          bool find_high = false;
          double low_s = std::fmax(0.0, path_s + backward_distance);
          double high_s =
              std::fmin(discretized_path.Length(), path_s + forward_distance);

          while (low_s < high_s) {
            if (find_low && find_high) {
              break;
            }
            if (!find_low) {
              const auto& point_low = discretized_path.Evaluate(
                  low_s + discretized_path.StartPoint().s);
              if (!CheckOverlap(point_low, obs_box,
                                st_boundary_config_.boundary_buffer)) {
                low_s += fine_tuning_step_length;
              } else {
                find_low = true;
              }
            }
            if (!find_high) {
              const auto& point_high = discretized_path.Evaluate(
                  high_s + discretized_path.StartPoint().s);
              if (!CheckOverlap(point_high, obs_box,
                                st_boundary_config_.boundary_buffer)) {
                high_s -= fine_tuning_step_length;
              } else {
                find_high = true;
              }
            }
          }
          if (find_high && find_low) {
            lower_points->emplace_back(
                low_s - st_boundary_config_.point_extension,
                trajectory_point_time);
            upper_points->emplace_back(
                high_s + st_boundary_config_.point_extension,
                trajectory_point_time);
          }
          break;
        }
      }
    }
  }
  assert(lower_points->size() == upper_points->size());
  // 如果没有overlap就返回false，此时无人车的规划路径与障碍物的预测轨迹没有干涉，这时候可以做出ignore决策
  return (lower_points->size() > 1 && upper_points->size() > 1);
}

bool StBoundaryMapper::MapWithDecision(
    PathObstacle* path_obstacle, const ObjectDecisionType& decision) const {
  assert(decision.has_follow() || decision.has_yield() ||
         decision.has_overtake());

  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  if (!GetOverlapBoundaryPoints(path_data_.discretized_path().path_points(),
                                *(path_obstacle->obstacle()), &upper_points,
                                &lower_points)) {
    return true;
  }

  if (decision.has_follow() && lower_points.back().t() < planning_time_) {
    const double diff_s = lower_points.back().s() - lower_points.front().s();
    const double diff_t = lower_points.back().t() - lower_points.front().t();
    double extend_lower_s =
        diff_s / diff_t * (planning_time_ - lower_points.front().t()) +
        lower_points.front().s();
    const double extend_upper_s =
        extend_lower_s + (upper_points.back().s() - lower_points.back().s()) +
        1.0;
    upper_points.emplace_back(extend_upper_s, planning_time_);
    lower_points.emplace_back(extend_lower_s, planning_time_);
  }

  auto boundary = StBoundary::GenerateStBoundary(lower_points, upper_points)
                      .ExpandByS(boundary_s_buffer)
                      .ExpandByT(boundary_t_buffer);

  // get characteristic_length and boundary_type.
  StBoundary::BoundaryType b_type = StBoundary::BoundaryType::UNKNOWN;
  double characteristic_length = 0.0;
  if (decision.has_follow()) {
    characteristic_length = std::fabs(decision.follow().distance_s);
    b_type = StBoundary::BoundaryType::FOLLOW;
  } else if (decision.has_yield()) {
    characteristic_length = std::fabs(decision.yield().distance_s);
    boundary = StBoundary::GenerateStBoundary(lower_points, upper_points)
                   .ExpandByS(characteristic_length);
    b_type = StBoundary::BoundaryType::YIELD;
  } else if (decision.has_overtake()) {
    characteristic_length = std::fabs(decision.overtake().distance_s);
    b_type = StBoundary::BoundaryType::OVERTAKE;
  }
  boundary.SetBoundaryType(b_type);
  boundary.SetId(path_obstacle->obstacle()->Id());
  boundary.SetCharacteristicLength(characteristic_length);
  path_obstacle->SetStBoundary(boundary);

  return true;
}

bool StBoundaryMapper::CheckOverlap(const PathPoint& path_point,
                                    const Box2d& obs_box,
                                    const double buffer) const {
  double left_delta_l = 0.0;
  double right_delta_l = 0.0;
  if (is_change_lane_) {
    if ((adc_sl_boundary_.start_l + adc_sl_boundary_.end_l) / 2.0 > 0.0) {
      // change to right
      left_delta_l = 1.0;
    } else {
      // change to left
      right_delta_l = 1.0;
    }
  }
  Vec2d vec_to_center =
      Vec2d((vehicle_param_.front_edge_to_center -
             vehicle_param_.back_edge_to_center) /
                2.0,
            (vehicle_param_.left_edge_to_center + left_delta_l -
             vehicle_param_.right_edge_to_center + right_delta_l) /
                2.0)
          .rotate(path_point.theta);
  Vec2d center = Vec2d(path_point.x, path_point.y) + vec_to_center;

  const Box2d adc_box =
      Box2d(center, path_point.theta, vehicle_param_.length + 2 * buffer,
            vehicle_param_.width + 2 * buffer);
  return obs_box.HasOverlap(adc_box);
}

}  // namespace tasks
}  // namespace hqplanner
