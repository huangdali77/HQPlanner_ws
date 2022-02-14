#include "hqplanner/reference_line/reference_line_info.h"

#include <ros/ros.h>

#include "hqplanner/math/box2d.h"
#include "hqplanner/math/vec2d.h"
namespace hqplanner {
using hqplanner::PathDecision;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::ObjectDecisionType;
using hqplanner::forproto::PathPoint;
using hqplanner::forproto::SLBoundary;
using hqplanner::forproto::SLPoint;
using hqplanner::forproto::SpeedPoint;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::forproto::VehicleConfig;
using hqplanner::forproto::VehicleConfigHelper;
using hqplanner::forproto::VehicleParam;
using hqplanner::forproto::VehicleState;
using hqplanner::math::Box2d;
using hqplanner::math::Vec2d;
using hqplanner::path::PathData;
using hqplanner::speed::SpeedData;
using hqplanner::trajectory::DiscretizedTrajectory;

ReferenceLineInfo::ReferenceLineInfo(const VehicleState& vehicle_state,
                                     const TrajectoryPoint& adc_planning_point,
                                     const ReferenceLine& reference_line)
    : vehicle_state_(vehicle_state),
      adc_planning_point_(adc_planning_point),
      reference_line_(reference_line) {}

bool ReferenceLineInfo::Init(const std::vector<const Obstacle*>& obstacles) {
  const auto& param = VehicleConfigHelper::GetConfig().vehicle_param;
  const auto& path_point = adc_planning_point_.path_point;
  Vec2d position(path_point.x, path_point.y);
  Vec2d vec_to_center(
      (param.front_edge_to_center - param.back_edge_to_center) / 2.0,
      (param.left_edge_to_center - param.right_edge_to_center) / 2.0);
  Vec2d center(position + vec_to_center.rotate(path_point.theta));
  Box2d box(center, path_point.theta, param.length, param.width);

  // 获取自车的sl_boundary
  if (!reference_line_.GetSLBoundary(box, &adc_sl_boundary_)) {
    ROS_INFO("Failed to get ADC boundary from box.");
    return false;
  }
  if (adc_sl_boundary_.end_s < 0 ||
      adc_sl_boundary_.start_s > reference_line_.Length()) {
    ROS_INFO("Vehicle SL is not on reference line:[0, %f]",
             reference_line_.Length());
    assert(0);
  }
  constexpr double kOutOfReferenceLineL = 10.0;  // in meters
  if (adc_sl_boundary_.start_l > kOutOfReferenceLineL ||
      adc_sl_boundary_.end_l < -kOutOfReferenceLineL) {
    is_on_reference_line_ = false;
    ROS_INFO("Ego vehicle is too far away from reference line.");
    assert(0);
    // AERROR << ;
    return false;
  }
  is_on_reference_line_ = reference_line_.IsOnRoad(adc_sl_boundary_);
  if (!AddObstacles(obstacles)) {
    ROS_INFO("Failed to add obstacles to reference line");
    assert(0);
    return false;
  }

  // 添加速度限制
  auto ref_line = reference_line_.reference_points();
  reference_line_.AddSpeedLimit(
      ref_line.front().s, ref_line.back().s,
      ConfigParam::instance()->FLAGS_planning_upper_speed_limit);
  // if (hdmap::GetSpeedControls()) {
  //   auto* speed_controls = hdmap::GetSpeedControls();
  //   for (const auto& speed_control : speed_controls->speed_control()) {
  //     reference_line_.AddSpeedLimit(speed_control);
  //   }
  // }

  // set lattice planning target speed limit;
  // SetCruiseSpeed(FLAGS_default_cruise_speed);
  // is_safe_to_change_lane_ = CheckChangeLane();
  is_inited_ = true;
  return true;
}

bool ReferenceLineInfo::AddObstacles(
    const std::vector<const Obstacle*>& obstacles) {
  for (const auto* obstacle : obstacles) {
    if (!AddObstacle(obstacle)) {
      return false;
    }
  }
  return true;
}

PathObstacle* ReferenceLineInfo::AddObstacle(const Obstacle* obstacle) {
  if (!obstacle) {
    // AERROR << "The provided obstacle is empty";
    return nullptr;
  }
  auto* path_obstacle = path_decision_.AddPathObstacle(PathObstacle(obstacle));
  if (!path_obstacle) {
    // AERROR << "failed to add obstacle " << obstacle->Id();
    return nullptr;
  }

  SLBoundary perception_sl;
  if (!reference_line_.GetSLBoundary(obstacle->PerceptionBoundingBox(),
                                     &perception_sl)) {
    // AERROR << "Failed to get sl boundary for obstacle: " << obstacle->Id();
    return path_obstacle;
  }
  path_obstacle->SetPerceptionSlBoundary(perception_sl);

  if (IsUnrelaventObstacle(path_obstacle)) {
    ObjectDecisionType ignore;
    ignore.object_tag = ObjectDecisionType::IGNORE;
    // ignore.mutable_ignore();
    path_decision_.AddLateralDecision("reference_line_filter", obstacle->Id(),
                                      ignore);
    path_decision_.AddLongitudinalDecision("reference_line_filter",
                                           obstacle->Id(), ignore);

  } else {
    path_obstacle->BuildReferenceLineStBoundary(reference_line_,
                                                adc_sl_boundary_.start_s);
  }
  return path_obstacle;
}

bool ReferenceLineInfo::IsUnrelaventObstacle(PathObstacle* path_obstacle) {
  // if adc is on the road, and obstacle behind adc, ignore
  if (path_obstacle->PerceptionSLBoundary().end_s > reference_line_.Length()) {
    return true;
  }
  if (is_on_reference_line_ &&
      path_obstacle->PerceptionSLBoundary().end_s < adc_sl_boundary_.end_s &&
      reference_line_.IsOnRoad(path_obstacle->PerceptionSLBoundary())) {
    return true;
  }
  return false;
}

void ReferenceLineInfo::SetDrivable(bool drivable) { is_drivable_ = drivable; }

bool ReferenceLineInfo::IsStartFrom(
    const ReferenceLineInfo& previous_reference_line_info) const {
  if (reference_line_.reference_points().empty()) {
    return false;
  }
  auto start_ref_point = reference_line_.reference_points().front();
  Vec2d start_point(start_ref_point.x, start_ref_point.y);
  const auto& prev_reference_line =
      previous_reference_line_info.reference_line();
  SLPoint sl_point;
  prev_reference_line.XYToSL(start_point, &sl_point);
  return previous_reference_line_info.reference_line_.IsOnRoad(sl_point);
}

const PathData& ReferenceLineInfo::path_data() const { return path_data_; }

const SpeedData& ReferenceLineInfo::speed_data() const { return speed_data_; }

PathData* ReferenceLineInfo::mutable_path_data() { return &path_data_; }

SpeedData* ReferenceLineInfo::mutable_speed_data() { return &speed_data_; }

bool ReferenceLineInfo::IsInited() const { return is_inited_; }

PathDecision* ReferenceLineInfo::path_decision() { return &path_decision_; }

const PathDecision& ReferenceLineInfo::path_decision() const {
  return path_decision_;
}

const ReferenceLine& ReferenceLineInfo::reference_line() const {
  return reference_line_;
}
const TrajectoryPoint& ReferenceLineInfo::AdcPlanningPoint() const {
  return adc_planning_point_;
}

const SLBoundary& ReferenceLineInfo::AdcSlBoundary() const {
  return adc_sl_boundary_;
}
void ReferenceLineInfo::SetTrajectory(const DiscretizedTrajectory& trajectory) {
  discretized_trajectory_ = trajectory;
}

const DiscretizedTrajectory& ReferenceLineInfo::trajectory() const {
  return discretized_trajectory_;
}
double ReferenceLineInfo::TrajectoryLength() const {
  const auto& tps = discretized_trajectory_.trajectory_points();
  if (tps.empty()) {
    return 0.0;
  }
  return tps.back().path_point.s;
}

bool ReferenceLineInfo::CombinePathAndSpeedProfile(
    const double relative_time, const double start_s,
    DiscretizedTrajectory* ptr_discretized_trajectory) {
  assert(ptr_discretized_trajectory != nullptr);
  // use varied resolution to reduce data load but also provide enough data
  // point for control module
  const double kDenseTimeResoltuion =
      ConfigParam::instance()->FLAGS_trajectory_time_min_interval;

  const double kSparseTimeResolution =
      ConfigParam::instance()->FLAGS_trajectory_time_max_interval;

  const double kDenseTimeSec =
      ConfigParam::instance()->FLAGS_trajectory_time_high_density_period;

  if (path_data_.discretized_path().NumOfPoints() == 0) {
    ROS_INFO("path data is empty");

    return false;
  }

  // 轨迹规划前1秒的时间步长为0.02秒，后面的时间为0.1秒，但是这里修改kDenseTimeResoltuion=0.1
  for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.TotalTime();
       cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion
                                                     : kSparseTimeResolution)) {
    SpeedPoint speed_point;
    if (!speed_data_.EvaluateByTime(cur_rel_time, &speed_point)) {
      ROS_INFO("Fail to get speed point with relative time:%f", cur_rel_time);

      return false;
    }

    if (speed_point.s > path_data_.discretized_path().Length()) {
      break;
    }
    PathPoint path_point;
    if (!path_data_.GetPathPointWithPathS(speed_point.s, &path_point)) {
      ROS_INFO("Fail to get path data with s: %f, path total length:%f",
               speed_point.s(), path_data_.discretized_path().Length());

      return false;
    }
    path_point.s = path_point.s + start_s;

    TrajectoryPoint trajectory_point;

    trajectory_point.path_point = path_point;
    trajectory_point.v = speed_point.v;
    trajectory_point.a = speed_point.a;
    trajectory_point.relative_time = speed_point.t + relative_time;
    ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
  }
  return true;
}
bool ReferenceLineInfo::IsDrivable() const { return is_drivable_; }

bool ReferenceLineInfo::ReachedDestination() const {
  constexpr double kDestinationDeltaS = 0.05;
  const auto* dest_ptr = path_decision_.Find(
      ConfigParam::instance()->FLAGS_destination_obstacle_id);
  if (!dest_ptr) {
    return false;
  }
  if (!dest_ptr->LongitudinalDecision().has_stop()) {
    return false;
  }
  if (!reference_line_.IsOnRoad(
          dest_ptr->obstacle()->PerceptionBoundingBox().center())) {
    return false;
  }
  const double stop_s = dest_ptr->PerceptionSLBoundary().start_s +
                        dest_ptr->LongitudinalDecision().stop().distance_s;
  return adc_sl_boundary_.end_s + kDestinationDeltaS > stop_s;
}

}  // namespace hqplanner
