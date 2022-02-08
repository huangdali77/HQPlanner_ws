// #include "modules/planning/tasks/speed_decider/speed_decider.h"
#include "hqplanner/tasks/speed_decider/speed_decider.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "hqplanner/for_proto/decision.h"
#include "hqplanner/for_proto/perception_obstacle.h"
#include "hqplanner/for_proto/vehicle_config_helper.h"
#include "hqplanner/math/line_segment2d.h"
#include "hqplanner/util/util.h"

namespace hqplanner {
namespace tasks {
using hqplanner::Frame;
using hqplanner::PathDecision;
using hqplanner::PathObstacle;
using hqplanner::ReferenceLineInfo;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::ObjectDecisionType;
using hqplanner::forproto::PerceptionObstacle;
using hqplanner::forproto::PlanningConfig;
using hqplanner::math::Vec2d;
using hqplanner::speed::SpeedData;
using hqplanner::speed::StBoundary;
using hqplanner::speed::STPoint;
SpeedDecider::SpeedDecider() : Task("SpeedDecider") {}

bool SpeedDecider::Init(const PlanningConfig& config) {
  dp_st_speed_config_ = config.em_planner_config.dp_st_speed_config;
  st_boundary_config_ = dp_st_speed_config_.st_boundary_config;
  return true;
}

bool SpeedDecider::Execute(Frame* frame,
                           ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);
  init_point_ = frame_->PlanningStartPoint();
  adc_sl_boundary_ = reference_line_info_->AdcSlBoundary();
  reference_line_ = &reference_line_info_->reference_line();
  if (!MakeObjectDecision(reference_line_info->speed_data(),
                          reference_line_info->path_decision())) {
    // const std::string msg = "Get object decision by speed profile failed.";
    // AERROR << msg;
    // return Status(ErrorCode::PLANNING_ERROR, msg);
    return false;
  }
  return true;
}

SpeedDecider::StPosition SpeedDecider::GetStPosition(
    const SpeedData& speed_profile, const StBoundary& st_boundary) const {
  StPosition st_position = BELOW;
  if (st_boundary.IsEmpty()) {
    return st_position;
  }

  bool st_position_set = false;
  const double start_t = st_boundary.min_t();
  const double end_t = st_boundary.max_t();
  for (size_t i = 0; i + 1 < speed_profile.speed_vector().size(); ++i) {
    const STPoint curr_st(speed_profile.speed_vector()[i].s,
                          speed_profile.speed_vector()[i].t);
    const STPoint next_st(speed_profile.speed_vector()[i + 1].s,
                          speed_profile.speed_vector()[i + 1].t);
    if (curr_st.t() < start_t && next_st.t() < start_t) {
      continue;
    }
    if (curr_st.t() > end_t) {
      break;
    }

    hqplanner::math::LineSegment2d speed_line(curr_st, next_st);
    if (st_boundary.HasOverlap(speed_line)) {
      //   ADEBUG << "speed profile cross st_boundaries.";
      st_position = CROSS;
      break;
    }

    // note: st_position can be calculated by checking two st points once
    //       but we need iterate all st points to make sure there is no CROSS
    if (!st_position_set) {
      if (start_t < next_st.t() && curr_st.t() < end_t) {
        STPoint bd_point_front = st_boundary.upper_points().front();
        double side =
            hqplanner::math::CrossProd(bd_point_front, curr_st, next_st);
        st_position = side < 0.0 ? ABOVE : BELOW;
        st_position_set = true;
      }
    }
  }
  return st_position;
}

bool SpeedDecider::IsFollowTooClose(const PathObstacle& path_obstacle) const {
  if (!path_obstacle.IsBlockingObstacle()) {
    return false;
  }

  //   if (path_obstacle.st_boundary().min_t() > 0.0) {
  //     return false;
  //   }
  const double obs_speed = path_obstacle.obstacle()->Speed();
  const double ego_speed = init_point_.v;
  if (obs_speed > ego_speed) {
    return false;
  }
  const double distance =
      path_obstacle.st_boundary().min_s() -
      ConfigParam::instance()->FLAGS_min_stop_distance_obstacle;
  constexpr double decel = 1.0;
  return distance <
         (ego_speed * ego_speed - obs_speed * obs_speed) / (2 * decel);
}

bool SpeedDecider::MakeObjectDecision(const SpeedData& speed_profile,
                                      PathDecision* const path_decision) const {
  if (speed_profile.speed_vector().size() < 2) {
    // const std::string msg = "dp_st_graph failed to get speed profile.";
    // AERROR << msg;
    // return Status(ErrorCode::PLANNING_ERROR, msg);
    return false;
  }
  for (const auto* obstacle : path_decision->path_obstacle_items()) {
    auto* path_obstacle = path_decision->Find(obstacle->Id());
    const auto& boundary = path_obstacle->st_boundary();

    if (boundary.IsEmpty() || boundary.max_s() < 0.0 ||
        boundary.max_t() < 0.0 ||
        boundary.min_t() > dp_st_speed_config_.total_time) {
      AppendIgnoreDecision(path_obstacle);
      continue;
    }

    // if (path_obstacle->HasLongitudinalDecision()) {
    //   AppendIgnoreDecision(path_obstacle);
    //   continue;
    // }

    auto position = GetStPosition(speed_profile, boundary);
    switch (position) {
      case BELOW:
        if (boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(
                  *path_obstacle, &stop_decision,
                  -ConfigParam::instance()->FLAGS_stop_line_stop_distance)) {
            path_obstacle->AddLongitudinalDecision("dp_st_graph/keep_clear",
                                                   stop_decision);
          }
        } else if (CheckIsFollowByT(boundary) &&
                   (boundary.max_t() - boundary.min_t() >
                    ConfigParam::instance()->FLAGS_follow_min_time_sec)) {
          // stop for low_speed decelerating
          if (IsFollowTooClose(*path_obstacle)) {
            ObjectDecisionType stop_decision;
            if (CreateStopDecision(*path_obstacle, &stop_decision,
                                   -ConfigParam::instance()
                                        ->FLAGS_min_stop_distance_obstacle)) {
              path_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
                                                     stop_decision);
            }
          } else {  // high speed or low speed accelerating
            // FOLLOW decision
            ObjectDecisionType follow_decision;
            if (CreateFollowDecision(*path_obstacle, &follow_decision)) {
              path_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                     follow_decision);
            }
          }
        } else {
          // YIELD decision
          ObjectDecisionType yield_decision;
          if (CreateYieldDecision(*path_obstacle, &yield_decision)) {
            path_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                   yield_decision);
          }
        }
        break;
      case ABOVE:
        if (boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
          ObjectDecisionType ignore;
          ignore.object_tag = ObjectDecisionType::ObjectTag::IGNORE;

          path_obstacle->AddLongitudinalDecision("dp_st_graph", ignore);
        } else {
          // OVERTAKE decision
          ObjectDecisionType overtake_decision;
          if (CreateOvertakeDecision(*path_obstacle, &overtake_decision)) {
            path_obstacle->AddLongitudinalDecision("dp_st_graph/overtake",
                                                   overtake_decision);
          }
        }
        break;
      case CROSS:
        if (obstacle->IsBlockingObstacle()) {
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(
                  *path_obstacle, &stop_decision,
                  -ConfigParam::instance()->FLAGS_min_stop_distance_obstacle)) {
            path_obstacle->AddLongitudinalDecision("dp_st_graph/cross",
                                                   stop_decision);
          }
          // const std::string msg =
          //     "Failed to find a solution for crossing obstacle:" +
          //     obstacle->Id();
          // AERROR << msg;
          return false;
        }
        break;
      default:
        // AERROR << "Unknown position:" << position;
    }
    AppendIgnoreDecision(path_obstacle);
  }
  return true;
}

void SpeedDecider::AppendIgnoreDecision(PathObstacle* path_obstacle) const {
  ObjectDecisionType ignore_decision;
  ignore_decision.object_tag = ObjectDecisionType::ObjectTag::IGNORE;
  //   ignore_decision.mutable_ignore();
  if (!path_obstacle->HasLongitudinalDecision()) {
    path_obstacle->AddLongitudinalDecision("dp_st_graph", ignore_decision);
  }
  if (!path_obstacle->HasLateralDecision()) {
    path_obstacle->AddLateralDecision("dp_st_graph", ignore_decision);
  }
}

bool SpeedDecider::CreateStopDecision(const PathObstacle& path_obstacle,
                                      ObjectDecisionType* const stop_decision,
                                      double stop_distance) const {
  //   DCHECK_NOTNULL(stop_decision);
  assert(stop_decision != nullptr);
  const auto& boundary = path_obstacle.st_boundary();
  const double fence_s =
      adc_sl_boundary_.end_s + boundary.min_s() + stop_distance;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < fence_s) {
    // ADEBUG << "Stop fence is further away, ignore.";
    return false;
  }

  const auto fence_point = reference_line_->GetReferencePoint(fence_s);

  // set STOP decision
  stop_decision->object_tag = ObjectDecisionType::ObjectTag::STOP;
  stop_decision->stop_.distance_s = stop_distance;
  stop_decision->stop_.stop_point.x = fence_point.x;
  stop_decision->stop_.stop_point.y = fence_point.y;
  stop_decision->stop_.stop_point.z = 0.0;
  stop_decision->stop_.stop_heading = fence_point.heading;

  //   PerceptionObstacle::Type obstacle_type =
  //       path_obstacle.obstacle()->Perception().type();
  //   ADEBUG << "STOP: obstacle_id[" << path_obstacle.obstacle()->Id()
  //          << "] obstacle_type[" <<
  //          PerceptionObstacle_Type_Name(obstacle_type)
  //          << "]";

  return true;
}

bool SpeedDecider::CreateFollowDecision(
    const PathObstacle& path_obstacle,
    ObjectDecisionType* const follow_decision) const {
  assert(follow_decision != nullptr);
  const double follow_speed = init_point_.v;
  const double follow_distance_s = -std::fmax(
      follow_speed * ConfigParam::instance()->FLAGS_follow_time_buffer,
      ConfigParam::instance()->FLAGS_follow_min_distance);

  const auto& boundary = path_obstacle.st_boundary();
  const double reference_s =
      adc_sl_boundary_.end_s + boundary.min_s() + follow_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < reference_s) {
    // ADEBUG << "Follow reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_s);

  // set FOLLOW decision

  follow_decision->object_tag = ObjectDecisionType::ObjectTag::FOLLOW;
  follow_decision->follow_.distance_s = follow_distance_s;
  follow_decision->follow_.fence_point.x = ref_point.x;
  follow_decision->follow_.fence_point.y = ref_point.y;
  follow_decision->follow_.fence_point.z = 0;
  follow_decision->follow_.fence_heading = ref_point.heading;

  //   PerceptionObstacle::Type obstacle_type =
  //       path_obstacle.obstacle()->Perception().type();
  //   ADEBUG << "FOLLOW: obstacle_id[" << path_obstacle.obstacle()->Id()
  //          << "] obstacle_type[" <<
  //          PerceptionObstacle_Type_Name(obstacle_type)
  //          << "]";

  return true;
}

bool SpeedDecider::CreateYieldDecision(
    const PathObstacle& path_obstacle,
    ObjectDecisionType* const yield_decision) const {
  assert(yield_decision != nullptr);

  PerceptionObstacle::Type obstacle_type =
      path_obstacle.obstacle()->Perception().type;
  double yield_distance = ConfigParam::instance()->FLAGS_yield_distance;
  switch (obstacle_type) {
    case PerceptionObstacle::PEDESTRIAN:
    case PerceptionObstacle::BICYCLE:
      yield_distance =
          ConfigParam::instance()->FLAGS_yield_distance_pedestrian_bycicle;
      break;
    default:
      yield_distance = ConfigParam::instance()->FLAGS_yield_distance;
      break;
  }

  const auto& obstacle_boundary = path_obstacle.st_boundary();
  const double yield_distance_s =
      std::max(-obstacle_boundary.min_s(), -yield_distance);

  const double reference_line_fence_s =
      adc_sl_boundary_.end_s + obstacle_boundary.min_s() + yield_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < reference_line_fence_s) {
    // ADEBUG << "Yield reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

  // set YIELD decision
  yield_decision->object_tag = ObjectDecisionType::ObjectTag::YIELD;
  yield_decision->yield_.distance_s = yield_distance_s;
  yield_decision->yield_.fence_point.x = ref_point.x;
  yield_decision->yield_.fence_point.y = ref_point.y;
  yield_decision->yield_.fence_point.z = 0;
  yield_decision->yield_.fence_heading = ref_point.heading;

  // ADEBUG << "YIELD: obstacle_id[" << path_obstacle.obstacle()->Id()
  //        << "] obstacle_type[" << PerceptionObstacle_Type_Name(obstacle_type)
  //        << "]";

  return true;
}

bool SpeedDecider::CreateOvertakeDecision(
    const PathObstacle& path_obstacle,
    ObjectDecisionType* const overtake_decision) const {
  assert(overtake_decision != nullptr);

  constexpr double kOvertakeTimeBuffer = 3.0;    // in seconds
  constexpr double kMinOvertakeDistance = 10.0;  // in meters

  const auto& velocity = path_obstacle.obstacle()->Perception().velocity;
  const double obstacle_speed =
      hqplanner::math::Vec2d::CreateUnitVec2d(init_point_.path_point.theta)
          .InnerProd(Vec2d(velocity.x, velocity.y));

  const double overtake_distance_s =
      std::fmax(std::fmax(init_point_.v, obstacle_speed) * kOvertakeTimeBuffer,
                kMinOvertakeDistance);

  const auto& boundary = path_obstacle.st_boundary();
  const double reference_line_fence_s =
      adc_sl_boundary_.end_s + boundary.min_s() + overtake_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < reference_line_fence_s) {
    // ADEBUG << "Overtake reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

  // set OVERTAKE decision
  overtake_decision->object_tag = ObjectDecisionType::ObjectTag::OVERTAKE;
  overtake_decision->overtake_.distance_s = overtake_distance_s;
  overtake_decision->overtake_.fence_point.x = ref_point.x;
  overtake_decision->overtake_.fence_point.y = ref_point.y;
  overtake_decision->overtake_.fence_point.z = 0.0;
  overtake_decision->overtake_.fence_heading = ref_point.heading;

  return true;
}

bool SpeedDecider::CheckIsFollowByT(const StBoundary& boundary) const {
  if (boundary.BottomLeftPoint().s() > boundary.BottomRightPoint().s()) {
    return false;
  }
  constexpr double kFollowTimeEpsilon = 1e-3;
  constexpr double kFollowCutOffTime = 0.5;
  if (boundary.min_t() > kFollowCutOffTime ||
      boundary.max_t() < kFollowTimeEpsilon) {
    return false;
  }
  return true;
}

}  // namespace tasks
}  // namespace hqplanner
