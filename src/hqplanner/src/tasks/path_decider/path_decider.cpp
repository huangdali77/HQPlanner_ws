

#include "hqplanner/tasks/path_decider/path_decider.h"

#include <assert.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "hqplanner/for_proto/decision.h"
#include "hqplanner/for_proto/vehicle_config_helper.h"
#include "hqplanner/speed/st_boundary.h"
#include "hqplanner/util/util.h"
namespace hqplanner {
namespace tasks {
using hqplanner::Frame;
using hqplanner::PathDecision;
using hqplanner::ReferenceLineInfo;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::ObjectDecisionType;
using hqplanner::forproto::ObjectNudge;
using hqplanner::forproto::ObjectStop;
using hqplanner::forproto::PerceptionObstacle;
using hqplanner::forproto::StopReasonCode;
using hqplanner::forproto::VehicleConfigHelper;
using hqplanner::path::PathData;
using hqplanner::speed::StBoundary;
// using hqplanner::Path_
PathDecider::PathDecider() : Task("PathDecider") {}

bool PathDecider::Execute(Frame *frame,
                          ReferenceLineInfo *reference_line_info) {
  Task::Execute(frame, reference_line_info);
  return Process(reference_line_info->path_data(),
                 reference_line_info->path_decision());
}

bool PathDecider::Process(const PathData &path_data,
                          PathDecision *const path_decision) {
  assert(path_decision != nullptr);

  if (!MakeObjectDecision(path_data, path_decision)) {
    return false;
  }
  return true;
}

bool PathDecider::MakeObjectDecision(const PathData &path_data,
                                     PathDecision *const path_decision) {
  assert(path_decision != nullptr);

  if (!MakeStaticObstacleDecision(path_data, path_decision)) {
    return false;
  }
  return true;
}

bool PathDecider::MakeStaticObstacleDecision(
    const PathData &path_data, PathDecision *const path_decision) {
  assert(path_decision != nullptr);
  const auto &frenet_path = path_data.frenet_frame_path();
  const auto &frenet_points = frenet_path.points();
  if (frenet_points.empty()) {
    return false;
  }

  const double half_width =
      VehicleConfigHelper::GetConfig().vehicle_param.width / 2.0;

  const double lateral_radius =
      half_width + ConfigParam::instance()->FLAGS_lateral_ignore_buffer;

  const double lateral_stop_radius =
      half_width +
      ConfigParam::instance()->FLAGS_static_decision_nudge_l_buffer;

  for (const auto *path_obstacle : path_decision->path_obstacle_items()) {
    const auto &obstacle = *path_obstacle->obstacle();
    bool is_bycycle_or_pedestrain =
        (obstacle.Perception().type == PerceptionObstacle::BICYCLE ||
         obstacle.Perception().type == PerceptionObstacle::PEDESTRIAN);

    if (!is_bycycle_or_pedestrain && !obstacle.IsStatic()) {
      continue;
    }

    if (path_obstacle->HasLongitudinalDecision() &&
        path_obstacle->LongitudinalDecision().has_ignore() &&
        path_obstacle->HasLateralDecision() &&
        path_obstacle->LateralDecision().has_ignore()) {
      continue;
    }
    if (path_obstacle->HasLongitudinalDecision() &&
        path_obstacle->LongitudinalDecision().has_stop()) {
      // STOP decision
      continue;
    }
    if (path_obstacle->HasLateralDecision() &&
        path_obstacle->LateralDecision().has_sidepass()) {
      // SIDE_PASS decision
      continue;
    }

    if (path_obstacle->reference_line_st_boundary().boundary_type() ==
        StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    // IGNORE by default
    ObjectDecisionType object_decision;
    object_decision.object_tag = ObjectDecisionType::ObjectTag::IGNORE;

    const auto &sl_boundary = path_obstacle->PerceptionSLBoundary();

    if (sl_boundary.end_s < frenet_points.front().s ||
        sl_boundary.start_s > frenet_points.back().s) {
      path_decision->AddLongitudinalDecision("PathDecider/not-in-s",
                                             obstacle.Id(), object_decision);
      path_decision->AddLateralDecision("PathDecider/not-in-s", obstacle.Id(),
                                        object_decision);
      continue;
    }

    const auto frenet_point = frenet_path.GetNearestPoint(sl_boundary);
    const double curr_l = frenet_point.l;
    if (curr_l - lateral_radius > sl_boundary.end_l ||
        curr_l + lateral_radius < sl_boundary.start_l) {
      // ignore
      path_decision->AddLateralDecision("PathDecider/not-in-l", obstacle.Id(),
                                        object_decision);
    } else if (curr_l - lateral_stop_radius < sl_boundary.end_l &&
               curr_l + lateral_stop_radius > sl_boundary.start_l) {
      // stop
      object_decision.object_tag = ObjectDecisionType::ObjectTag::STOP;
      object_decision.stop_ = GenerateObjectStopDecision(*path_obstacle);
      //   *object_decision.mutable_stop() =
      //       GenerateObjectStopDecision(*path_obstacle);

      if (path_decision->MergeWithMainStop(
              object_decision.stop(), obstacle.Id(),
              reference_line_info_->reference_line(),
              reference_line_info_->AdcSlBoundary())) {
        path_decision->AddLongitudinalDecision("PathDecider/nearest-stop",
                                               obstacle.Id(), object_decision);
      } else {
        ObjectDecisionType object_decision;
        object_decision.object_tag = ObjectDecisionType::ObjectTag::IGNORE;
        path_decision->AddLongitudinalDecision("PathDecider/not-nearest-stop",
                                               obstacle.Id(), object_decision);
      }
    } else if (ConfigParam::instance()->FLAGS_enable_nudge_decision) {
      // nudge
      if (curr_l - lateral_stop_radius > sl_boundary.end_l) {
        // LEFT_NUDGE
        object_decision.object_tag = ObjectDecisionType::ObjectTag::NUDGE;

        object_decision.nudge_.type = ObjectNudge::Type::LEFT_NUDGE;

        object_decision.nudge_.distance_l =
            ConfigParam::instance()->FLAGS_nudge_distance_obstacle;

        path_decision->AddLateralDecision("PathDecider/left-nudge",
                                          obstacle.Id(), object_decision);
      } else {
        // RIGHT_NUDGE
        object_decision.object_tag = ObjectDecisionType::ObjectTag::NUDGE;
        object_decision.nudge_.type = ObjectNudge::Type::RIGHT_NUDGE;
        object_decision.nudge_.distance_l =
            -ConfigParam::instance()->FLAGS_nudge_distance_obstacle;
        path_decision->AddLateralDecision("PathDecider/right-nudge",
                                          obstacle.Id(), object_decision);
      }
    }
  }

  return true;
}

ObjectStop PathDecider::GenerateObjectStopDecision(
    const PathObstacle &path_obstacle) const {
  ObjectStop object_stop;

  double stop_distance = path_obstacle.MinRadiusStopDistance(
      VehicleConfigHelper::GetConfig().vehicle_param);
  object_stop.reason_code = StopReasonCode::STOP_REASON_OBSTACLE;
  //   object_stop.set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
  object_stop.distance_s = -stop_distance;
  //   object_stop.set_distance_s(-stop_distance);

  const double stop_ref_s =
      path_obstacle.PerceptionSLBoundary().start_s - stop_distance;
  const auto stop_ref_point =
      reference_line_info_->reference_line().GetReferencePoint(stop_ref_s);
  object_stop.stop_point.x = stop_ref_point.x;
  object_stop.stop_point.y = stop_ref_point.y;
  object_stop.stop_heading = stop_ref_point.heading;

  return object_stop;
}

}  // namespace tasks
}  // namespace hqplanner
