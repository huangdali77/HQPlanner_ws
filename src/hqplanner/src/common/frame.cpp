#include "hqplanner/common/frame.h"

#include <ros/ros.h>

#include "hqplanner/for_proto/vehicle_state_provider.h"
#include "hqplanner/main/prediction_obstacles_provider.h"
namespace hqplanner {
// using hqplanner::Subscribe;
using hqplanner::PredictionObstaclesProvider;
using hqplanner::forproto::ADCTrajectory;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::PerceptionObstacle;
using hqplanner::forproto::PerceptionObstacles;
using hqplanner::forproto::PredictionObstacles;
using hqplanner::forproto::SLPoint;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::forproto::VehicleConfigHelper;
using hqplanner::forproto::VehicleParam;
using hqplanner::forproto::VehicleState;
using hqplanner::forproto::VehicleStateProvider;
using hqplanner::math::Box2d;
using hqplanner::math::IndexedQueue;
using hqplanner::math::LineSegment2d;
using hqplanner::math::Vec2d;

constexpr double kMathEpsilon = 1e-8;

FrameHistory::FrameHistory()
    : IndexedQueue<std::uint32_t, Frame>(
          ConfigParam::instance()->FLAGS_max_history_frame_num) {}

Frame::Frame(std::uint32_t sequence_num,
             const TrajectoryPoint &planning_start_point,
             const double start_time, const VehicleState &vehicle_state,
             ReferenceLineProvider *reference_line_provider)
    : sequence_num_(sequence_num),
      planning_start_point_(planning_start_point),
      start_time_(start_time),
      vehicle_state_(vehicle_state),
      reference_line_provider_(reference_line_provider) {
  prediction_ =
      PredictionObstaclesProvider::instance()->GetPredictionObstacles();
  // prediction_ = subscribe_info_.GetPredictionObstacles();
}

const TrajectoryPoint &Frame::PlanningStartPoint() const {
  return planning_start_point_;
}

const VehicleState &Frame::vehicle_state() const { return vehicle_state_; }

std::list<ReferenceLineInfo> &Frame::reference_line_info() {
  return reference_line_info_;
}

const Obstacle *Frame::CreateStopObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_s) {
  if (reference_line_info == nullptr) {
    // AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  const auto &reference_line = reference_line_info->reference_line();
  const double box_center_s =
      obstacle_s +
      ConfigParam::instance()->FLAGS_virtual_stop_wall_length / 2.0;
  auto box_center = reference_line.GetReferencePoint(box_center_s);
  double heading = reference_line.GetReferencePoint(obstacle_s).heading;
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  reference_line.GetLaneWidth(obstacle_s, &lane_left_width, &lane_right_width);
  Box2d stop_wall_box(Vec2d(box_center.x, box_center.y), heading,
                      ConfigParam::instance()->FLAGS_virtual_stop_wall_length,
                      lane_left_width + lane_right_width);

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

const Obstacle *Frame::CreateStaticObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_start_s,
    const double obstacle_end_s) {
  if (reference_line_info == nullptr) {
    // AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  const auto &reference_line = reference_line_info->reference_line();

  // start_xy
  SLPoint sl_point;
  sl_point.s = obstacle_start_s;
  sl_point.l = 0.0;
  Vec2d obstacle_start_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_start_xy)) {
    // AERROR << "Failed to get start_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  // end_xy
  sl_point.s = obstacle_end_s;
  sl_point.l = 0.0;
  Vec2d obstacle_end_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_end_xy)) {
    // AERROR << "Failed to get end_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  if (!reference_line.GetLaneWidth(obstacle_start_s, &left_lane_width,
                                   &right_lane_width)) {
    // AERROR << "Failed to get lane width at s[" << obstacle_start_s << "]";
    return nullptr;
  }

  Box2d obstacle_box{LineSegment2d(obstacle_start_xy, obstacle_end_xy),
                     left_lane_width + right_lane_width};

  return CreateStaticVirtualObstacle(obstacle_id, obstacle_box);
}

const Obstacle *Frame::CreateStaticVirtualObstacle(const std::string &id,
                                                   const Box2d &box) {
  const auto object = obstacles_.find(id);
  if (object != obstacles_.end()) {
    // AWARN << "obstacle " << id << " already exist.";
    return object->second.get();
  }

  std::shared_ptr<Obstacle> obs_ptr =
      std::move(Obstacle::CreateStaticVirtualObstacles(id, box));

  obstacles_.insert(std::make_pair(id, std::move(obs_ptr)));
  return (obstacles_.find(id))->second.get();
}

bool Frame::Init() {
  vehicle_state_ = VehicleStateProvider::instance()->vehicle_state();
  // vehicle_state_ =    Subscribe::vehicle_state_;
  // hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  // vehicle_state_ = VehicleStateProvider::vehicle_state_;

  // prediction
  // 添加障碍物信息
  if (!prediction_.prediction_obstacle.empty()) {
    for (auto &ptr : Obstacle::CreateObstacles(prediction_)) {
      AddObstacle(*ptr);
    }
  }
  // 碰撞检查
  const auto *collision_obstacle = FindCollisionObstacle();
  // if (collision_obstacle) {
  //   return false;
  // }
  // 生成ReferenceLineInfo
  if (!CreateReferenceLineInfo()) {
    return false;
  }

  return true;
}

const Obstacle *Frame::FindCollisionObstacle() const {
  if (obstacles_.empty()) {
    return nullptr;
  }
  // const VehicleParam param;
  const auto &param =
      VehicleConfigHelper::instance()->GetConfig().vehicle_param;
  Vec2d position(vehicle_state_.x, vehicle_state_.y);
  Vec2d vec_to_center(
      (param.front_edge_to_center - param.back_edge_to_center) / 2.0,
      (param.left_edge_to_center - param.right_edge_to_center) / 2.0);
  Vec2d center(position + vec_to_center.rotate(vehicle_state_.heading));
  Box2d adc_box(center, vehicle_state_.heading, param.length, param.width);
  const double adc_half_diagnal = adc_box.diagonal() / 2.0;

  for (const auto &obstacle : obstacle_items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }

    double center_dist =
        adc_box.center().DistanceTo(obstacle->PerceptionBoundingBox().center());

    if (center_dist >
        obstacle->PerceptionBoundingBox().diagonal() / 2.0 + adc_half_diagnal +
            ConfigParam::instance()->FLAGS_max_collision_distance) {
      // ADEBUG << "Obstacle : " << obstacle->Id() << " is too far to collide";
      continue;
    }
    double distance = obstacle->PerceptionPolygon().DistanceTo(adc_box);

    if (distance < ConfigParam::instance()->FLAGS_max_collision_distance) {
      // AERROR << "Found collision with obstacle " << obstacle->Id();
      return obstacle;
    }
  }

  return nullptr;
}

bool Frame::CreateReferenceLineInfo() {
  std::list<ReferenceLine> reference_lines;
  // std::list<hdmap::RouteSegments> segments;
  if (!reference_line_provider_->GetReferenceLines(&reference_lines)) {
    return false;
  }

  assert(!reference_lines.empty());
  auto forword_limit =
      ReferenceLineProvider::LookForwardDistance(vehicle_state_);

  for (auto &ref_line : reference_lines) {
    if (!ref_line.Shrink(Vec2d(vehicle_state_.x, vehicle_state_.y),
                         ConfigParam::instance()->FLAGS_look_backward_distance,
                         forword_limit)) {
      return false;
    }
  }

  reference_line_info_.clear();
  auto ref_line_iter = reference_lines.begin();
  // auto segments_iter = segments.begin();
  while (ref_line_iter != reference_lines.end()) {
    if (ref_line_iter->reference_points().size() <
        ConfigParam::instance()->FLAGS_num_reference_points_near_destination) {
      is_near_destination_ = true;
    }
    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
                                      *ref_line_iter);
    ++ref_line_iter;
  }

  // if (FLAGS_enable_change_lane_decider &&
  //     !change_lane_decider_.Apply(&reference_line_info_)) {
  //   AERROR << "Failed to apply change lane decider";
  //   return false;
  // }

  if (reference_line_info_.size() == 2) {
    Vec2d xy_point(vehicle_state_.x, vehicle_state_.y);
    SLPoint first_sl;
    if (!reference_line_info_.front().reference_line().XYToSL(xy_point,
                                                              &first_sl)) {
      return false;
    }
    SLPoint second_sl;
    if (!reference_line_info_.back().reference_line().XYToSL(xy_point,
                                                             &second_sl)) {
      return false;
    }
    const double offset = first_sl.l - second_sl.l;
    reference_line_info_.front().SetOffsetToOtherReferenceLine(offset);
    reference_line_info_.back().SetOffsetToOtherReferenceLine(-offset);
  }

  bool has_valid_reference_line = false;
  for (auto &ref_info : reference_line_info_) {
    if (!ref_info.Init(obstacle_items())) {
      // AERROR << "Failed to init reference line";
      continue;
    } else {
      has_valid_reference_line = true;
    }
  }
  return has_valid_reference_line;
}

void Frame::AddObstacle(const Obstacle &obstacle) {
  std::shared_ptr<Obstacle> temp(new Obstacle(obstacle));
  obstacles_.insert({obstacle.Id(), std::move(temp)});
  // obstacles_.insert({obstacle.Id(), obstacle});
  // obstacle_items_.push_back(&(obstacles_[obstacle.Id()]));
}
// const std::map<std::string, Obstacle> obstacles() const;
const std::map<std::string, std::shared_ptr<Obstacle>> Frame::obstacles()
    const {
  return obstacles_;
}

const std::vector<const Obstacle *> Frame::obstacle_items() const {
  std::vector<const Obstacle *> obstacle_ptrs;
  for (const auto &obs : obstacles_) {
    obstacle_ptrs.emplace_back(obs.second.get());
  }
  return obstacle_ptrs;
}

Obstacle *Frame::Find(const std::string &id) {
  auto obs_ptr = obstacles_.find(id);
  if (obs_ptr == obstacles_.end()) {
    return nullptr;
  }
  return obs_ptr->second.get();
}

const ReferenceLineInfo *Frame::FindDriveReferenceLineInfo() {
  drive_reference_line_info_ = &(reference_line_info_.front());
  return drive_reference_line_info_;
}

const ReferenceLineInfo *Frame::DriveReferenceLineInfo() const {
  return drive_reference_line_info_;
}

}  // namespace hqplanner
