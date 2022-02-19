#include "hqplanner/common/path_decision.h"

#include <ros/ros.h>

#include <memory>
#include <utility>
namespace hqplanner {
using hqplanner::PathObstacle;
using hqplanner::ReferenceLine;
using hqplanner::forproto::MainStop;
using hqplanner::forproto::ObjectDecisionType;
using hqplanner::forproto::ObjectStop;
using hqplanner::forproto::PointENU;
using hqplanner::forproto::SLBoundary;
using hqplanner::forproto::SLPoint;
using hqplanner::speed::StBoundary;

PathObstacle *PathDecision::AddPathObstacle(const PathObstacle &path_obstacle) {
  auto obs = path_obstacles_.find(path_obstacle.Id());
  std::shared_ptr<PathObstacle> temp(new PathObstacle(path_obstacle));
  if (obs != path_obstacles_.end()) {
    ROS_INFO("duplicate obstacle");
    assert(0);
    path_obstacles_.erase(path_obstacle.Id());
    path_obstacles_.insert({path_obstacle.Id(), std::move(temp)});
    //删除 path_obstacle_items_中重复元素，并重新添加
    auto path_obstacle_ptr = path_obstacle_items_.begin();
    for (; path_obstacle_ptr != path_obstacle_items_.end();
         ++path_obstacle_ptr) {
      if ((*path_obstacle_ptr)->Id() == path_obstacle.Id()) {
        break;
      }
    }
    path_obstacle_items_.erase(path_obstacle_ptr);
    path_obstacle_items_.push_back(path_obstacles_[path_obstacle.Id()].get());

  } else {
    path_obstacles_.insert({path_obstacle.Id(), std::move(temp)});
    path_obstacle_items_.push_back(path_obstacles_[path_obstacle.Id()].get());
  }
  return path_obstacles_.find(path_obstacle.Id())->second.get();
  // return path_obstacles_[path_obstacle.Id()].get();
}

const PathObstacle *PathDecision::Find(const std::string &object_id) const {
  // return path_obstacles_.Find(object_id);
  const auto path_obs_ptr = path_obstacles_.find(object_id);
  if (path_obs_ptr == path_obstacles_.end()) {
    return nullptr;
  }

  return path_obs_ptr->second.get();
}

PathObstacle *PathDecision::Find(const std::string &object_id) {
  // return path_obstacles_.Find(object_id);
  const auto path_obs_ptr = path_obstacles_.find(object_id);
  if (path_obs_ptr == path_obstacles_.end()) {
    return nullptr;
  }

  return path_obs_ptr->second.get();
}

const std::map<std::string, std::shared_ptr<PathObstacle>>
    &PathDecision::path_obstacles() const {
  return path_obstacles_;
}

void PathDecision::SetStBoundary(const std::string &id,
                                 const StBoundary &boundary) {
  auto obstacle = path_obstacles_.find(id);

  if (obstacle == path_obstacles_.end()) {
    // AERROR << "Failed to find obstacle : " << id;
    return;
  } else {
    obstacle->second->SetStBoundary(boundary);
    // obstacle->SetStBoundary(boundary);
  }
}

bool PathDecision::AddLateralDecision(const std::string &tag,
                                      const std::string &object_id,
                                      const ObjectDecisionType &decision) {
  auto path_obstacle = path_obstacles_.find(object_id);
  if (path_obstacle == path_obstacles_.end()) {
    ROS_INFO("failed to find obstacle");
    return false;
  }
  path_obstacle->second->AddLateralDecision(tag, decision);
  // path_obstacle->AddLateralDecision(tag, decision);
  return true;
}

bool PathDecision::AddLongitudinalDecision(const std::string &tag,
                                           const std::string &object_id,
                                           const ObjectDecisionType &decision) {
  auto path_obstacle = path_obstacles_.find(object_id);
  if (path_obstacle == path_obstacles_.end()) {
    ROS_INFO("failed to find obstacle");

    return false;
  }
  path_obstacle->second->AddLongitudinalDecision(tag, decision);

  return true;
}

void PathDecision::EraseStBoundaries() {
  for (auto &path_obstacle : path_obstacles_) {
    path_obstacle.second->EraseStBoundary();
  }
}

bool PathDecision::MergeWithMainStop(const ObjectStop &obj_stop,
                                     const std::string &obj_id,
                                     const ReferenceLine &reference_line,
                                     const SLBoundary &adc_sl_boundary) {
  PointENU stop_point = obj_stop.stop_point;
  SLPoint stop_line_sl;
  reference_line.XYToSL({stop_point.x, stop_point.y}, &stop_line_sl);

  double stop_line_s = stop_line_sl.s;
  if (stop_line_s < 0 || stop_line_s > reference_line.Length()) {
    ROS_INFO("Ignore object: %s fence route_s[%f] not in range[0, %f]",
             obj_id.c_str(), stop_line_s, reference_line.Length());
    return false;
  }

  // check stop_line_s vs adc_s, ignore if it is further way than main stop
  const double kStopBuff = 1.0;
  stop_line_s = std::fmax(stop_line_s, adc_sl_boundary.end_s - kStopBuff);

  if (stop_line_s >= stop_reference_line_s_) {
    return false;
  }

  main_stop_.reason_code = obj_stop.reason_code;
  main_stop_.reason = ("stop by " + obj_id);
  main_stop_.stop_point.x = obj_stop.stop_point.x;
  main_stop_.stop_point.y = obj_stop.stop_point.y;
  main_stop_.stop_heading = obj_stop.stop_heading;
  stop_reference_line_s_ = stop_line_s;

  return true;
}

const std::vector<const PathObstacle *> &PathDecision::path_obstacle_items()
    const {
  return path_obstacle_items_;
}
}  // namespace hqplanner
