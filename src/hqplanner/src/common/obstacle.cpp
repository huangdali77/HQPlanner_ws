#include "hqplanner/common/obstacle.h"

#include "hqplanner/math/linear_interpolation.h"
#include "hqplanner/util/util.h"
namespace hqplanner {
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::PerceptionObstacle;
using hqplanner::forproto::Point;
using hqplanner::forproto::PredictionObstacles;
// using hqplanner::forproto::Trajectory;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::math::Box2d;
using hqplanner::math::Polygon2d;
using hqplanner::math::Vec2d;

Obstacle::Obstacle(const std::string &id,
                   const PerceptionObstacle &perception_obstacle)
    : id_(id),
      perception_id_(perception_obstacle.id),
      perception_obstacle_(perception_obstacle),
      perception_bounding_box_(
          {perception_obstacle_.position.x, perception_obstacle_.position.y},
          perception_obstacle_.theta, perception_obstacle_.length,
          perception_obstacle_.width) {
  std::vector<Vec2d> polygon_points;

  for (const auto &point : perception_obstacle.polygon_point) {
    polygon_points.emplace_back(point.x, point.y);
  }

  assert(Polygon2d::ComputeConvexHull(polygon_points, &perception_polygon_));

  is_static_ = IsStaticObstacle(perception_obstacle);

  is_virtual_ = IsVirtualObstacle(perception_obstacle);
  speed_ = std::hypot(perception_obstacle.velocity.x,
                      perception_obstacle.velocity.y);
}

Obstacle::Obstacle(const std::string &id,
                   const PerceptionObstacle &perception_obstacle,
                   const hqplanner::forproto::Trajectory &trajectory)
    : Obstacle(id, perception_obstacle) {
  trajectory_ = trajectory;
  auto &trajectory_points = trajectory_.trajectory_point;
  double cumulative_s = 0.0;
  if (trajectory_points.size() > 0) {
    trajectory_points[0].path_point.s = 0.0;
  }
  for (int i = 1; i < trajectory_points.size(); ++i) {
    const auto &prev = trajectory_points[i - 1];
    const auto &cur = trajectory_points[i];
    // if (prev.relative_time() >= cur.relative_time()) {
    //   AERROR << "prediction time is not increasing."
    //          << "current point: " << cur.ShortDebugString()
    //          << "previous point: " << prev.ShortDebugString();
    // }
    cumulative_s += std::hypot(prev.path_point.x - cur.path_point.x,
                               prev.path_point.y - cur.path_point.y);
    // hqplanner::util::DistanceXY(prev.path_point, cur.path_point);
    trajectory_points[i].path_point.s = cumulative_s;
  }
}

bool Obstacle::IsStaticObstacle(const PerceptionObstacle &perception_obstacle) {
  if (perception_obstacle.type == PerceptionObstacle::UNKNOWN_UNMOVABLE) {
    return true;
  }
  auto moving_speed = std::hypot(perception_obstacle.velocity.x,
                                 perception_obstacle.velocity.y);
  return moving_speed <= 0.5;
}

const PerceptionObstacle &Obstacle::Perception() const {
  return perception_obstacle_;
}
const std::string &Obstacle::Id() const { return id_; }
double Obstacle::Speed() const { return speed_; }

bool Obstacle::IsStatic() const { return is_static_; }

const math::Polygon2d &Obstacle::PerceptionPolygon() const {
  return perception_polygon_;
}

const forproto::Trajectory &Obstacle::Trajectory() const { return trajectory_; }
const math::Box2d &Obstacle::PerceptionBoundingBox() const {
  return perception_bounding_box_;
}
math::Box2d Obstacle::GetBoundingBox() const {
  return perception_bounding_box_;
}

math::Box2d Obstacle::GetBoundingBox(const TrajectoryPoint &point) const {
  return math::Box2d({point.path_point.x, point.path_point.y},
                     point.path_point.theta, perception_obstacle_.length,
                     perception_obstacle_.width);
}

std::shared_ptr<Obstacle> Obstacle::CreateStaticVirtualObstacles(
    const std::string &id, const Box2d &obstacle_box) {
  // create a "virtual" perception_obstacle
  PerceptionObstacle perception_obstacle;
  // simulator needs a valid integer
  std::int32_t negative_id = std::hash<std::string>{}(id);
  // set the first bit to 1 so negative_id became negative number
  negative_id |= (0x1 << 31);
  perception_obstacle.id = negative_id;
  perception_obstacle.position.x = obstacle_box.center().x();
  perception_obstacle.position.y = obstacle_box.center().y();

  // perception_obstacle.mutable_position()->set_x(obstacle_box.center().x());
  // perception_obstacle.mutable_position()->set_y(obstacle_box.center().y());
  perception_obstacle.theta = obstacle_box.heading();

  perception_obstacle.velocity.x = 0;
  perception_obstacle.velocity.y = 0;

  perception_obstacle.length = obstacle_box.length();
  perception_obstacle.width = obstacle_box.width();
  perception_obstacle.height =
      ConfigParam::instance()->FLAGS_virtual_stop_wall_height;
  perception_obstacle.type = PerceptionObstacle::UNKNOWN_UNMOVABLE;
  perception_obstacle.tracking_time = 1.0;

  std::vector<Vec2d> corner_points;
  obstacle_box.GetAllCorners(&corner_points);
  for (const auto &corner_point : corner_points) {
    Point point = {corner_point.x(), corner_point.y(), 0};
    perception_obstacle.polygon_point.emplace_back(std::move(point));
    // auto *point = perception_obstacle.add_polygon_point();
    // point->set_x(corner_point.x());
    // point->set_y(corner_point.y());
  }
  auto *obstacle = new Obstacle(id, perception_obstacle);
  obstacle->is_virtual_ = true;
  return std::shared_ptr<Obstacle>(obstacle);
}
bool Obstacle::IsVirtual() const { return is_virtual_; }

bool Obstacle::IsVirtualObstacle(
    const PerceptionObstacle &perception_obstacle) {
  return perception_obstacle.id < 0;
}

std::list<std::shared_ptr<Obstacle>> Obstacle::CreateObstacles(
    const PredictionObstacles &predictions) {
  std::list<std::shared_ptr<Obstacle>> obstacles;

  for (const auto &prediction_obstacle : predictions.prediction_obstacle) {
    const auto perception_id =
        std::to_string(prediction_obstacle.perception_obstacle.id);

    if (prediction_obstacle.trajectory.empty()) {
      obstacles.emplace_back(
          new Obstacle(perception_id, prediction_obstacle.perception_obstacle));
      continue;
    }

    obstacles.emplace_back(
        new Obstacle(perception_id, prediction_obstacle.perception_obstacle,
                     prediction_obstacle.trajectory.front()));
  }
  return obstacles;
}

TrajectoryPoint Obstacle::GetPointAtTime(const double relative_time) const {
  const auto &points = trajectory_.trajectory_point;
  if (points.size() < 2) {
    TrajectoryPoint point;
    point.path_point.x = perception_obstacle_.position.x;
    point.path_point.y = perception_obstacle_.position.y;
    point.path_point.z = perception_obstacle_.position.z;
    point.path_point.theta = perception_obstacle_.theta;
    point.path_point.s = 0.0;
    point.path_point.kappa = 0.0;
    point.path_point.dkappa = 0.0;
    point.path_point.ddkappa = 0.0;
    point.v = 0.0;
    point.a = 0.0;
    point.relative_time = 0.0;

    return point;
  } else {
    auto comp = [](const TrajectoryPoint p, const double time) {
      return p.relative_time < time;
    };

    auto it_lower =
        std::lower_bound(points.begin(), points.end(), relative_time, comp);

    if (it_lower == points.begin()) {
      return *points.begin();
    } else if (it_lower == points.end()) {
      return *points.rbegin();
    }
    return hqplanner::math::InterpolateUsingLinearApproximation(
        *(it_lower - 1), *it_lower, relative_time);
  }
}

}  // namespace hqplanner
