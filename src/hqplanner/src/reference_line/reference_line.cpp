#include "hqplanner/reference_line/reference_line.h"

#include <ros/ros.h>
namespace hqplanner {
using hqplanner::forproto::AnchorPoint;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::ReferencePoint;
using hqplanner::forproto::SLBoundary;
using hqplanner::forproto::SLPoint;
using hqplanner::math::Box2d;
using hqplanner::math::CubicSpline;
using hqplanner::math::LineSegment2d;
using hqplanner::math::Vec2d;

double REFERENCE_LINE_SAMPLE_STEP = 0.1;

ReferenceLine::ReferenceLine(const std::vector<AnchorPoint> anchor_points)
    : anchor_points_(anchor_points) {
  // std::vector<double> x;
  // std::vector<double> y;
  for (auto anchor_point : anchor_points) {
    anchor_points_x_.push_back(anchor_point.cartesian_x);
    anchor_points_y_.push_back(anchor_point.cartesian_y);
  }

  AccumulateOnS();
  anchor_points_x_s_ = CubicSpline(anchor_points_s_, anchor_points_x_);
  anchor_points_y_s_ = CubicSpline(anchor_points_s_, anchor_points_y_);
  ConstructReferenceLineByFixedStep();  //根据上游信息给出路由信息，供寻找最近点使用，
}

ReferenceLine::ReferenceLine(const std::vector<double> &x,
                             const std::vector<double> &y)
    : anchor_points_x_(x), anchor_points_y_(y) {
  std::vector<AnchorPoint> anchor_points(x.size());

  for (int i = 0; i < x.size(); ++i) {
    anchor_points[i].cartesian_x = x[i];
    anchor_points[i].cartesian_y = y[i];
  }
  anchor_points_ = anchor_points;

  AccumulateOnS();
  anchor_points_x_s_ = CubicSpline(anchor_points_s_, anchor_points_x_);
  anchor_points_y_s_ = CubicSpline(anchor_points_s_, anchor_points_y_);
  ConstructReferenceLineByFixedStep();  //根据上游信息给出路由信息，供寻找最近点使用，
}

void ReferenceLine::AccumulateOnS() {
  anchor_points_[0].frenet_s = 0;
  anchor_points_s_.resize(anchor_points_.size());
  anchor_points_s_[0] = 0;

  for (int i = 1; i < anchor_points_.size(); ++i) {
    double ds = hypot(
        anchor_points_[i].cartesian_x - anchor_points_[i - 1].cartesian_x,
        anchor_points_[i].cartesian_y - anchor_points_[i - 1].cartesian_y);
    anchor_points_[i].frenet_s = anchor_points_[i - 1].frenet_s + ds;
    anchor_points_s_[i] = anchor_points_[i].frenet_s;
  }
}

double ReferenceLine::ComputeCurvature(double dx, double ddx, double dy,
                                       double ddy) const {
  double a = dx * ddy - dy * ddx;
  double norm_square = dx * dx + dy * dy;
  double norm = sqrt(norm_square);
  double b = norm * norm_square;
  return a / b;
}

double ReferenceLine::ComputeCurvatureDerivative(double dx, double ddx,
                                                 double dddx, double dy,
                                                 double ddy,
                                                 double dddy) const {
  double a = dx * ddy - dy * ddx;
  double b = dx * dddy - dy * dddx;
  double c = dx * ddx + dy * ddy;
  double d = dx * dx + dy * dy;
  return (b * d - 3.0 * a * c) / (d * d * d);
}

bool ReferenceLine::SLToXY(const SLPoint &sl_point,
                           Vec2d *const xy_point) const {
  assert(xy_point != nullptr);

  if (reference_line_points_.size() < 2) {
    return false;
  }

  const auto angle = GetHeadingByS(sl_point.s);
  const auto x = GetPositionXByS(sl_point.s);
  const auto y = GetPositionYByS(sl_point.s);

  xy_point->set_x(x - std::sin(angle) * sl_point.l);
  xy_point->set_y(y + std::cos(angle) * sl_point.l);
  return true;
}

bool ReferenceLine::XYToSL(const Vec2d &xy_point,
                           SLPoint *const sl_point) const {
  assert(sl_point != nullptr);
  assert(reference_line_points_.size() >= 2);
  // DCHECK_NOTNULL(sl_point);
  double s = 0.0;
  double l = 0.0;

  // 寻找最近点
  double min_distance = std::numeric_limits<double>::infinity();
  int index_min = 0;
  int start_index = 0;
  for (int i = 0; i < reference_line_points_.size(); ++i) {
    const auto ref_point = reference_line_points_[i];
    double square_distance =
        xy_point.DistanceSquareTo(Vec2d(ref_point.x, ref_point.y));
    if (square_distance < min_distance) {
      min_distance = square_distance;
      index_min = i;
    }
  }
  LineSegment2d closest_segment;
  if (index_min == 0) {
    closest_segment = LineSegment2d(
        Vec2d(reference_line_points_[0].x, reference_line_points_[0].y),
        Vec2d(reference_line_points_[1].x, reference_line_points_[1].y));
    start_index = 0;
  } else if (index_min == reference_line_points_.size() - 1) {
    closest_segment =
        LineSegment2d(Vec2d(reference_line_points_[index_min - 1].x,
                            reference_line_points_[index_min - 1].y),
                      Vec2d(reference_line_points_[index_min].x,
                            reference_line_points_[index_min].y));
    start_index = index_min - 1;
  } else {
    double square_dist1 = xy_point.DistanceSquareTo(
        Vec2d(reference_line_points_[index_min - 1].x,
              reference_line_points_[index_min - 1].y));
    double square_dist2 = xy_point.DistanceSquareTo(
        Vec2d(reference_line_points_[index_min + 1].x,
              reference_line_points_[index_min + 1].y));
    if (square_dist1 < square_dist2) {
      closest_segment =
          LineSegment2d(Vec2d(reference_line_points_[index_min - 1].x,
                              reference_line_points_[index_min - 1].y),
                        Vec2d(reference_line_points_[index_min].x,
                              reference_line_points_[index_min].y));
      start_index = index_min - 1;
    } else {
      closest_segment =
          LineSegment2d(Vec2d(reference_line_points_[index_min].x,
                              reference_line_points_[index_min].y),
                        Vec2d(reference_line_points_[index_min + 1].x,
                              reference_line_points_[index_min + 1].y));
      start_index = index_min;
    }
  }

  const double distance = closest_segment.DistanceTo(xy_point);
  const double proj = closest_segment.ProjectOntoUnit(xy_point);
  const double prod = closest_segment.ProductOntoUnit(xy_point);

  sl_point->s = reference_line_points_[start_index].s +
                std::min(proj, closest_segment.length());
  sl_point->l = (prod > 0.0 ? distance : -distance);
  return true;
}

double ReferenceLine::GetPositionXByS(double i_s) const {
  return anchor_points_x_s_.GetSplinePointValue(i_s);
}

double ReferenceLine::GetPositionYByS(double i_s) const {
  return anchor_points_y_s_.GetSplinePointValue(i_s);
}

double ReferenceLine::GetHeadingByS(double i_s) const {
  double i_dx = anchor_points_x_s_.GetSplinePointFirstDerivativeValue(i_s);
  double i_dy = anchor_points_y_s_.GetSplinePointFirstDerivativeValue(i_s);
  return atan2(i_dy, i_dx);
}

double ReferenceLine::GetKappaByS(double i_s) const {
  double i_dx = anchor_points_x_s_.GetSplinePointFirstDerivativeValue(i_s);
  double i_dy = anchor_points_y_s_.GetSplinePointFirstDerivativeValue(i_s);
  double i_ddx = anchor_points_x_s_.GetSplinePointSecondDerivativeValue(i_s);
  double i_ddy = anchor_points_y_s_.GetSplinePointSecondDerivativeValue(i_s);
  return ComputeCurvature(i_dx, i_ddx, i_dy, i_ddy);
}

double ReferenceLine::GetKappaDerivativeByS(double i_s) const {
  double i_dx = anchor_points_x_s_.GetSplinePointFirstDerivativeValue(i_s);
  double i_dy = anchor_points_y_s_.GetSplinePointFirstDerivativeValue(i_s);
  double i_ddx = anchor_points_x_s_.GetSplinePointSecondDerivativeValue(i_s);
  double i_ddy = anchor_points_y_s_.GetSplinePointSecondDerivativeValue(i_s);
  double i_dddx = anchor_points_x_s_.GetSplinePointThirdDerivativeValue(i_s);
  double i_dddy = anchor_points_y_s_.GetSplinePointThirdDerivativeValue(i_s);
  return ComputeCurvatureDerivative(i_dx, i_ddx, i_dddx, i_dy, i_ddy, i_dddy);
}

ReferencePoint ReferenceLine::GetReferencePoint(const double s) const {
  ReferencePoint ref_point(s);

  ref_point.x = anchor_points_x_s_.GetSplinePointValue(s);
  ref_point.y = anchor_points_y_s_.GetSplinePointValue(s);

  double dx = anchor_points_x_s_.GetSplinePointFirstDerivativeValue(s);
  double dy = anchor_points_y_s_.GetSplinePointFirstDerivativeValue(s);
  ref_point.heading = atan2(dy, dx);

  double ddx = anchor_points_x_s_.GetSplinePointSecondDerivativeValue(s);
  double ddy = anchor_points_y_s_.GetSplinePointSecondDerivativeValue(s);
  ref_point.kappa = ComputeCurvature(dx, ddx, dy, ddy);

  double dddx = anchor_points_x_s_.GetSplinePointThirdDerivativeValue(s);
  double dddy = anchor_points_y_s_.GetSplinePointThirdDerivativeValue(s);
  ref_point.dkappa = ComputeCurvatureDerivative(dx, ddx, dddx, dy, ddy, dddy);

  return ref_point;
}

bool ReferenceLine::GetSLBoundary(const Box2d &box,
                                  SLBoundary *const sl_boundary) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());
  std::vector<Vec2d> corners;
  box.GetAllCorners(&corners);
  for (const auto &point : corners) {
    SLPoint sl_point;
    if (!XYToSL(point, &sl_point)) {
      return false;
    }
    start_s = std::fmin(start_s, sl_point.s);
    end_s = std::fmax(end_s, sl_point.s);
    start_l = std::fmin(start_l, sl_point.l);
    end_l = std::fmax(end_l, sl_point.l);
  }
  sl_boundary->start_s = start_s;
  sl_boundary->end_s = end_s;
  sl_boundary->start_l = start_l;
  sl_boundary->end_l = end_l;
  return true;
}

bool ReferenceLine::GetApproximateSLBoundary(
    const Box2d &box, SLBoundary *const sl_boundary) const {
  SLPoint box_center_sl_point;
  if (!XYToSL(box.center(), &box_center_sl_point)) {
    return false;
  }
  ReferencePoint reference_point = GetReferencePoint(box_center_sl_point.s);

  auto rotated_box = box;
  rotated_box.RotateFromCenter(-reference_point.heading);

  std::vector<Vec2d> corners;
  rotated_box.GetAllCorners(&corners);

  double min_s(std::numeric_limits<double>::max());
  double max_s(std::numeric_limits<double>::lowest());
  double min_l(std::numeric_limits<double>::max());
  double max_l(std::numeric_limits<double>::lowest());

  for (const auto &point : corners) {
    // x <--> s, y <--> l
    // because the box is rotated to align the reference line
    min_s = std::fmin(
        min_s, point.x() - rotated_box.center().x() + box_center_sl_point.s);
    max_s = std::fmax(
        max_s, point.x() - rotated_box.center().x() + box_center_sl_point.s);
    min_l = std::fmin(
        min_l, point.y() - rotated_box.center().y() + box_center_sl_point.l);
    max_l = std::fmax(
        max_l, point.y() - rotated_box.center().y() + box_center_sl_point.l);
  }
  sl_boundary->start_s = min_s;
  sl_boundary->end_s = max_s;
  sl_boundary->start_l = min_l;
  sl_boundary->end_l = max_l;
  return true;
}

void ReferenceLine::ConstructReferenceLineByFixedStep() {
  int reference_line_points_num =
      int(anchor_points_s_.back() /
          ConfigParam::instance()->FLAGS_reference_line_sample_step);
  std::vector<double> ref_s(reference_line_points_num, 0);
  for (int i = 1; i < ref_s.size(); ++i) {
    ref_s[i] = ref_s[i - 1] +
               ConfigParam::instance()->FLAGS_reference_line_sample_step;
  }
  accumulated_s_ = ref_s;
  for (auto i_s : ref_s) {
    reference_line_points_.emplace_back();
    reference_line_points_.back().s = i_s;
    // x

    double i_x = anchor_points_x_s_.GetSplinePointValue(i_s);
    reference_line_points_.back().x = i_x;
    // y
    double i_y = anchor_points_y_s_.GetSplinePointValue(i_s);
    reference_line_points_.back().y = i_y;

    // yaw
    double i_dx = anchor_points_x_s_.GetSplinePointFirstDerivativeValue(i_s);
    double i_dy = anchor_points_y_s_.GetSplinePointFirstDerivativeValue(i_s);
    reference_line_points_.back().heading = atan2(i_dy, i_dx);

    // curvature
    double i_ddx = anchor_points_x_s_.GetSplinePointSecondDerivativeValue(i_s);
    double i_ddy = anchor_points_y_s_.GetSplinePointSecondDerivativeValue(i_s);
    reference_line_points_.back().kappa =
        ComputeCurvature(i_dx, i_ddx, i_dy, i_ddy);

    // d_curvature
    double i_dddx = anchor_points_x_s_.GetSplinePointThirdDerivativeValue(i_s);
    double i_dddy = anchor_points_y_s_.GetSplinePointThirdDerivativeValue(i_s);
    reference_line_points_.back().dkappa =
        ComputeCurvatureDerivative(i_dx, i_ddx, i_dddx, i_dy, i_ddy, i_dddy);
  }
  route_points_ = reference_line_points_;
}

bool ReferenceLine::GetLaneWidth(const double s, double *const lane_left_width,
                                 double *const lane_right_width) const {
  *lane_left_width = ConfigParam::instance()->FLAGS_lane_left_width;
  *lane_right_width = ConfigParam::instance()->FLAGS_lane_right_width;
  return true;
}

bool ReferenceLine::Shrink(const Vec2d &point, double look_backward,
                           double look_forward) {
  reference_line_points_ = route_points_;
  SLPoint sl;
  if (!XYToSL(point, &sl)) {
    // AERROR << "Failed to project point: " << point.DebugString();
    return false;
  }
  // const auto& accumulated_s = map_path_.accumulated_s();
  size_t start_index = 0;
  if (sl.s > look_backward) {
    auto it_lower = std::lower_bound(
        accumulated_s_.begin(), accumulated_s_.end(), sl.s - look_backward);
    start_index = std::distance(accumulated_s_.begin(), it_lower);
  }
  size_t end_index = accumulated_s_.size();
  if (sl.s + look_forward < Length()) {
    auto start_it = accumulated_s_.begin();
    std::advance(start_it, start_index);
    auto it_higher =
        std::upper_bound(start_it, accumulated_s_.end(), sl.s + look_forward);
    end_index = std::distance(accumulated_s_.begin(), it_higher);
  }
  reference_line_points_.erase(reference_line_points_.begin() + end_index,
                               reference_line_points_.end());
  reference_line_points_.erase(reference_line_points_.begin(),
                               reference_line_points_.begin() + start_index);
  if (reference_line_points_.size() < 2) {
    // AERROR << "Too few reference points after shrinking.";
    return false;
  }
  // map_path_ = MapPath(std::move(std::vector<hdmap::MapPathPoint>(
  //     reference_points_.begin(), reference_points_.end())));
  return true;
}

double ReferenceLine::GetSpeedLimitFromS(const double s) const {
  // ================NoNeed==================================
  // for (const auto &speed_limit : speed_limit_) {
  //   if (s >= speed_limit.start_s && s <= speed_limit.end_s) {
  //     return speed_limit.speed_limit;
  //   }
  // }
  // const auto& map_path_point = GetReferencePoint(s);
  // double speed_limit =
  //     ConfigParam::instance()->FLAGS_planning_upper_speed_limit;
  // for (const auto& lane_waypoint : map_path_point.lane_waypoints()) {
  //   if (lane_waypoint.lane == nullptr) {
  //     AWARN << "lane_waypoint.lane is nullptr";
  //     continue;
  //   }
  //   speed_limit =
  //       std::fmin(lane_waypoint.lane->lane().speed_limit(), speed_limit);
  // }
  return ConfigParam::instance()->FLAGS_planning_upper_speed_limit;
}

void ReferenceLine::AddSpeedLimit(double start_s, double end_s,
                                  double speed_limit) {
  // assume no overlaps between speed limit regions.
  speed_limit_.emplace_back(start_s, end_s, speed_limit);
}

bool ReferenceLine::IsOnRoad(const hqplanner::math::Vec2d &vec2d_point) const {
  SLPoint sl_point;
  if (!XYToSL(vec2d_point, &sl_point)) {
    return false;
  }
  return IsOnRoad(sl_point);
}

bool ReferenceLine::IsOnRoad(const SLBoundary &sl_boundary) const {
  if (sl_boundary.end_s < 0 ||
      sl_boundary.start_s > reference_line_points_.back().s) {
    return false;
  }
  double middle_s = (sl_boundary.start_s + sl_boundary.end_s) / 2.0;
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;

  if (!GetLaneWidth(middle_s, &lane_left_width, &lane_right_width)) {
    return false;
  }

  return !(sl_boundary.start_l > lane_left_width ||
           sl_boundary.end_l < -lane_right_width);
}

// bool ReferenceLine::IsBlockRoad(const common::math::Box2d& box2d,
//                                 double gap) const {
//   return map_path_.OverlapWith(box2d, gap);
// }

bool ReferenceLine::IsOnRoad(const SLPoint &sl_point) const {
  if (sl_point.s <= 0 || sl_point.s > reference_line_points_.back().s) {
    return false;
  }
  double left_width = 0.0;
  double right_width = 0.0;

  if (!GetLaneWidth(sl_point.s, &left_width, &right_width)) {
    return false;
  }

  return !(sl_point.l < -right_width || sl_point.l > left_width);
}

}  // namespace hqplanner
