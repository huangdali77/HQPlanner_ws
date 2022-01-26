#include "hqplanner/path/frenet_frame_path.h"

namespace hqplanner {
namespace path {
using hqplanner::forproto::FrenetFramePoint;
using hqplanner::forproto::SLBoundary;
FrenetFramePath::FrenetFramePath(
    const std::vector<FrenetFramePoint>& sl_points) {
  points_ = sl_points;
}

void FrenetFramePath::set_points(const std::vector<FrenetFramePoint>& points) {
  points_ = points;
}

const std::vector<FrenetFramePoint>& FrenetFramePath::points() const {
  return points_;
}

double FrenetFramePath::Length() const {
  if (points_.empty()) {
    return 0.0;
  }
  return points_.back().s - points_.front().s;
}

std::uint32_t FrenetFramePath::NumOfPoints() const { return points_.size(); }

const FrenetFramePoint& FrenetFramePath::PointAt(
    const std::uint32_t index) const {
  assert(index < points_.size());

  return points_[index];
}

FrenetFramePoint FrenetFramePath::GetNearestPoint(const SLBoundary& sl) const {
  auto it_lower = std::lower_bound(points_.begin(), points_.end(), sl.start_s,
                                   LowerBoundComparator);
  if (it_lower == points_.end()) {
    return points_.back();
  }
  auto it_upper =
      std::upper_bound(it_lower, points_.end(), sl.end_s, UpperBoundComparator);
  double min_dist = std::numeric_limits<double>::max();
  auto min_it = it_upper;
  for (auto it = it_lower; it != it_upper; ++it) {
    if (it->l >= sl.start_l && it->l <= sl.end_l) {
      return *it;
    } else if (it->l > sl.end_l) {
      double diff = it->l - sl.end_l;
      if (diff < min_dist) {
        min_dist = diff;
        min_it = it;
      }
    } else {
      double diff = sl.start_l - it->l;
      if (diff < min_dist) {
        min_dist = diff;
        min_it = it;
      }
    }
  }
  return *min_it;
}

FrenetFramePoint FrenetFramePath::EvaluateByS(const double s) const {
  assert(points_.size() > 1);
  //   CHECK_GT(points_.size(), 1);
  auto it_lower =
      std::lower_bound(points_.begin(), points_.end(), s, LowerBoundComparator);
  if (it_lower == points_.begin()) {
    return points_.front();
  } else if (it_lower == points_.end()) {
    return points_.back();
  }
  const auto& p0 = *(it_lower - 1);
  const auto s0 = p0.s;
  const auto& p1 = *it_lower;
  const auto s1 = p1.s;

  FrenetFramePoint p;
  p.s = s;
  p.l = hqplanner::math::lerp(p0.l, s0, p1.l, s1, s);
  p.dl = hqplanner::math::lerp(p0.dl, s0, p1.dl, s1, s);
  p.ddl = hqplanner::math::lerp(p0.ddl, s0, p1.ddl, s1, s);
  return p;
}

void FrenetFramePath::Clear() { points_.clear(); }

}  // namespace path
}  // namespace hqplanner
