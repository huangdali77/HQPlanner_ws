
#include "hqplanner/path/discretized_path.h"

namespace hqplanner {
namespace path {
using hqplanner::forproto::PathPoint;

DiscretizedPath::DiscretizedPath(const std::vector<PathPoint> &path_points) {
  path_points_ = path_points;
}

void DiscretizedPath::set_path_points(
    const std::vector<PathPoint> &path_points) {
  path_points_ = path_points;
}

double DiscretizedPath::Length() const {
  if (path_points_.empty()) {
    return 0.0;
  }
  return path_points_.back().s - path_points_.front().s;
}

PathPoint DiscretizedPath::Evaluate(const double path_s) const {
  assert(!path_points_.empty());

  auto it_lower = QueryLowerBound(path_s);
  if (it_lower == path_points_.begin()) {
    return path_points_.front();
  }
  if (it_lower == path_points_.end()) {
    return path_points_.back();
  }
  return hqplanner::math ::InterpolateUsingLinearApproximation(
      *(it_lower - 1), *it_lower, path_s);
}

const std::vector<PathPoint> &DiscretizedPath::path_points() const {
  return path_points_;
}

std::uint32_t DiscretizedPath::NumOfPoints() const {
  return path_points_.size();
}

const PathPoint &DiscretizedPath::StartPoint() const {
  assert(!path_points_.empty());
  return path_points_.front();
}

const PathPoint &DiscretizedPath::EndPoint() const {
  assert(!path_points_.empty());
  return path_points_.back();
}

void DiscretizedPath::Clear() { path_points_.clear(); }

std::vector<PathPoint>::const_iterator DiscretizedPath::QueryLowerBound(
    const double path_s) const {
  auto func = [](const PathPoint &tp, const double path_s) {
    return tp.s < path_s;
  };
  return std::lower_bound(path_points_.begin(), path_points_.end(), path_s,
                          func);
}

}  // namespace path
}  // namespace hqplanner
