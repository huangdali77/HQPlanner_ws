
#ifndef HQPLANNER_PATH_DISCRETIZED_PATH_H_
#define HQPLANNER_PATH_DISCRETIZED_PATH_H_

#include <assert.h>

#include <algorithm>
#include <utility>
#include <vector>

#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/math/linear_interpolation.h"
namespace hqplanner {
namespace path {

class DiscretizedPath {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(
      const std::vector<hqplanner::forproto::PathPoint> &path_points);

  virtual ~DiscretizedPath() = default;

  void set_path_points(
      const std::vector<hqplanner::forproto::PathPoint> &path_points);

  double Length() const;

  const hqplanner::forproto::PathPoint &StartPoint() const;

  const hqplanner::forproto::PathPoint &EndPoint() const;

  hqplanner::forproto::PathPoint Evaluate(const double path_s) const;

  const std::vector<hqplanner::forproto::PathPoint> &path_points() const;

  std::uint32_t NumOfPoints() const;

  virtual void Clear();

 protected:
  std::vector<hqplanner::forproto::PathPoint>::const_iterator QueryLowerBound(
      const double path_s) const;

  std::vector<hqplanner::forproto::PathPoint> path_points_;
};

}  // namespace path
}  // namespace hqplanner

#endif
