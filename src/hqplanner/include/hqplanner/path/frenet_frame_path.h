#ifndef HQPLANNER_PATH_FRENET_FRAME_PATH_H_
#define HQPLANNER_PATH_FRENET_FRAME_PATH_H_

#include <assert.h>

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/sl_boundary.h"
#include "hqplanner/math/linear_interpolation.h"
namespace hqplanner {
namespace path {

class FrenetFramePath {
 public:
  FrenetFramePath() = default;
  explicit FrenetFramePath(
      const std::vector<hqplanner::forproto::FrenetFramePoint>& sl_points);
  virtual ~FrenetFramePath() = default;

  void set_points(
      const std::vector<hqplanner::forproto::FrenetFramePoint>& points);
  const std::vector<hqplanner::forproto::FrenetFramePoint>& points() const;
  std::uint32_t NumOfPoints() const;
  double Length() const;
  const hqplanner::forproto::FrenetFramePoint& PointAt(
      const std::uint32_t index) const;
  hqplanner::forproto::FrenetFramePoint EvaluateByS(const double s) const;

  /**
   * @brief Get the FrenetFramePoint that is within SLBoundary, or the one with
   * smallest l() in SLBoundary's s range [start_s(), end_s()]
   */
  hqplanner::forproto::FrenetFramePoint GetNearestPoint(
      const hqplanner::forproto::SLBoundary& sl) const;

  virtual void Clear();

 private:
  static bool LowerBoundComparator(
      const hqplanner::forproto::FrenetFramePoint& p, const double s) {
    return p.s < s;
  }
  static bool UpperBoundComparator(
      const double s, const hqplanner::forproto::FrenetFramePoint& p) {
    return s < p.s;
  }

  std::vector<hqplanner::forproto::FrenetFramePoint> points_;
};

}  // namespace path
}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_PATH_FRENET_FRAME_PATH_H_
