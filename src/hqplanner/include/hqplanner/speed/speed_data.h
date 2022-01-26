#ifndef HQPLANNER_SPEED_SPEED_DATA_H_
#define HQPLANNER_SPEED_SPEED_DATA_H_

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/math/linear_interpolation.h"
namespace hqplanner {
namespace speed {

class SpeedData {
 public:
  SpeedData() = default;

  explicit SpeedData(std::vector<hqplanner::forproto::SpeedPoint> speed_points);

  virtual ~SpeedData() = default;

  const std::vector<hqplanner::forproto::SpeedPoint>& speed_vector() const;

  void set_speed_vector(
      std::vector<hqplanner::forproto::SpeedPoint> speed_points);

  void AppendSpeedPoint(const double s, const double time, const double v,
                        const double a, const double da);

  bool EvaluateByTime(const double time,
                      hqplanner::forproto::SpeedPoint* const speed_point) const;

  double TotalTime() const;

  bool Empty() const { return speed_vector_.empty(); }

  void Clear();

  //   virtual std::string DebugString() const;

 private:
  std::vector<hqplanner::forproto::SpeedPoint> speed_vector_;
};

}  // namespace speed
}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_SPEED_SPEED_DATA_H_
