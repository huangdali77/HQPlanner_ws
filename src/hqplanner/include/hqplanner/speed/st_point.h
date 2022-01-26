#ifndef HQPLANNER_SPEED_ST_POINT_H_
#define HQPLANNER_SPEED_ST_POINT_H_

#include <iomanip>
#include <string>

#include "math/vec2d.h"
namespace hqplanner {
namespace speed {

class STPoint : public hqplanner::math::Vec2d {
 public:
  STPoint() = default;
  STPoint(const double s, const double t);
  explicit STPoint(const hqplanner::math::Vec2d& vec2d_point);

  double s() const;
  double t() const;
  void set_s(const double s);
  void set_t(const double t);
  std::string DebugString() const;
};

}  // namespace speed
}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_SPEED_ST_POINT_H_
