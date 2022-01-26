#include "hqplanner/speed/st_point.h"

namespace hqplanner {
namespace speed {
STPoint::STPoint(const double s, const double t) : Vec2d(t, s) {}

STPoint::STPoint(const hqplanner::math::Vec2d& vec2d_point)
    : Vec2d(vec2d_point) {}

double STPoint::s() const { return y_; }

double STPoint::t() const { return x_; }

void STPoint::set_s(const double s) { return set_y(s); }

void STPoint::set_t(const double t) { return set_x(t); }
}  // namespace speed
}  // namespace hqplanner