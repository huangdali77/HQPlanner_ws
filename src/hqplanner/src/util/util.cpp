#include "hqplanner/util/util.h"

#include <cmath>
#include <vector>

namespace hqplanner {

namespace util {
using hqplanner::forproto::PathPoint;
using hqplanner::forproto::SLPoint;
using hqplanner::forproto::SpeedPoint;
using hqplanner::math::Vec2d;

SLPoint MakeSLPoint(const double s, const double l) {
  SLPoint sl;
  sl.s = s;
  sl.l = l;
  return sl;
}

// PointENU MakePointENU(const double x, const double y, const double z) {
//   PointENU point_enu;
//   point_enu.set_x(x);
//   point_enu.set_y(y);
//   point_enu.set_z(z);
//   return point_enu;
// }

// PointENU operator+(const PointENU enu, const math::Vec2d& xy) {
//   PointENU point;
//   point.set_x(enu.x() + xy.x());
//   point.set_y(enu.y() + xy.y());
//   point.set_z(enu.z());
//   return point;
// }

// PointENU MakePointENU(const math::Vec2d& xy) {
//   PointENU point_enu;
//   point_enu.set_x(xy.x());
//   point_enu.set_y(xy.y());
//   point_enu.set_z(0.0);
//   return point_enu;
// }

// apollo::perception::Point MakePerceptionPoint(const double x, const double y,
//                                               const double z) {
//   perception::Point point3d;
//   point3d.set_x(x);
//   point3d.set_y(y);
//   point3d.set_z(z);
//   return point3d;
// }

SpeedPoint MakeSpeedPoint(const double s, const double t, const double v,
                          const double a, const double da) {
  SpeedPoint speed_point;
  speed_point.s = s;
  speed_point.t = t;

  speed_point.v = v;

  speed_point.a = a;

  speed_point.da = da;
  return speed_point;
}

PathPoint MakePathPoint(const double x, const double y, const double z,
                        const double theta, const double kappa,
                        const double dkappa, const double ddkappa) {
  PathPoint path_point;

  path_point.x = x;

  path_point.y = y;

  path_point.z = z;

  path_point.theta = theta;

  path_point.kappa = kappa;

  path_point.dkappa = dkappa;

  path_point.ddkappa = ddkappa;
  return path_point;
}

PathPoint GetWeightedAverageOfTwoPathPoints(const PathPoint& p1,
                                            const PathPoint& p2,
                                            const double w1, const double w2) {
  PathPoint p;
  p.x = p1.x * w1 + p2.x * w2;

  p.y = p1.y * w1 + p2.y * w2;

  p.z = p1.z * w1 + p2.z * w2;

  p.theta = p1.theta * w1 + p2.theta * w2;

  p.kappa = p1.kappa * w1 + p2.kappa * w2;

  p.dkappa = p1.dkappa * w1 + p2.dkappa * w2;

  p.ddkappa = p1.ddkappa * w1 + p2.ddkappa * w2;

  p.s = p1.s * w1 + p2.s * w2;
  return p;
}

}  // namespace util

}  // namespace hqplanner
