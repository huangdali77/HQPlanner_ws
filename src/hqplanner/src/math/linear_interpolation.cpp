#include "hqplanner/math/linear_interpolation.h"

namespace hqplanner {
namespace math {

using hqplanner::forproto::PathPoint;
using hqplanner::forproto::SLPoint;
using hqplanner::forproto::TrajectoryPoint;

double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    return NormalizeAngle(a0);
  }
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return NormalizeAngle(a);
}

SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0,
                                            const SLPoint &p1, const double w) {
  //   CHECK_GE(w, 0.0);
  assert(w >= 0);
  SLPoint p;
  p.s = (1 - w) * p0.s + w * p1.s;

  p.l = (1 - w) * p0.l + w * p1.l;
  return p;
}

PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const double s) {
  double s0 = p0.s;
  double s1 = p1.s;
  assert(s0 <= s1);
  //   CHECK_LE(s0, s1);

  PathPoint path_point;
  double weight = (s - s0) / (s1 - s0);
  double x = (1 - weight) * p0.x + weight * p1.x;
  double y = (1 - weight) * p0.y + weight * p1.y;
  double theta = slerp(p0.theta, p0.s, p1.theta, p1.s, s);
  double kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
  double dkappa = (1 - weight) * p0.dkappa + weight * p1.dkappa;
  double ddkappa = (1 - weight) * p0.ddkappa + weight * p1.ddkappa;
  path_point.x = x;
  path_point.y = y;
  path_point.theta = theta;
  path_point.kappa = kappa;
  path_point.dkappa = dkappa;
  path_point.ddkappa = ddkappa;
  path_point.s = s;
  return path_point;
}

TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &tp0,
                                                    const TrajectoryPoint &tp1,
                                                    const double t) {
  //   if (!tp0.has_path_point() || !tp1.has_path_point()) {
  //     TrajectoryPoint p;
  //     p.mutable_path_point()->CopyFrom(PathPoint());
  //     return p;
  //   }
  const PathPoint pp0 = tp0.path_point;
  const PathPoint pp1 = tp1.path_point;
  double t0 = tp0.relative_time;
  double t1 = tp1.relative_time;

  TrajectoryPoint tp;
  tp.v = lerp(tp0.v, t0, tp1.v, t1, t);
  tp.a = lerp(tp0.a, t0, tp1.a, t1, t);
  tp.relative_time = t;

  PathPoint path_point;
  //   PathPoint *path_point = tp.mutable_path_point();
  path_point.x = lerp(pp0.x, t0, pp1.x, t1, t);
  path_point.y = lerp(pp0.y, t0, pp1.y, t1, t);
  path_point.theta = slerp(pp0.theta, t0, pp1.theta, t1, t);
  path_point.kappa = lerp(pp0.kappa, t0, pp1.kappa, t1, t);
  path_point.dkappa = lerp(pp0.dkappa, t0, pp1.dkappa, t1, t);
  path_point.ddkappa = lerp(pp0.ddkappa, t0, pp1.ddkappa, t1, t);
  path_point.s = lerp(pp0.s, t0, pp1.s, t1, t);
  tp.path_point = path_point;
  return tp;
}
}  // namespace math

}  // namespace hqplanner
