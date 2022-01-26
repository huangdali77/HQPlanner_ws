#ifndef HQPLANNER_MATH_LINEAR_INTERPOLATION_H_
#define HQPLANNER_MATH_LINEAR_INTERPOLATION_H_

#include <assert.h>

#include <cmath>

#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/math/math_utils.h"
/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace hqplanner {
namespace math {

/**
 * @brief Linear interpolation between two points of type T.
 * @param x0 The coordinate of the first point.
 * @param t0 The interpolation parameter of the first point.
 * @param x1 The coordinate of the second point.
 * @param t1 The interpolation parameter of the second point.
 * @param t The interpolation parameter for interpolation.
 * @param x The coordinate of the interpolated point.
 * @return Interpolated point.
 */
template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1,
       const double t) {
  if (std::abs(t1 - t0) <= 1.0e-6) {
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x;
}

/**
 * @brief Spherical linear interpolation between two angles.
 *        The two angles are within range [-M_PI, M_PI).
 * @param a0 The value of the first angle.
 * @param t0 The interpolation parameter of the first angle.
 * @param a1 The value of the second angle.
 * @param t1 The interpolation parameter of the second angle.
 * @param t The interpolation parameter for interpolation.
 * @param a The value of the spherically interpolated angle.
 * @return Interpolated angle.
 */
double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t);

hqplanner::forproto::SLPoint InterpolateUsingLinearApproximation(
    const hqplanner::forproto::SLPoint &p0,
    const hqplanner::forproto::SLPoint &p1, const double w);

hqplanner::forproto::PathPoint InterpolateUsingLinearApproximation(
    const hqplanner::forproto::PathPoint &p0,
    const hqplanner::forproto::PathPoint &p1, const double s);

hqplanner::forproto::TrajectoryPoint InterpolateUsingLinearApproximation(
    const hqplanner::forproto::TrajectoryPoint &tp0,
    const hqplanner::forproto::TrajectoryPoint &tp1, const double t);

}  // namespace math

}  // namespace hqplanner

#endif  // MODULES_COMMON_MATH_LINEAR_INTERPOLATION_H_
