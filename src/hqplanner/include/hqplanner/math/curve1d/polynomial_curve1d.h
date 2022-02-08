
#ifndef HQPLANNER_MATH_CURVE1D_POLYNOMIAL_CURVE1D_H_
#define HQPLANNER_MATH_CURVE1D_POLYNOMIAL_CURVE1D_H_
#include "hqplanner/math/curve1d/curve1d.h"

namespace hqplanner {
namespace math {

class PolynomialCurve1d : public Curve1d {
 public:
  PolynomialCurve1d() = default;
  virtual ~PolynomialCurve1d() = default;

 protected:
  double param_ = 0.0;
};

}  // namespace math
}  // namespace hqplanner

#endif