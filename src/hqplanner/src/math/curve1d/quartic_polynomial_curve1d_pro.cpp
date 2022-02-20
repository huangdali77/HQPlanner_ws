#include "hqplanner/math/curve1d/quartic_polynomial_curve1d_pro.h"

#include <assert.h>

namespace hqplanner {
namespace math {

QuarticPolynomialCurve1dPro::QuarticPolynomialCurve1dPro(
    const std::array<double, 3>& start, const std::array<double, 2>& end,
    const double param)
    : QuarticPolynomialCurve1dPro(start[0], start[1], start[2], end[0], end[1],
                                  param) {}

QuarticPolynomialCurve1dPro::QuarticPolynomialCurve1dPro(
    const double x0, const double dx0, const double ddx0, const double x1,
    const double dx1, const double param) {
  param_ = param;
  start_condition_[0] = x0;
  start_condition_[1] = dx0;
  start_condition_[2] = ddx0;
  end_condition_[0] = x1;
  end_condition_[1] = dx1;
  ComputeCoefficients(x0, dx0, ddx0, x1, dx1, param);
}

QuarticPolynomialCurve1dPro::QuarticPolynomialCurve1dPro(
    const QuarticPolynomialCurve1dPro& other) {
  param_ = other.param_;
  coef_ = other.coef_;
}

double QuarticPolynomialCurve1dPro::Evaluate(const std::uint32_t order,
                                             const double p) const {
  switch (order) {
    case 0: {
      return (((coef_[4] * p + coef_[3]) * p + coef_[2]) * p + coef_[1]) * p +
             coef_[0];
    }
    case 1: {
      return ((4.0 * coef_[4] * p + 3.0 * coef_[3]) * p + 2.0 * coef_[2]) * p +
             coef_[1];
    }
    case 2: {
      return (12.0 * coef_[4] * p + 6.0 * coef_[3]) * p + 2.0 * coef_[2];
    }
    case 3: {
      return 24.0 * coef_[4] * p + 6.0 * coef_[3];
    }
    case 4: {
      return 24.0 * coef_[4];
    }
    default:
      return 0.0;
  }
}

void QuarticPolynomialCurve1dPro::ComputeCoefficients(
    const double x0, const double dx0, const double ddx0, const double x1,
    const double dx1, const double p) {
  assert(p > 0.0);
  //   CHECK_GT(p, 0.0);

  coef_[0] = x0;
  coef_[1] = dx0;
  coef_[2] = 0.5 * ddx0;

  double p2 = p * p;
  double p3 = p2 * p;
  double p4 = p2 * p2;
  coef_[3] = (4.0 * (x1 - x0) - 3.0 * dx0 * p - dx1 * p - ddx0 * p2) / p3;

  coef_[4] = (3.0 * (x0 - x1) + 2.0 * dx0 * p + dx1 * p + 0.5 * ddx0 * p2) / p4;
}

}  // namespace math
}  // namespace hqplanner
