#ifndef HQPLANNER_MATH_CURVE1D_QUARTIC_POLYNOMIAL_CURVE1D_PRO_H_
#define HQPLANNER_MATH_CURVE1D_QUARTIC_POLYNOMIAL_CURVE1D_PRO_H_

#include <array>
#include <string>

#include "hqplanner/math/curve1d/polynomial_curve1d.h"

namespace hqplanner {
namespace math {

// 1D quartic polynomial curve: (x0, dx0, ddx0) -- [0, param] --> (x1, dx1)
class QuarticPolynomialCurve1dPro : public PolynomialCurve1d {
 public:
  QuarticPolynomialCurve1dPro() = default;

  QuarticPolynomialCurve1dPro(const std::array<double, 3>& start,
                              const std::array<double, 2>& end,
                              const double param);

  QuarticPolynomialCurve1dPro(const double x0, const double dx0,
                              const double ddx0, const double x1,
                              const double dx1, const double param);

  QuarticPolynomialCurve1dPro(const QuarticPolynomialCurve1dPro& other);

  virtual ~QuarticPolynomialCurve1dPro() = default;

  double Evaluate(const std::uint32_t order, const double p) const override;

  double ParamLength() const override { return param_; }
  // std::string ToString() const override;

 private:
  void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                           const double x1, const double dx1,
                           const double param);

  std::array<double, 5> coef_ = {{0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> start_condition_ = {{0.0, 0.0, 0.0}};
  std::array<double, 2> end_condition_ = {{0.0, 0.0}};
};

}  // namespace math
}  // namespace hqplanner

#endif  // MODULES_PLANNING_MATH_CURVE1D_QUARTIC_POLYNOMIAL_CURVE1D_H_
