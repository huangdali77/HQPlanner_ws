#ifndef HQPLANNER_CUBIC_SPLINE_CLAMPED_H_
#define HQPLANNER_CUBIC_SPLINE_CLAMPED_H_
#include "hqplanner/math/cubic_spline.h"

namespace hqplanner {
namespace math {
class CubicSplineClamped : public CubicSpline {
 public:
  CubicSplineClamped() = default;
  CubicSplineClamped(std::vector<double>& x, std::vector<double>& y, double dx0,
                     double dxn);

  //   void CalculateSplineCoefs() override;
  Eigen::MatrixXd CalculateAMtrix(const std::vector<double>& h) override;
  Eigen::MatrixXd CalculateBMtrix(const std::vector<double>& h) override;

 private:
  double dx0_;
  double dxn_;
};
}  // namespace math
}  // namespace hqplanner
#endif