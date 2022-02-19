#ifndef HQPLANNER_CUBIC_SPLINE_START_CLAMPED_H_
#define HQPLANNER_CUBIC_SPLINE_START_CLAMPED_H_
#include "hqplanner/math/cubic_spline.h"

namespace hqplanner {
namespace math {
class CubicSplineStartClamped : public CubicSpline {
 public:
  CubicSplineStartClamped() = default;
  CubicSplineStartClamped(std::vector<double>& x, std::vector<double>& y,
                          double dx0, double ddx0);

  void CalculateSplineCoefs() override;
  //   Eigen::MatrixXd CalculateAMtrix(const std::vector<double>& h) override;
  //   Eigen::MatrixXd CalculateBMtrix(const std::vector<double>& h) override;

 private:
  double dy0_;
  double ddy0_;
};
}  // namespace math
}  // namespace hqplanner
#endif