#ifndef HQPLANNER_CUBIC_SPLINE_H_
#define HQPLANNER_CUBIC_SPLINE_H_

#include <assert.h>

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace hqplanner {
namespace math {

class CubicSpline {
 public:
  CubicSpline() = default;
  explicit CubicSpline(std::vector<double>& x, std::vector<double>& y);
  void CalculateSplineCoefs();
  Eigen::MatrixXd CalculateAMtrix(const std::vector<double>& h);
  Eigen::MatrixXd CalculateBMtrix(const std::vector<double>& h);
  double GetSplinePointValue(double t) const;
  double GetSplinePointFirstDerivativeValue(double t) const;
  double GetSplinePointSecondDerivativeValue(double t) const;
  double GetSplinePointThirdDerivativeValue(double t) const;

 private:
  /* data */
  std::vector<double> spline_coefs_a_;
  std::vector<double> spline_coefs_b_;
  std::vector<double> spline_coefs_c_;
  std::vector<double> spline_coefs_d_;

  std::vector<double> spline_anchor_points_x_;
  std::vector<double> spline_anchor_points_y_;
  int anchor_points_num_ = 0;
};

}  // namespace math
}  // namespace hqplanner

#endif