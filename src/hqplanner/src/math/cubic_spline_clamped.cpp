#include "hqplanner/math/cubic_spline_clamped.h"
namespace hqplanner {
namespace math {

using namespace Eigen;

CubicSplineClamped::CubicSplineClamped(std::vector<double>& x,
                                       std::vector<double>& y, double dx0,
                                       double dxn)
    : dx0_(dx0), dxn_(dxn) {
  spline_anchor_points_x_ = x;
  spline_anchor_points_y_ = y;
  anchor_points_num_ = spline_anchor_points_x_.size();
  CalculateSplineCoefs();
}

MatrixXd CubicSplineClamped::CalculateAMtrix(const std::vector<double>& h) {
  MatrixXd A = MatrixXd::Zero(anchor_points_num_, anchor_points_num_);

  A(0, 0) = 2.0 * h[0];
  for (int i = 0; i < anchor_points_num_ - 1; ++i) {
    if (i != anchor_points_num_ - 2) {
      A(i + 1, i + 1) = 2.0 * (h[i] + h[i + 1]);
    }
    A(i + 1, i) = h[i];
    A(i, i + 1) = h[i];
  }

  A(anchor_points_num_ - 1, anchor_points_num_ - 1) =
      2.0 * h[anchor_points_num_ - 2];

  return A;
}

MatrixXd CubicSplineClamped::CalculateBMtrix(const std::vector<double>& h) {
  MatrixXd b = MatrixXd::Zero(anchor_points_num_, 1);
  for (int i = 0; i < anchor_points_num_ - 2; ++i) {
    b(i + 1, 0) =
        3.0 * (spline_coefs_a_[i + 2] - spline_coefs_a_[i + 1]) / h[i + 1] -
        3.0 * (spline_coefs_a_[i + 1] - spline_coefs_a_[i]) / h[i];
  }

  b(0, 0) = 3.0 * (spline_coefs_a_[1] - spline_coefs_a_[0]) / h[0] - 3.0 * dx0_;
  b(anchor_points_num_ - 1, 0) =
      3.0 * dxn_ - 3.0 *
                       (spline_coefs_a_[anchor_points_num_ - 1] -
                        spline_coefs_a_[anchor_points_num_ - 2]) /
                       h[anchor_points_num_ - 2];
  return b;
}

}  // namespace math
}  // namespace hqplanner