#include "hqplanner/math/cubic_spline_start_clamped.h"
namespace hqplanner {
namespace math {

using namespace Eigen;

CubicSplineStartClamped::CubicSplineStartClamped(std::vector<double>& x,
                                                 std::vector<double>& y,
                                                 double dx0, double ddx0)
    : dx0_(dx0), ddx0_(ddx0) {
  spline_anchor_points_x_ = x;
  spline_anchor_points_y_ = y;
  anchor_points_num_ = spline_anchor_points_x_.size();
  CalculateSplineCoefs();
}

void CubicSplineStartClamped::CalculateSplineCoefs() {
  std::vector<double> h(anchor_points_num_ - 1, 0);
  for (int i = 0; i < anchor_points_num_ - 1; ++i) {
    h[i] = spline_anchor_points_x_[i + 1] - spline_anchor_points_x_[i];
  }
  spline_coefs_a_ = spline_anchor_points_y_;

  spline_coefs_c_.resize(anchor_points_num_);
  // m0
  double m0 = 0.5 * ddx0_;
  spline_coefs_c_[0] = m0;

  // m1
  assert(h[0] != 0);
  double m1 = (3.0 * ((spline_coefs_a_[1] - spline_coefs_a_[0]) / h[0]) -
               3.0 * dx0_ - 2.0 * h[0] * spline_coefs_c_[0]) /
              h[0];
  spline_coefs_c_[1] = m1;

  for (int i = 0; i < spline_coefs_c_.size() - 2; ++i) {
    assert(h[i] != 0);
    assert(h[i + 1] != 0);

    double mi1 =
        3.0 * (spline_coefs_a_[i + 2] - spline_coefs_a_[i + 1]) / h[i + 1] -
        3.0 * (spline_coefs_a_[i + 1] - spline_coefs_a_[i]) / h[i];
    double mi2 = h[i] * spline_coefs_c_[i] +
                 2 * (h[i] + h[i + 1]) * spline_coefs_c_[i + 1];
    double mi = (mi1 - mi2) / h[i + 1];
    spline_coefs_c_[i + 2] = mi;
  }

  for (int i = 0; i < anchor_points_num_ - 1; ++i) {
    double td = (spline_coefs_c_[i + 1] - spline_coefs_c_[i]) / (3.0 * h[i]);
    spline_coefs_d_.push_back(td);
    double tb =
        (spline_coefs_a_[i + 1] - spline_coefs_a_[i]) / h[i] -
        h[i] * (spline_coefs_c_[i + 1] + 2.0 * spline_coefs_c_[i]) / 3.0;
    spline_coefs_b_.push_back(tb);
  }
}

}  // namespace math
}  // namespace hqplanner