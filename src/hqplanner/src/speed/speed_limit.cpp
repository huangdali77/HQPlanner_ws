#include "hqplanner/speed/speed_limit.h"

#include <assert.h>

#include <algorithm>
namespace hqplanner {
namespace speed {

void SpeedLimit::AppendSpeedLimit(const double s, const double v) {
  if (!speed_limit_points_.empty()) {
    assert(s >= speed_limit_points_.back().first);
    // DCHECK_GE(s, speed_limit_points_.back().first);
  }
  speed_limit_points_.emplace_back(s, v);
}

const std::vector<std::pair<double, double>>& SpeedLimit::speed_limit_points()
    const {
  return speed_limit_points_;
}

double SpeedLimit::GetSpeedLimitByS(const double s) const {
  assert(speed_limit_points_.size() >= 2);
  assert(s >= speed_limit_points_.front().first);
  // DCHECK_GE(speed_limit_points_.size(), 2);
  // DCHECK_GE(s, speed_limit_points_.front().first);

  auto compare_s = [](const std::pair<double, double>& point, const double s) {
    return point.first < s;
  };

  auto it_lower = std::lower_bound(speed_limit_points_.begin(),
                                   speed_limit_points_.end(), s, compare_s);

  if (it_lower == speed_limit_points_.end()) {
    return (it_lower - 1)->second;
  }
  return it_lower->second;
}

void SpeedLimit::Clear() { speed_limit_points_.clear(); }

}  // namespace speed
}  // namespace hqplanner
