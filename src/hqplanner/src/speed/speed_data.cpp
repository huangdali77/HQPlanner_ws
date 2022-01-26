#include "hqplanner/speed/speed_data.h"
namespace hqplanner {
namespace speed {
using hqplanner::forproto::SpeedPoint;

SpeedData::SpeedData(std::vector<SpeedPoint> speed_points)
    : speed_vector_(std::move(speed_points)) {}

void SpeedData::AppendSpeedPoint(const double s, const double time,
                                 const double v, const double a,
                                 const double da) {
  if (!speed_vector_.empty()) {
    assert(speed_vector_.back().t < time);
    // CHECK(speed_vector_.back().t < time);
  }
  SpeedPoint speed_point;
  speed_point.s = s;
  speed_point.t = time;
  speed_point.v = v;
  speed_point.a = a;
  speed_point.da = da;

  speed_vector_.push_back(std::move(speed_point));
}

const std::vector<SpeedPoint>& SpeedData::speed_vector() const {
  return speed_vector_;
}

void SpeedData::set_speed_vector(std::vector<SpeedPoint> speed_points) {
  speed_vector_ = std::move(speed_points);
}

bool SpeedData::EvaluateByTime(const double t,
                               SpeedPoint* const speed_point) const {
  if (speed_vector_.size() < 2) {
    return false;
  }
  if (!(speed_vector_.front().t < t + 1.0e-6 &&
        t - 1.0e-6 < speed_vector_.back().t)) {
    return false;
  }

  auto comp = [](const SpeedPoint& sp, const double t) { return sp.t < t; };

  auto it_lower =
      std::lower_bound(speed_vector_.begin(), speed_vector_.end(), t, comp);
  if (it_lower == speed_vector_.end()) {
    *speed_point = speed_vector_.back();
  } else if (it_lower == speed_vector_.begin()) {
    *speed_point = speed_vector_.front();
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double t0 = p0.t;
    double t1 = p1.t;

    double s = hqplanner::math::lerp(p0.s, t0, p1.s, t1, t);
    double v = hqplanner::math::lerp(p0.v, t0, p1.v, t1, t);
    double a = hqplanner::math::lerp(p0.a, t0, p1.a, t1, t);
    double j = hqplanner::math::lerp(p0.da, t0, p1.da, t1, t);

    SpeedPoint speed_point_temp;
    speed_point_temp.s = s;
    speed_point_temp.t = t;
    speed_point_temp.v = v;
    speed_point_temp.a = a;
    speed_point_temp.da = j;

    *speed_point = std::move(speed_point_temp);
  }
  return true;
}

double SpeedData::TotalTime() const {
  if (speed_vector_.empty()) {
    return 0.0;
  }
  return speed_vector_.back().t - speed_vector_.front().t;
}

void SpeedData::Clear() { speed_vector_.clear(); }

}  // namespace speed
}  // namespace hqplanner
