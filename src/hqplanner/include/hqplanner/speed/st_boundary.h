#ifndef HQPLANNER_SPEED_ST_BOUNDARY_H_
#define HQPLANNER_SPEED_ST_BOUNDARY_H_

#include <assert.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hqplanner/math/box2d.h"
#include "hqplanner/math/math_utils.h"
#include "hqplanner/math/polygon2d.h"
#include "hqplanner/math/vec2d.h"
#include "hqplanner/speed/st_point.h"
namespace hqplanner {
namespace speed {

class StBoundary : public hqplanner::math::Polygon2d {
 public:
  StBoundary() = default;

  explicit StBoundary(
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs);
  //   explicit StBoundary(const hqplanner::speed::StBoundary& st_boundary);
  explicit StBoundary(const hqplanner::math::Box2d& box);
  explicit StBoundary(std::vector<hqplanner::math::Vec2d> points);

  ~StBoundary() = default;

  bool IsEmpty() const { return lower_points_.empty(); }
  bool IsPointInBoundary(const STPoint& st_point) const;

  STPoint BottomLeftPoint() const;
  STPoint BottomRightPoint() const;

  StBoundary ExpandByS(const double s) const;
  StBoundary ExpandByT(const double t) const;

  // if you need to add boundary type, make sure you modify
  // GetUnblockSRange accordingly.
  enum class BoundaryType {
    UNKNOWN,
    STOP,
    FOLLOW,
    YIELD,
    OVERTAKE,
    KEEP_CLEAR,
  };

  static std::string TypeName(BoundaryType type);

  BoundaryType boundary_type() const;
  const std::string& id() const;
  double characteristic_length() const;

  void SetId(const std::string& id);
  void SetBoundaryType(const BoundaryType& boundary_type);
  void SetCharacteristicLength(const double characteristic_length);

  bool GetUnblockSRange(const double curr_time, double* s_upper,
                        double* s_lower) const;

  bool GetBoundarySRange(const double curr_time, double* s_upper,
                         double* s_lower) const;

  double min_s() const;
  double min_t() const;
  double max_s() const;
  double max_t() const;

  double Area() const;

  std::vector<STPoint> upper_points() const { return upper_points_; }
  std::vector<STPoint> lower_points() const { return lower_points_; }

  static StBoundary GenerateStBoundary(
      const std::vector<STPoint>& lower_points,
      const std::vector<STPoint>& upper_points);

  StBoundary CutOffByT(const double t) const;

 private:
  bool IsValid(
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const;

  bool IsPointNear(const hqplanner::math::LineSegment2d& seg,
                   const hqplanner::math::Vec2d& point, const double max_dist);

  //   FRIEND_TEST(StBoundaryTest, remove_redundant_points);
  void RemoveRedundantPoints(
      std::vector<std::pair<STPoint, STPoint>>* point_pairs);

  //   FRIEND_TEST(StBoundaryTest, get_index_range);
  bool GetIndexRange(const std::vector<STPoint>& points, const double t,
                     size_t* left, size_t* right) const;

 private:
  BoundaryType boundary_type_ = BoundaryType::UNKNOWN;

  std::vector<STPoint> upper_points_;
  std::vector<STPoint> lower_points_;

  double area_ = 0.0;

  std::string id_;
  double characteristic_length_ = 1.0;
  double s_high_limit_ = 200.0;
  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();
};

}  // namespace speed
}  // namespace hqplanner

#endif
